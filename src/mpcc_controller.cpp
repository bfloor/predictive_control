
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/mpcc_controller.h>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MPCC::~MPCC()
{
    clearDataMember();
}

void MPCC::spinNode()
{
    ROS_INFO(" Predictive control node is running, now it's 'Spinning Node'");
    ros::spin();
}

// disallocated memory
void MPCC::clearDataMember()
{
    last_position_ = Eigen::VectorXd(3);
    last_velocity_ = Eigen::VectorXd(3);

}

// initialize all helper class of predictive control and subscibe joint state and publish controlled joint velocity
bool MPCC::initialize()
{
    // make sure node is still running
    if (ros::ok())
    {
        // initialize helper classes, make sure pd_config should be initialized first as mother of other class
        controller_config_.reset(new predictive_configuration());
        bool controller_config_success = controller_config_->initialize();

        bool kinematic_success = true;

        if (controller_config_success == false)
         {
            ROS_ERROR("MPCC: FAILED TO INITILIZED!!");
            std::cout << "States: \n"
                                << " pd_config: " << std::boolalpha << controller_config_success << "\n"                       
                                << " pd config init success: " << std::boolalpha << controller_config_->initialize_success_
                                << std::endl;
            return false;
        }

        // initialize data member of class
        clock_frequency_ = controller_config_->clock_frequency_;

        //DEBUG
        activate_debug_output_ = controller_config_->activate_debug_output_;
        publish_feedback_ = controller_config_->publish_feedback_;
        tracking_ = true;
        move_action_result_.reach = false;
        plotting_result_ = controller_config_->plotting_result_;

        // DEBUG
        if (controller_config_->activate_controller_node_output_)
        {
            ROS_WARN("===== DEBUG INFO ACTIVATED =====");
        }

        // resize position and velocity velocity vectors
        current_state_ = Eigen::Vector3d(0,0,0);
        last_state_ = Eigen::Vector3d(0,0,0);
		prev_pose_ = Eigen::Vector3d(0,0,0);
		goal_pose_ = Eigen::Vector3d(0,0,0);
		next_pose_= Eigen::Vector3d(0,0,0);
		prev_pose_.setZero();
        goal_pose_.setZero();

        // ros interfaces
        static const std::string MOVE_ACTION_NAME = "move_action";
        move_action_server_.reset(new actionlib::SimpleActionServer<predictive_control::moveAction>(nh, MOVE_ACTION_NAME, false));
        move_action_server_->registerGoalCallback(boost::bind(&MPCC::moveGoalCB, this));
        move_action_server_->registerPreemptCallback(boost::bind(&MPCC::movePreemptCB, this));
        move_action_server_->start();

	    // MOVEIT interfaces
	    static const std::string MOVEIT_ACTION_NAME = "fake_base_controller";
	    moveit_action_server_.reset(new actionlib::SimpleActionServer<predictive_control::trajAction>(nh, MOVEIT_ACTION_NAME, false));
	    moveit_action_server_->registerGoalCallback(boost::bind(&MPCC::moveitGoalCB, this));
	    moveit_action_server_->start();

        robot_state_sub_ = nh.subscribe(controller_config_->robot_state_topic_, 1, &MPCC::StateCallBack, this);

        //Publishers
        traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory",1);
		pred_cmd_pub_ = nh.advertise<nav_msgs::Path>("predicted_cmd",1);
		cost_pub_ = nh.advertise<std_msgs::Float64>("cost",1);
        contour_error_pub_ = nh.advertise<std_msgs::Float64MultiArray>("contour_error",1);
        controlled_velocity_pub_ = nh.advertise<geometry_msgs::Twist>(controller_config_->output_cmd,1);
		joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
        pred_traj_pub_ = nh.advertise<nav_msgs::Path>("predicted_trajectory",1);
		global_plan_pub_ = nh.advertise<visualization_msgs::MarkerArray>("global_plan",1);
        local_spline_traj_pub1_ = nh.advertise<nav_msgs::Path>("reference_trajectory_seg1",1);
        local_spline_traj_pub2_ = nh.advertise<nav_msgs::Path>("reference_trajectory_seg2",1);
        local_spline_traj_pub3_ = nh.advertise<nav_msgs::Path>("reference_trajectory_seg3",1);
		feedback_pub_ = nh.advertise<predictive_control::control_feedback>("controller_feedback",1);

		ros::Duration(1).sleep();

        timer_ = nh.createTimer(ros::Duration(1/clock_frequency_), &MPCC::runNode, this);
        timer_.start();

		//Initialize trajectory variables
		next_point_dist = 0;
		goal_dist = 0;
		prev_point_dist = 0;
		idx = 1;
		idy = 1;
		epsilon_ = 0.01;
		goal_reached_ = false;
        last_poly_ = false;

		moveit_msgs::RobotTrajectory j;
		traj = j;

		//initialize trajectory variable to plot prediction trajectory
        local_spline_traj1_.poses.resize(50);
        local_spline_traj2_.poses.resize(50);
        local_spline_traj3_.poses.resize(50);

		pred_traj_.poses.resize(ACADO_N);
		pred_cmd_.poses.resize(ACADO_N);
		pred_traj_.header.frame_id = controller_config_->tracking_frame_;
		for(int i=0;i < ACADO_N; i++)
		{
			pred_traj_.poses[i].header.frame_id = controller_config_->tracking_frame_;
		}

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

        // initialize state and control weight factors
        cost_contour_weight_factors_ = transformStdVectorToEigenVector(controller_config_->contour_weight_factors_);
        cost_control_weight_factors_ = transformStdVectorToEigenVector(controller_config_->control_weight_factors_);
        reference_velocity_ = controller_config_->reference_velocity_;

        ros::NodeHandle nh_predictive("predictive_controller");

        /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
        reconfigure_server_.reset(new dynamic_reconfigure::Server<predictive_control::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
        reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));

		//Controller options
		enable_output_ = false;
		n_iterations_ = 100;
		simulation_mode_ = true;

        global_plan.type = visualization_msgs::Marker::CYLINDER;
        global_plan.id = 800;
        global_plan.color.r = 0.8;
        global_plan.color.g = 0.0;
        global_plan.color.b = 0.0;
        global_plan.color.a = 0.8;
        global_plan.header.frame_id = controller_config_->tracking_frame_;
        global_plan.ns = "trajectory";
        global_plan.action = visualization_msgs::Marker::ADD;
        global_plan.lifetime = ros::Duration(0);
        global_plan.scale.x = 0.1;
        global_plan.scale.y = 0.1;
        global_plan.scale.z = 0.05;

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

//		// MPCC reference path variables
//		X_global.resize(controller_config_->ref_x_.size());
//		Y_global.resize(controller_config_->ref_y_.size());
//		Theta_global.resize(controller_config_->ref_theta_.size());

		// Check if all reference vectors are of the same length
		if (!( (controller_config_->ref_x_.size() == controller_config_->ref_y_.size()) && ( controller_config_->ref_x_.size() == controller_config_->ref_theta_.size() ) && (controller_config_->ref_y_.size() == controller_config_->ref_theta_.size()) ))
        {
            ROS_ERROR("Reference path inputs should be of equal length");
        }


		// Initialize global reference path
        referencePath.SetGlobalPath(controller_config_->ref_x_, controller_config_->ref_y_, controller_config_->ref_theta_);
        referencePath.PrintGlobalPath();    // Print global reference path
        seg_i = 0;                          // Initialize segment counter
        n_traj_per_cloth = controller_config_->n_poly_per_clothoid_;

        ROS_WARN("PREDICTIVE CONTROL INTIALIZED!!");
		return true;
	}
	else
	{
		ROS_ERROR("MPCC: Failed to initialize as ROS Node is shoutdown");
		return false;
	}
}

void MPCC::reconfigureCallback(predictive_control::PredictiveControllerConfig& config, uint32_t level){

    ROS_INFO("reconfigure callback!");
    cost_contour_weight_factors_(0) = config.Wcontour;
    cost_contour_weight_factors_(1) = config.Wlag;
    cost_control_weight_factors_(0) = config.Kv;
    cost_control_weight_factors_(1) = config.Kw;

    reference_velocity_ = config.vRef;

    enable_output_ = config.enable_output;
    loop_mode_ = config.loop_mode;
    n_iterations_ = config.n_iterations;
    simulation_mode_ = config.simulation_mode;

    //Search window parameters
    window_size_ = config.window_size;
    n_search_points_ = config.n_search_points;
}

void MPCC::broadcastPathPose(){

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = controller_config_->tracking_frame_;
	transformStamped.child_frame_id = "path";

	transformStamped.transform.translation.x = referencePath.ref_path_x(acadoVariables.x[3]);
	transformStamped.transform.translation.y = referencePath.ref_path_y(acadoVariables.x[3]);
	transformStamped.transform.translation.z = 0.0;
	tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
	transformStamped.transform.rotation.x = 0;
	transformStamped.transform.rotation.y = 0;
	transformStamped.transform.rotation.z = 0;
	transformStamped.transform.rotation.w = 1;


	path_pose_pub_.sendTransform(transformStamped);

}

void MPCC::broadcastTF(){

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = controller_config_->tracking_frame_;
	transformStamped.child_frame_id = controller_config_->robot_base_link_;
	if(!enable_output_){
		transformStamped.transform.translation.x = current_state_(0);
		transformStamped.transform.translation.y = current_state_(1);
		transformStamped.transform.translation.z = 0.0;
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();
	}

	else{
		transformStamped.transform.translation.x = pred_traj_.poses[1].pose.position.x;
		transformStamped.transform.translation.y = pred_traj_.poses[1].pose.position.y;
		transformStamped.transform.translation.z = 0.0;

		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();
	}

	state_pub_.sendTransform(transformStamped);

	sensor_msgs::JointState empty;
	empty.position.resize(4);
	empty.name ={"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
	empty.header.stamp = ros::Time::now();
	joint_state_pub_.publish(empty);
}

// update this function 1/clock_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{
    int N_iter;
    acado_timer t;
    acado_tic( &t );

    acado_initializeSolver( );

    int traj_n = traj.multi_dof_joint_trajectory.points.size();
	if(!simulation_mode_)
		broadcastTF();
    if (traj_n>0) {
        acadoVariables.x[0] = current_state_(0);
        acadoVariables.x[1] = current_state_(1);
        acadoVariables.x[2] = current_state_(2);

        acadoVariables.u[0] = controlled_velocity_.linear.x;
        acadoVariables.u[1] = controlled_velocity_.angular.z;

		if(acadoVariables.x[3] > ss[2]) {

		    if (seg_i + N_local > referencePath.GlobalPathLenght()*n_traj_per_cloth){
		        goal_reached_ = true;
                ROS_ERROR_STREAM("GOAL REACHED");
                if (loop_mode_)
                {
                    seg_i = 0;
                    goal_reached_ = false;
                    last_poly_ = false;
                    acadoVariables.x[3] = referencePath.GetS0();
                    ROS_ERROR_STREAM("LOOP STARTED");
                }
            } else{
			    seg_i++;
                referencePath.UpdateLocalRefPath(seg_i, ss, xx, yy, vv);
                acadoVariables.x[3] = referencePath.GetS0();
                ROS_ERROR_STREAM("SWITCH SPLINE, seg_i =  " << seg_i);
		    }
        }

        if(idx == 1) {
            double smin;
            smin = referencePath.ClosestPointOnPath(current_state_, ss[1], 100, acadoVariables.x[3], window_size_, n_search_points_);
            acadoVariables.x[3] = smin;
        }
        else
            acadoVariables.x[3] = acadoVariables.x[3];


        for (N_iter = 0; N_iter < ACADO_N; N_iter++) {

            // Initialize Online Data variables
            acadoVariables.od[(ACADO_NOD * N_iter) + 0 ] = referencePath.ref_path_x.m_a[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 1 ] = referencePath.ref_path_x.m_b[1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 2 ] = referencePath.ref_path_x.m_c[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 3 ] = referencePath.ref_path_x.m_d[1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 4 ] = referencePath.ref_path_y.m_a[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 5 ] = referencePath.ref_path_y.m_b[1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 6 ] = referencePath.ref_path_y.m_c[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 7 ] = referencePath.ref_path_y.m_d[1];

            acadoVariables.od[(ACADO_NOD * N_iter) + 8 ] = referencePath.ref_path_x.m_a[2];         // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 9 ] = referencePath.ref_path_x.m_b[2];
            acadoVariables.od[(ACADO_NOD * N_iter) + 10] = referencePath.ref_path_x.m_c[2];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 11] = referencePath.ref_path_x.m_d[2];
            acadoVariables.od[(ACADO_NOD * N_iter) + 12] = referencePath.ref_path_y.m_a[2];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 13] = referencePath.ref_path_y.m_b[2];
            acadoVariables.od[(ACADO_NOD * N_iter) + 14] = referencePath.ref_path_y.m_c[2];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 15] = referencePath.ref_path_y.m_d[2];

            acadoVariables.od[(ACADO_NOD * N_iter) + 16] = referencePath.ref_path_x.m_a[3];         // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 17] = referencePath.ref_path_x.m_b[3];
            acadoVariables.od[(ACADO_NOD * N_iter) + 18] = referencePath.ref_path_x.m_c[3];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 19] = referencePath.ref_path_x.m_d[3];
            acadoVariables.od[(ACADO_NOD * N_iter) + 20] = referencePath.ref_path_y.m_a[3];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 21] = referencePath.ref_path_y.m_b[3];
            acadoVariables.od[(ACADO_NOD * N_iter) + 22] = referencePath.ref_path_y.m_c[3];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 23] = referencePath.ref_path_y.m_d[3];

			acadoVariables.od[(ACADO_NOD * N_iter) + 24] = cost_contour_weight_factors_(0);       // weight factor on contour error
			acadoVariables.od[(ACADO_NOD * N_iter) + 25] = cost_contour_weight_factors_(1);       // weight factor on lag error
			acadoVariables.od[(ACADO_NOD * N_iter) + 26] = cost_control_weight_factors_(0);      // weight factor on theta
			acadoVariables.od[(ACADO_NOD * N_iter) + 27] = cost_control_weight_factors_(1);      // weight factor on v

            acadoVariables.od[(ACADO_NOD * N_iter) + 28 ] = ss[1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 29 ] = ss[2];
            acadoVariables.od[(ACADO_NOD * N_iter) + 30 ] = ss[3];

            acadoVariables.od[(ACADO_NOD * N_iter) + 31] = vv[0]*reference_velocity_;
            acadoVariables.od[(ACADO_NOD * N_iter) + 32] = vv[1]*reference_velocity_;
            acadoVariables.od[(ACADO_NOD * N_iter) + 33] = vv[2]*reference_velocity_;

            acadoVariables.od[(ACADO_NOD * N_iter) + 34] = ss[2] + 0.02;
            acadoVariables.od[(ACADO_NOD * N_iter) + 35] = ss[3] + 0.02;
}

        acadoVariables.x0[ 0 ] = current_state_(0);
        acadoVariables.x0[ 1 ] = current_state_(1);
        acadoVariables.x0[ 2 ] = current_state_(2);
		acadoVariables.x0[ 3 ] = acadoVariables.x[3];


        acado_preparationStep();

        acado_feedbackStep();

//        printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());

		int j=1;
        while (acado_getKKT()> 1e-3 && j<n_iterations_){ //  && acado_getKKT() < 100

			acado_preparationStep();

            acado_feedbackStep();

//            printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());
			j++;    //        acado_printDifferentialVariables();
        }

        te_ = acado_toc(&t);

		controlled_velocity_.linear.x = acadoVariables.u[0];
		controlled_velocity_.angular.z = acadoVariables.u[1];

        publishPredictedTrajectory();
		publishPredictedOutput();
		publishLocalRefPath();
		broadcastPathPose();
        publishContourError();
		cost_.data = acado_getObjective();
		publishCost();

//		ROS_INFO_STREAM("Solve time " << te_ * 1e6 << " us");

    // publish zero controlled velocity
        if (!tracking_)
        {
            actionSuccess();
        }
	}
    if(!enable_output_ || acado_getKKT() > 1e-3) {
		publishZeroJointVelocity();
		idx = 2; // used to keep computation of the mpc and did not let it move because we set the initial state as the predicted state
	}
	else {
		idx=1;
		controlled_velocity_pub_.publish(controlled_velocity_);
	}

}

void MPCC::moveGoalCB()
{
//    ROS_INFO("MOVEGOALCB");
    if(move_action_server_->isNewGoalAvailable())
    {
        boost::shared_ptr<const predictive_control::moveGoal> move_action_goal_ptr = move_action_server_->acceptNewGoal();
        tracking_ = false;

        //erase previous trajectory
        for (auto it = traj_marker_array_.markers.begin(); it != traj_marker_array_.markers.end(); ++it)
        {
            it->action = visualization_msgs::Marker::DELETE;
            traj_pub_.publish(traj_marker_array_);
        }

        traj_marker_array_.markers.clear();
    }
}

void MPCC::moveitGoalCB()
{
    ROS_INFO_STREAM("Got new MoveIt goal!!!");
    //Reset trajectory index
    idx = 1;
    if(moveit_action_server_->isNewGoalAvailable())
    {
        boost::shared_ptr<const predictive_control::trajGoal> moveit_action_goal_ptr = moveit_action_server_->acceptNewGoal();
        traj = moveit_action_goal_ptr->trajectory;
        tracking_ = false;

        int traj_n = traj.multi_dof_joint_trajectory.points.size();
        goal_pose_(0) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.x;
        goal_pose_(1) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.y;
        goal_pose_(2) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].rotation.z;
        
        acado_initializeSolver( );

        seg_i = 0;

		goal_reached_ = false;
		last_poly_ = false;

		// Initialize local reference path
        referencePath.InitLocalRefPath(3,2,ss,xx,yy,vv);
        referencePath.PrintLocalPath(ss,xx,yy);     // Print local reference path

		publishLocalRefPath();             // Publish local reference path for visualization
        publishGlobalPlan();                        // Publish global reference path for visualization
    }
}

void MPCC::movePreemptCB()
{
    move_action_result_.reach = true;
    move_action_server_->setPreempted(move_action_result_, "Action has been preempted");
    tracking_ = true;
}

void MPCC::actionSuccess()
{
    move_action_server_->setSucceeded(move_action_result_, "Goal succeeded!");
    tracking_ = true;
}

void MPCC::actionAbort()
{
    move_action_server_->setAborted(move_action_result_, "Action has been aborted");
    tracking_ = true;
}

// read current position and velocity of robot joints
void MPCC::StateCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
    if (activate_debug_output_)
    {
//        ROS_INFO("MPCC::StateCallBack");
    }
    last_state_ = current_state_;
    current_state_(0) =    msg->position.x;
    current_state_(1) =    msg->position.y;
    current_state_(2) =    msg->orientation.z;
}

void MPCC::publishZeroJointVelocity()
{
    if (activate_debug_output_)
    {
        ROS_INFO("Publishing ZERO joint velocity!!");
    }

    geometry_msgs::Twist pub_msg;

	if(!simulation_mode_)
		broadcastTF();

    controlled_velocity_ = pub_msg;
    controlled_velocity_pub_.publish(controlled_velocity_);
}

void MPCC::publishGlobalPlan(void)
{
    visualization_msgs::MarkerArray plan;

    std::vector<double> X_global, Y_global;

    referencePath.GetGlobalPath(X_global,Y_global);

    for (int i = 0; i < X_global.size(); i++)
    {
        global_plan.id = 800+i;
        global_plan.pose.position.x = X_global[i];
        global_plan.pose.position.y = Y_global[i];
        global_plan.pose.orientation.x = 0;
        global_plan.pose.orientation.y = 0;
        global_plan.pose.orientation.z = 0;
        global_plan.pose.orientation.w = 1;
        plan.markers.push_back(global_plan);
    }

    global_plan_pub_.publish(plan);
}

void MPCC::publishLocalRefPath(void)
{
    local_spline_traj1_.header.stamp = ros::Time::now();
    local_spline_traj2_.header.stamp = ros::Time::now();
    local_spline_traj3_.header.stamp = ros::Time::now();

    local_spline_traj1_.header.frame_id = controller_config_->tracking_frame_;
    local_spline_traj2_.header.frame_id = controller_config_->tracking_frame_;
    local_spline_traj3_.header.frame_id = controller_config_->tracking_frame_;

	double s1,s2,s3;
    int j=0;
	for (int i = 0; i < 50; i++)
	{
        s1= i*(ss[2] - ss[1])/50.0;
        s2= i*(ss[3] - ss[2])/50.0;
        s3= i*(ss[4] - ss[3])/50.0;

        local_spline_traj1_.poses[i].pose.position.x = referencePath.ref_path_x.m_a[1]*s1*s1*s1+referencePath.ref_path_x.m_b[1]*s1*s1+referencePath.ref_path_x.m_c[1]*s1+referencePath.ref_path_x.m_d[1]; //x
        local_spline_traj1_.poses[i].pose.position.y = referencePath.ref_path_y.m_a[1]*s1*s1*s1+referencePath.ref_path_y.m_b[1]*s1*s1+referencePath.ref_path_y.m_c[1]*s1+referencePath.ref_path_y.m_d[1]; //y
        local_spline_traj1_.poses[i].header.stamp = ros::Time::now();
        local_spline_traj1_.poses[i].header.frame_id = controller_config_->tracking_frame_;

        local_spline_traj2_.poses[i].pose.position.x = referencePath.ref_path_x.m_a[2]*s2*s2*s2+referencePath.ref_path_x.m_b[2]*s2*s2+referencePath.ref_path_x.m_c[2]*s2+referencePath.ref_path_x.m_d[2]; //x
        local_spline_traj2_.poses[i].pose.position.y = referencePath.ref_path_y.m_a[2]*s2*s2*s2+referencePath.ref_path_y.m_b[2]*s2*s2+referencePath.ref_path_y.m_c[2]*s2+referencePath.ref_path_y.m_d[2]; //y
        local_spline_traj2_.poses[i].header.stamp = ros::Time::now();
        local_spline_traj2_.poses[i].header.frame_id = controller_config_->tracking_frame_;

        local_spline_traj3_.poses[i].pose.position.x = referencePath.ref_path_x.m_a[3]*s3*s3*s3+referencePath.ref_path_x.m_b[3]*s3*s3+referencePath.ref_path_x.m_c[3]*s3+referencePath.ref_path_x.m_d[3]; //x
        local_spline_traj3_.poses[i].pose.position.y = referencePath.ref_path_y.m_a[3]*s3*s3*s3+referencePath.ref_path_y.m_b[3]*s3*s3+referencePath.ref_path_y.m_c[3]*s3+referencePath.ref_path_y.m_d[3]; //y
        local_spline_traj3_.poses[i].header.stamp = ros::Time::now();
        local_spline_traj3_.poses[i].header.frame_id = controller_config_->tracking_frame_;
	}

    local_spline_traj_pub1_.publish(local_spline_traj1_);
    local_spline_traj_pub2_.publish(local_spline_traj2_);
    local_spline_traj_pub3_.publish(local_spline_traj3_);
}

void MPCC::publishPredictedTrajectory(void)
{
    for (int i = 0; i < ACADO_N; i++)
    {
        pred_traj_.poses[i].pose.position.x = acadoVariables.x[i * ACADO_NX + 0]; //x
        pred_traj_.poses[i].pose.position.y = acadoVariables.x[i * ACADO_NX + 1]; //y
		pred_traj_.poses[i].pose.orientation.z = acadoVariables.x[i * ACADO_NX + 2]; //theta
    }

	pred_traj_pub_.publish(pred_traj_);
}

void MPCC::publishPredictedOutput(void)
{
	for (int i = 0; i < ACADO_N; i++)
	{
		pred_cmd_.poses[i].pose.position.x = acadoVariables.u[i + 0]; //x
		pred_cmd_.poses[i].pose.position.y = acadoVariables.u[i + 1]; //y
	}

	pred_cmd_pub_.publish(pred_cmd_);
}

void MPCC::publishCost(void){

	cost_pub_.publish(cost_);
}

void MPCC::publishContourError(void){

    // Compute contour and lag error to publish
    double x_path, y_path, dx_path, dy_path, abs_grad, dx_path_norm, dy_path_norm;

    x_path = (referencePath.ref_path_x.m_a[seg_i]*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_x.m_b[seg_i]*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_x.m_c[seg_i]*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_x.m_d[seg_i]);
    y_path = (referencePath.ref_path_y.m_a[seg_i]*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_y.m_b[seg_i]*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_y.m_c[seg_i]*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_y.m_d[seg_i]);
    dx_path = (3*referencePath.ref_path_x.m_a[seg_i]*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i]) + 2*referencePath.ref_path_x.m_b[seg_i]*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_x.m_c[seg_i]);
    dy_path = (3*referencePath.ref_path_y.m_a[seg_i]*(acadoVariables.x[3]-ss[seg_i])*(acadoVariables.x[3]-ss[seg_i]) + 2*referencePath.ref_path_y.m_b[seg_i]*(acadoVariables.x[3]-ss[seg_i]) + referencePath.ref_path_y.m_c[seg_i]);

    abs_grad = sqrt(pow(dx_path,2) + pow(dy_path,2));

    dx_path_norm = dx_path/abs_grad;
    dy_path_norm = dy_path/abs_grad;

    contour_error_ =  dy_path_norm * (acadoVariables.x[0] - x_path) - dx_path_norm * (acadoVariables.x[1] - y_path);
    lag_error_ = -dx_path_norm * (acadoVariables.x[0] - x_path) - dy_path_norm * (acadoVariables.x[1] - y_path);

    std_msgs::Float64MultiArray errors;

    errors.data.resize(2);

    errors.data[0] = contour_error_;
    errors.data[1] = lag_error_;

    contour_error_pub_.publish(errors);
}

void MPCC::publishFeedback(int& it, double& time)
{

    predictive_control::control_feedback feedback_msg;

    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.header.frame_id = controller_config_->tracking_frame_;

    feedback_msg.cost = cost_.data;
    feedback_msg.iterations = it;
    feedback_msg.computation_time = time;
    feedback_msg.kkt = acado_getKKT();

    feedback_msg.wC = cost_contour_weight_factors_(0);       // weight factor on contour error
    feedback_msg.wL = cost_contour_weight_factors_(1);       // weight factor on lag error
    feedback_msg.wV = cost_control_weight_factors_(0);       // weight factor on theta
    feedback_msg.wW = cost_control_weight_factors_(1);

    // Compute contour errors
    feedback_msg.contour_errors.data.resize(2);

    feedback_msg.contour_errors.data[0] = contour_error_;
    feedback_msg.contour_errors.data[1] = lag_error_;

//    feedback_msg.reference_path = spline_traj2_;
    feedback_msg.prediction_horizon = pred_traj_;
    feedback_msg.prediction_horizon.poses[0].pose.position.z = acadoVariables.x[3];
    feedback_msg.computed_control = controlled_velocity_;

    feedback_msg.enable_output = enable_output_;

    feedback_msg.vRef = reference_velocity_;

    //Search window parameters
    feedback_msg.window = window_size_;
    feedback_msg.search_points = n_search_points_;

    feedback_pub_.publish(feedback_msg);
}