
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

        obstacle_feed_sub_ = nh.subscribe(controller_config_->sub_ellipse_topic_, 1, &MPCC::ObstacleCallBack, this);

        //Publishers
        traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory",1);
		pred_cmd_pub_ = nh.advertise<nav_msgs::Path>("predicted_cmd",1);
        tr_path_pub_ = nh.advertise<nav_msgs::Path>("horizon",1);
		cost_pub_ = nh.advertise<std_msgs::Float64>("cost",1);
        controlled_velocity_pub_ = nh.advertise<geometry_msgs::Twist>(controller_config_->output_cmd,1);
		joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
		robot_collision_space_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/robot_collision_space", 100);
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

		moveit_msgs::RobotTrajectory j;
		traj = j;

		//initialize trajectory variable to plot prediction trajectory
		spline_traj_.poses.resize(ACADO_N*2);
		spline_traj2_.poses.resize(ACADO_N);
		pred_traj_.poses.resize(ACADO_N);
		pred_cmd_.poses.resize(ACADO_N);
		pred_traj_.header.frame_id = "odom";
		for(int i=0;i < ACADO_N; i++)
		{
			pred_traj_.poses[i].header.frame_id = "odom";
		}

		pred_traj_pub_ = nh.advertise<nav_msgs::Path>("mpc_horizon",1);
		spline_traj_pub_ = nh.advertise<nav_msgs::Path>("spline_traj",1);
		spline_traj_pub2_ = nh.advertise<nav_msgs::Path>("spline_traj2",1);

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

        // initialize state and control weight factors
        cost_state_weight_factors_ = transformStdVectorToEigenVector(controller_config_->lsq_state_weight_factors_);
        cost_control_weight_factors_ = transformStdVectorToEigenVector(controller_config_->lsq_control_weight_factors_);
        slack_weight_ = controller_config_->slack_weight_;
        repulsive_weight_ = controller_config_->repulsive_weight_;
        cost_state_terminal_weight_factors_ = transformStdVectorToEigenVector(controller_config_->lsq_state_terminal_weight_factors_);

        ros::NodeHandle nh_predictive("predictive_controller");

        /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
        reconfigure_server_.reset(new dynamic_reconfigure::Server<predictive_control::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
        reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));

	    // Initialize obstacles
        obstacle_feed::Obstacles obstacles;
		obstacles.Obstacles.resize(controller_config_->n_obstacles_);
		for (int obst_it = 0; obst_it < controller_config_->n_obstacles_; obst_it++)
        {
            obstacles.Obstacles[obst_it].pose.position.x = 1000;
            obstacles.Obstacles[obst_it].pose.position.y = 1000;
            obstacles.Obstacles[obst_it].pose.orientation.z = 0;
            obstacles.Obstacles[obst_it].major_semiaxis = 0.001;
            obstacles.Obstacles[obst_it].minor_semiaxis = 0.001;
		}
		obstacles_ = obstacles;

		computeEgoDiscs();

		//Controller options
		enable_output_ = true;
		n_iterations_ = 1;
		simulation_mode_ = true;

		//Plot variables
		ellips1.type = visualization_msgs::Marker::CYLINDER;
		ellips1.id = 60;
		ellips1.color.b = 1.0;
		ellips1.color.a = 0.5;
		ellips1.header.frame_id = controller_config_->tracking_frame_;
		ellips1.ns = "trajectory";
		ellips1.action = visualization_msgs::Marker::ADD;
		ellips1.lifetime = ros::Duration(0.1);
		ellips1.scale.x = r_discs_*2.0;
		ellips1.scale.y = r_discs_*2.0;
		ellips1.scale.z = 0.05;

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

		//MPCC variables
		X.resize(3);
		Y.resize(3);
		S.resize(3);
		traj_i =0;
		ROS_WARN("PREDICTIVE CONTROL INTIALIZED!!");
		return true;
	}
	else
	{
		ROS_ERROR("MPCC: Failed to initialize as ROS Node is shoutdown");
		return false;
	}
}

void MPCC::computeEgoDiscs()
{
    // Collect parameters for disc representation
    int n_discs = controller_config_->n_discs_;
    double length = controller_config_->ego_l_;
    double width = controller_config_->ego_w_;

    // Initialize positions of discs
    x_discs_.resize(n_discs);

    // Loop over discs and assign positions
    for ( int discs_it = 0; discs_it < n_discs; discs_it++){
        x_discs_[discs_it] = -length/2 + (discs_it + 1)*(length/(n_discs + 1));
    }

    // Compute radius of the discs
    r_discs_ = sqrt(pow(x_discs_[n_discs - 1] - length/2,2) + pow(width/2,2));
    ROS_WARN_STREAM("Generated " << n_discs <<  " ego-vehicle discs with radius " << r_discs_ );
}

void MPCC::broadcastTF(){

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "odom";
	transformStamped.child_frame_id = controller_config_->robot_base_link_;
	if(!enable_output_){
		transformStamped.transform.translation.x = current_state_(0);
		transformStamped.transform.translation.y = current_state_(1);
		transformStamped.transform.translation.z = 0.0;
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
		transformStamped.transform.rotation.x = 0;
		transformStamped.transform.rotation.y = 0;
		transformStamped.transform.rotation.z = 0;
		transformStamped.transform.rotation.w = 1;
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

    obstacle_feed::Obstacles obstacles = obstacles_;

    acado_initializeSolver( );

    int traj_n = traj.multi_dof_joint_trajectory.points.size();
	if(!simulation_mode_)
		broadcastTF();
    if (traj_n > 0) {
        acadoVariables.x[0] = current_state_(0);
        acadoVariables.x[1] = current_state_(1);
        acadoVariables.x[2] = current_state_(2);
        acadoVariables.x[3] = acadoVariables.x[ACADO_NX+3];

        acadoVariables.u[0] = controlled_velocity_.linear.x;
        acadoVariables.u[1] = controlled_velocity_.angular.z;

        for (N_iter = 0; N_iter < ACADO_N; N_iter++) {

            // Initialize Online Data variables
            acadoVariables.od[(ACADO_NOD * N_iter) + 0] = ref_path_x.m_a[traj_i];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 1] = ref_path_x.m_b[traj_i];
			acadoVariables.od[(ACADO_NOD * N_iter) + 2] = ref_path_x.m_c[traj_i];        // spline coefficients
			acadoVariables.od[(ACADO_NOD * N_iter) + 3] = ref_path_x.m_d[traj_i];
			acadoVariables.od[(ACADO_NOD * N_iter) + 4] = ref_path_y.m_a[traj_i];        // spline coefficients
			acadoVariables.od[(ACADO_NOD * N_iter) + 5] = ref_path_y.m_b[traj_i];
			acadoVariables.od[(ACADO_NOD * N_iter) + 6] = ref_path_y.m_c[traj_i];        // spline coefficients
			acadoVariables.od[(ACADO_NOD * N_iter) + 7] = ref_path_y.m_d[traj_i];
			acadoVariables.od[(ACADO_NOD * N_iter) + 8 ] = cost_state_weight_factors_(0);       // weight factor on x
			acadoVariables.od[(ACADO_NOD * N_iter) + 9 ] = cost_state_weight_factors_(1);       // weight factor on y
			acadoVariables.od[(ACADO_NOD * N_iter) + 10 ] = cost_control_weight_factors_(0);       // weight factor on theta
			acadoVariables.od[(ACADO_NOD * N_iter) + 11 ] = cost_control_weight_factors_(1);     // weight factor on v
			acadoVariables.od[(ACADO_NOD * N_iter) + 12 ] = cost_state_weight_factors_(2);     // weight factor on w
        }

        acadoVariables.x0[ 0 ] = current_state_(0);
        acadoVariables.x0[ 1 ] = current_state_(1);
        acadoVariables.x0[ 2 ] = current_state_(2);
        acadoVariables.x0[ 3 ] = acadoVariables.x[ACADO_NX+3];

        acado_preparationStep();

        acado_feedbackStep();

        controlled_velocity_.linear.x = acadoVariables.u[0];
        controlled_velocity_.angular.z = acadoVariables.u[1];
		printf("\tReal-Time Iteration:  Path distance = %.3e\n\n", acadoVariables.x[ACADO_NX+3]);
        printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());

		int j=1;
        while (acado_getKKT()> 1e-2 && j<n_iterations_){

			acado_preparationStep();

            acado_feedbackStep();

            controlled_velocity_.linear.x = acadoVariables.u[0];
            controlled_velocity_.angular.z = acadoVariables.u[1];

            printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());
			j++;    //        acado_printDifferentialVariables();
        }
        publishPredictedTrajectory();
		publishPredictedCollisionSpace();
		publishPredictedOutput();
		publishAnaliticSplineTrajectory();
		cost_.data = acado_getObjective();
		publishCost();
		real_t te = acado_toc(&t);
		ROS_INFO_STREAM("Solve time " << te * 1e6 << " us");

    // publish zero controlled velocity
        if (!tracking_)
        {
            actionSuccess();
        }

        if(!enable_output_)
			publishZeroJointVelocity();
		else
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

        int N_iter;
        for (N_iter = 0; N_iter < ACADO_N; N_iter++) {

            // Initialize Online Data variables
            acadoVariables.od[(ACADO_NOD * N_iter) + 8 ] = cost_state_weight_factors_(0);       // weight factor on x
            acadoVariables.od[(ACADO_NOD * N_iter) + 9 ] = cost_state_weight_factors_(1);       // weight factor on y
            acadoVariables.od[(ACADO_NOD * N_iter) + 10 ] = cost_state_weight_factors_(2);       // weight factor on theta
            acadoVariables.od[(ACADO_NOD * N_iter) + 11 ] = cost_control_weight_factors_(0);     // weight factor on v
            acadoVariables.od[(ACADO_NOD * N_iter) + 12 ] = cost_control_weight_factors_(1);     // weight factor on w

        }
/*
		X[0] = traj.multi_dof_joint_trajectory.points[0].transforms[0].translation.x;
		X[1] = traj.multi_dof_joint_trajectory.points[1].transforms[0].translation.x;
		X[2] = traj.multi_dof_joint_trajectory.points[2].transforms[0].translation.x;
		X[3] = traj.multi_dof_joint_trajectory.points[3].transforms[0].translation.x;
		Y[0] = traj.multi_dof_joint_trajectory.points[0].transforms[0].translation.y;
		Y[1] = traj.multi_dof_joint_trajectory.points[1].transforms[0].translation.y;
		Y[2] = traj.multi_dof_joint_trajectory.points[2].transforms[0].translation.y;
		Y[3] = traj.multi_dof_joint_trajectory.points[3].transforms[0].translation.y;
*/

		X[0] = 0;
		X[1] = 10;
		X[2] = 10;

		Y[0] = 0;
		Y[1] = 0;
		Y[2] = 10;

		S[0] = 0;
		S[1] = 10;
		S[2] = 20;

		ref_path_x.set_points(S, X);
		ref_path_y.set_points(S, Y);

		publishSplineTrajectory();
    }
}


void MPCC::reconfigureCallback(predictive_control::PredictiveControllerConfig& config, uint32_t level){

    ROS_INFO("reconfigure callback!");
    cost_state_weight_factors_(0) = config.Kx;
    cost_state_weight_factors_(1) = config.Ky;
    cost_state_weight_factors_(2) = config.Ktheta;
    cost_control_weight_factors_(0) = config.Kv;
    cost_control_weight_factors_(1) = config.Kw;

    cost_state_terminal_weight_factors_(0) = config.Px;
    cost_state_terminal_weight_factors_(1) = config.Py;
    cost_state_terminal_weight_factors_(2) = config.Ptheta;
    slack_weight_= config.Ws;
    repulsive_weight_ = config.WR;

	enable_output_ = config.enable_output;
	n_iterations_ = config.n_iterations;
	simulation_mode_ = config.simulation_mode;
}

void MPCC::executeTrajectory(const moveit_msgs::RobotTrajectory & traj){

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

void MPCC::ObstacleCallBack(const obstacle_feed::Obstacles& obstacles)
{
//    ROS_INFO("OBSTACLECB");

//    obstacle_feed::Obstacles total_obstacles;
//    total_obstacles.Obstacles.resize(controller_config_->n_obstacles_);
//    total_obstacles = obstacles;
//
//    if (obstacles.Obstacles.size() < controller_config_->n_obstacles_)
//    {
//        for (int obst_it = obstacles.Obstacles.size(); obst_it < controller_config_->n_obstacles_; obst_it++)
//        {
//            total_obstacles.Obstacles[obst_it].pose.position.x = 1000;
//            total_obstacles.Obstacles[obst_it].pose.position.y = 1000;
//            total_obstacles.Obstacles[obst_it].pose.orientation.z = 0;
//            total_obstacles.Obstacles[obst_it].major_semiaxis = 0.001;
//            total_obstacles.Obstacles[obst_it].minor_semiaxis = 0.001;
//        }
//    }
//
//    obstacles_ = total_obstacles;
    obstacles_ = obstacles;
}

void MPCC::publishZeroJointVelocity()
{
    if (activate_debug_output_)
    {
//        ROS_INFO("Publishing ZERO joint velocity!!");
    }
    geometry_msgs::Twist pub_msg;
	if(!simulation_mode_)
		broadcastTF();
    controlled_velocity_ = pub_msg;

    controlled_velocity_pub_.publish(controlled_velocity_);
}

void MPCC::publishSplineTrajectory(void)
{
	spline_traj_.header.stamp = ros::Time::now();
	spline_traj_.header.frame_id = controller_config_->tracking_frame_;
	for (int i = 0; i < ACADO_N; i++)
	{
		spline_traj_.poses[i].pose.position.x = ref_path_x(1.0*i*S[1]/ACADO_N); //x
		spline_traj_.poses[i].pose.position.y = ref_path_y(1.0*i*S[1]/ACADO_N); //y

	}

	for (int i = ACADO_N; i < 2*ACADO_N; i++)
	{
		spline_traj_.poses[i].pose.position.x = ref_path_x(1.0*(i-ACADO_N)*S[1]/ACADO_N+10); //x
		spline_traj_.poses[i].pose.position.y = ref_path_y(1.0*(i-ACADO_N)*S[1]/ACADO_N+10); //y

	}
	ROS_INFO_STREAM("REF_PATH_X size:  " << ref_path_x.m_a.size());
	for(int i =0; i< ref_path_x.m_a.size();i++){
		ROS_INFO_STREAM("REF_PATH_Xa:  " << i << "  " << ref_path_x.m_a[i]);
		ROS_INFO_STREAM("REF_PATH_Xb:  " << i << "  " << ref_path_x.m_b[i]);
		ROS_INFO_STREAM("REF_PATH_Xc:  " << i << "  " << ref_path_x.m_c[i]);
		ROS_INFO_STREAM("REF_PATH_Xd:  " << i << "  " << ref_path_x.m_d[i]);
		ROS_INFO_STREAM("REF_PATH_Ya:  " << i << "  " << ref_path_y.m_a[i]);
		ROS_INFO_STREAM("REF_PATH_Yb:  " << i << "  " << ref_path_y.m_b[i]);
		ROS_INFO_STREAM("REF_PATH_Yc:  " << i << "  " << ref_path_y.m_c[i]);
		ROS_INFO_STREAM("REF_PATH_Yd:  " << i << "  " << ref_path_y.m_d[i]);
	}

	spline_traj_pub_.publish(spline_traj_);
}

void MPCC::publishAnaliticSplineTrajectory(void)
{
	spline_traj2_.header.stamp = ros::Time::now();
	spline_traj2_.header.frame_id = controller_config_->tracking_frame_;
	double s;
	for (int i = 0; i < ACADO_N; i++)
	{
		s= 1.0*i*S[1]/ACADO_N;
		spline_traj2_.poses[i].pose.position.x = ref_path_x.m_a[0]*s*s*s+ref_path_x.m_b[0]*s*s+ref_path_x.m_c[0]*s+ref_path_x.m_d[0]; //x
		spline_traj2_.poses[i].pose.position.y = ref_path_y.m_a[0]*s*s*s+ref_path_y.m_b[0]*s*s+ref_path_y.m_c[0]*s+ref_path_y.m_d[0]; //y

	}

	spline_traj_pub2_.publish(spline_traj2_);
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

void MPCC::publishPredictedCollisionSpace(void)
{
	visualization_msgs::MarkerArray collision_space;


	for (int i = 0; i < ACADO_N; i++)
	{
		ellips1.id = 60+i;
		ellips1.pose.position.x = acadoVariables.x[i * ACADO_NX + 0];
		ellips1.pose.position.y = acadoVariables.x[i * ACADO_NX + 1];
		ellips1.pose.orientation.x = 0;
		ellips1.pose.orientation.y = 0;
		ellips1.pose.orientation.z = 0;
		ellips1.pose.orientation.w = 1;
		collision_space.markers.push_back(ellips1);
	}

	robot_collision_space_pub_.publish(collision_space);
}

void MPCC::publishCost(void){

	cost_pub_.publish(cost_);
}