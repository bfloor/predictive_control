
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

        obstacle_feed_sub_ = nh.subscribe(controller_config_->sub_ellipse_topic_, 1, &MPCC::ObstacleCallBack, this);

        map_service_ = nh.serviceClient<nav_msgs::GetMap>("static_map");

        //Publishers
        traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory",1);
		pred_cmd_pub_ = nh.advertise<nav_msgs::Path>("predicted_cmd",1);
		cost_pub_ = nh.advertise<std_msgs::Float64>("cost",1);
        contour_error_pub_ = nh.advertise<std_msgs::Float64MultiArray>("contour_error",1);
        controlled_velocity_pub_ = nh.advertise<geometry_msgs::Twist>(controller_config_->output_cmd,1);
		joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
		robot_collision_space_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/robot_collision_space", 100);
        pred_traj_pub_ = nh.advertise<nav_msgs::Path>("predicted_trajectory",1);
		global_plan_pub_ = nh.advertise<visualization_msgs::MarkerArray>("global_plan",1);
        local_spline_traj_pub1_ = nh.advertise<nav_msgs::Path>("reference_trajectory_seg1",1);
        local_spline_traj_pub2_ = nh.advertise<nav_msgs::Path>("reference_trajectory_seg2",1);
        local_spline_traj_pub3_ = nh.advertise<nav_msgs::Path>("reference_trajectory_seg3",1);
		feedback_pub_ = nh.advertise<predictive_control::control_feedback>("controller_feedback",1);
        collision_free_pub_ = nh.advertise<visualization_msgs::MarkerArray>("collision_free_circles",1);

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
        slack_weight_ = controller_config_->slack_weight_;
        repulsive_weight_ = controller_config_->repulsive_weight_;
        reference_velocity_ = controller_config_->reference_velocity_;

        ros::NodeHandle nh_predictive("predictive_controller");

        /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
        reconfigure_server_.reset(new dynamic_reconfigure::Server<predictive_control::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
        reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));

	    // Initialize obstacles
        obstacle_feed::Obstacles obstacles;
		obstacles.Obstacles.resize(controller_config_->n_obstacles_);
        obstacles_.Obstacles.resize(controller_config_->n_obstacles_);
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
		enable_output_ = false;
		n_iterations_ = 100;
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

        ellips2.type = visualization_msgs::Marker::CYLINDER;
        ellips2.id = 400;
        ellips2.color.r = 0.5;
        ellips2.color.g = 0.5;
        ellips2.color.b = 0.0;
        ellips2.color.a = 0.1;
        ellips2.header.frame_id = controller_config_->tracking_frame_;
        ellips2.ns = "trajectory";
        ellips2.action = visualization_msgs::Marker::ADD;
        ellips2.lifetime = ros::Duration(0.1);
        ellips2.scale.x = 1.0*2.0;
        ellips2.scale.y = 1.0*2.0;
        ellips2.scale.z = 0.05;

        global_plan.type = visualization_msgs::Marker::CYLINDER;
        global_plan.id = 800;
        global_plan.color.r = 0.8;
        global_plan.color.g = 0.0;
        global_plan.color.b = 0.0;
        global_plan.color.a = 0.8;
        global_plan.header.frame_id = controller_config_->tracking_frame_;
        global_plan.ns = "trajectory";
        global_plan.action = visualization_msgs::Marker::ADD;
        global_plan.lifetime = ros::Duration(1000);
        global_plan.scale.x = 0.1;
        global_plan.scale.y = 0.1;
        global_plan.scale.z = 0.05;

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

		// MPCC reference path variables
		X_global.resize(controller_config_->ref_x_.size());
		Y_global.resize(controller_config_->ref_y_.size());
		Theta_global.resize(controller_config_->ref_theta_.size());

        // Resize vector of collision free radii along prediction horizon
        collision_free_R_.resize(ACADO_N);
        collision_free_X_.resize(ACADO_N);
        collision_free_Y_.resize(ACADO_N);

        collision_free_r_max_ = 5;
        occupied_ = 50;

		// Check if all reference vectors are of the same length
		if (!( (controller_config_->ref_x_.size() == controller_config_->ref_y_.size()) && ( controller_config_->ref_x_.size() == controller_config_->ref_theta_.size() ) && (controller_config_->ref_y_.size() == controller_config_->ref_theta_.size()) ))
        {
            ROS_ERROR("Reference path inputs should be of equal length");
        }

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

void MPCC::reconfigureCallback(predictive_control::PredictiveControllerConfig& config, uint32_t level){

    ROS_INFO("reconfigure callback!");
    cost_contour_weight_factors_(0) = config.Wcontour;
    cost_contour_weight_factors_(1) = config.Wlag;
    cost_control_weight_factors_(0) = config.Kv;
    cost_control_weight_factors_(1) = config.Kw;

    slack_weight_= config.Ws;
    repulsive_weight_ = config.WR;

    reference_velocity_ = config.vRef;
    collision_free_r_max_ = config.rMax;
    occupied_ = config.occThres;

    enable_output_ = config.enable_output;
    loop_mode_ = config.loop_mode;
    n_iterations_ = config.n_iterations;
    simulation_mode_ = config.simulation_mode;

    //Search window parameters
    window_size_ = config.window_size;
    n_search_points_ = config.n_search_points;
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

void MPCC::broadcastPathPose(){

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = controller_config_->tracking_frame_;
	transformStamped.child_frame_id = "path";

	transformStamped.transform.translation.x = ref_path_x(acadoVariables.x[3]);
	transformStamped.transform.translation.y = ref_path_y(acadoVariables.x[3]);
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
        acadoVariables.x[4] = 0.0000001;          //dummy state

        acadoVariables.u[0] = controlled_velocity_.linear.x;
        acadoVariables.u[1] = controlled_velocity_.angular.z;
        acadoVariables.u[2] = 0.0000001;           //slack variable

		if(acadoVariables.x[3] > ss[2]) {

		    if (traj_i + N_local > controller_config_->ref_x_.size()*n_traj_per_cloth){
		        goal_reached_ = true;
                ROS_ERROR_STREAM("GOAL REACHED");
                if (loop_mode_)
                {
                    traj_i = 0;
                    goal_reached_ = false;
                    last_poly_ = false;
                    acadoVariables.x[3] = s0_;
                    ROS_ERROR_STREAM("LOOP STARTED");
                }
            } else{
			    traj_i++;
                UpdateLocalRefPath();
//                publishSplineTrajectory();
                acadoVariables.x[3] = s0_;
                ROS_ERROR_STREAM("SWITCH SPLINE, traj_i =  " << traj_i);
		    }
        }

//        ROS_INFO_STREAM("traj_i: " << traj_i);
//        ROS_INFO_STREAM("ss_length: " << ss.size());

        if(idx == 1) {
            double smin;
            smin = spline_closest_point(ss[1], 100, acadoVariables.x[3], window_size_, n_search_points_);
            acadoVariables.x[3] = smin;
//            ROS_ERROR_STREAM("smin: " << smin);
        }
        else
            acadoVariables.x[3] = acadoVariables.x[3];


        for (N_iter = 0; N_iter < ACADO_N; N_iter++) {

            // Initialize Online Data variables
            acadoVariables.od[(ACADO_NOD * N_iter) + 0 ] = ref_path_x.m_a[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 1 ] = ref_path_x.m_b[1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 2 ] = ref_path_x.m_c[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 3 ] = ref_path_x.m_d[1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 4 ] = ref_path_y.m_a[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 5 ] = ref_path_y.m_b[1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 6 ] = ref_path_y.m_c[1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 7 ] = ref_path_y.m_d[1];

            acadoVariables.od[(ACADO_NOD * N_iter) + 8 ] = ref_path_x.m_a[2];         // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 9 ] = ref_path_x.m_b[2];
            acadoVariables.od[(ACADO_NOD * N_iter) + 10] = ref_path_x.m_c[2];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 11] = ref_path_x.m_d[2];
            acadoVariables.od[(ACADO_NOD * N_iter) + 12] = ref_path_y.m_a[2];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 13] = ref_path_y.m_b[2];
            acadoVariables.od[(ACADO_NOD * N_iter) + 14] = ref_path_y.m_c[2];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 15] = ref_path_y.m_d[2];

            acadoVariables.od[(ACADO_NOD * N_iter) + 16] = ref_path_x.m_a[3];         // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 17] = ref_path_x.m_b[3];
            acadoVariables.od[(ACADO_NOD * N_iter) + 18] = ref_path_x.m_c[3];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 19] = ref_path_x.m_d[3];
            acadoVariables.od[(ACADO_NOD * N_iter) + 20] = ref_path_y.m_a[3];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 21] = ref_path_y.m_b[3];
            acadoVariables.od[(ACADO_NOD * N_iter) + 22] = ref_path_y.m_c[3];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 23] = ref_path_y.m_d[3];

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

            acadoVariables.od[(ACADO_NOD * N_iter) + 36] = slack_weight_;        // weight on the slack variable
            acadoVariables.od[(ACADO_NOD * N_iter) + 37] = repulsive_weight_;    // weight on the repulsive cost

            acadoVariables.od[(ACADO_NOD * N_iter) + 40] = obstacles_.Obstacles[0].pose.position.x;      // x position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 41] = obstacles_.Obstacles[0].pose.position.y;      // y position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 42] = obstacles_.Obstacles[0].pose.orientation.z;   // heading of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 43] = obstacles_.Obstacles[0].major_semiaxis;       // major semiaxis of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 44] = obstacles_.Obstacles[0].minor_semiaxis;       // minor semiaxis of obstacle 1

            acadoVariables.od[(ACADO_NOD * N_iter) + 45] = obstacles_.Obstacles[1].pose.position.x;      // x position of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 46] = obstacles_.Obstacles[1].pose.position.y;      // y position of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 47] = obstacles_.Obstacles[1].pose.orientation.z;   // heading of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 48] = obstacles_.Obstacles[1].major_semiaxis;       // major semiaxis of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 49] = obstacles_.Obstacles[1].minor_semiaxis;       // minor semiaxis of obstacle 2
    }

        acadoVariables.x0[ 0 ] = current_state_(0);
        acadoVariables.x0[ 1 ] = current_state_(1);
        acadoVariables.x0[ 2 ] = current_state_(2);
		acadoVariables.x0[ 3 ] = acadoVariables.x[3];
        acadoVariables.x0[ 4 ] = 0.0000001;             //dummy state

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
		publishPredictedCollisionSpace();
		publishPredictedOutput();
		publishLocalSplineTrajectory();
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

double MPCC::spline_closest_point(double s_min, double s_max, double s_guess, double window, int n_tries){

	double lower = std::max(s_min, s_guess-window);
	double upper = std::min(s_max, s_guess + window);
	double s_i=lower,spline_pos_x_i,spline_pos_y_i;
	double dist_i,min_dist,smin;

	spline_pos_x_i = ref_path_x(s_i);
	spline_pos_y_i = ref_path_y(s_i);

	min_dist = std::sqrt((spline_pos_x_i-current_state_(0))*(spline_pos_x_i-current_state_(0))+(spline_pos_y_i-current_state_(1))*(spline_pos_y_i-current_state_(1)));

	for(int i=0;i<n_tries;i++){
		s_i = lower+(upper-lower)/n_tries*i;
		spline_pos_x_i = ref_path_x(s_i);
		spline_pos_y_i = ref_path_y(s_i);
		dist_i = std::sqrt((spline_pos_x_i-current_state_(0))*(spline_pos_x_i-current_state_(0))+(spline_pos_y_i-current_state_(1))*(spline_pos_y_i-current_state_(1)));

		if(dist_i<min_dist){
			min_dist = dist_i;
			smin = s_i;
		}

        ros::Duration(1E-20).sleep();
	}
    if(smin < lower){
        smin=lower;
    }

	return smin;

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

void MPCC::InitLocalRefPath() {

    // Number of segments in the local reference path
    N_local = 3;

    // Number of segments in each clothoid fitted through the global points
    n_traj_per_cloth = controller_config_->n_poly_per_clothoid_;

    // Compute how many clothoid segments and points should be build for the local reference path
    n_cloth_segments = ceil((double) N_local/n_traj_per_cloth);
    n_clothoid = n_cloth_segments + 1;

    // Number of points defining the local spline +1 for continuous transition in beginning
    n_pts = N_local + 2;
    // Number of spline points in the local clothoid path
    n_pts_all = n_cloth_segments*n_traj_per_cloth + 1;

    // Initialize reference velocity setpoint
    vv.resize(N_local,1);

    // Read global path
    for (int ref_point_it = 0; ref_point_it < controller_config_->ref_x_.size(); ref_point_it++) {
        X_global[ref_point_it] = controller_config_->ref_x_.at(ref_point_it);
        Y_global[ref_point_it] = controller_config_->ref_y_.at(ref_point_it);
        Theta_global[ref_point_it] = controller_config_->ref_theta_.at(ref_point_it);
    }

    // Initialize variables to generate clothoid local reference path
    double k, dk, L;

    std::vector<double> X, Y;
    std::vector<double> X_all(n_pts_all), Y_all(n_pts_all), S_all(n_pts_all);

    total_length_ = 0;

    ROS_INFO_STREAM("X_global size: " << X_global.size());

    for (int clothoid_it = 0; clothoid_it < n_cloth_segments; clothoid_it++) {
        Clothoid::buildClothoid(X_global[clothoid_it], Y_global[clothoid_it], Theta_global[clothoid_it],
                                X_global[clothoid_it + 1], Y_global[clothoid_it + 1], Theta_global[clothoid_it + 1], k, dk,
                                L);

        Clothoid::pointsOnClothoid(X_global[clothoid_it], Y_global[clothoid_it], Theta_global[clothoid_it], k, dk, L,
                                   n_traj_per_cloth + 1, X, Y);

        ROS_INFO_STREAM("clothoid_it: " << clothoid_it);

        for (int clothoid_points_it = 0; clothoid_points_it < n_traj_per_cloth; clothoid_points_it++) {
            ROS_INFO_STREAM("X: " << X[clothoid_points_it]);
            ROS_INFO_STREAM("Y: " << Y[clothoid_points_it]);

            X_all[clothoid_it * n_traj_per_cloth + clothoid_points_it] = X[clothoid_points_it];
            Y_all[clothoid_it * n_traj_per_cloth + clothoid_points_it] = Y[clothoid_points_it];
            S_all[clothoid_it * n_traj_per_cloth + clothoid_points_it] =
                    total_length_ + (clothoid_points_it) * L / n_traj_per_cloth;
        }

        total_length_ += L;
    }

    X_all[n_pts_all - 1] = X_global[X_global.size() - 1];
    Y_all[n_pts_all - 1] = Y_global[X_global.size() - 1];
    S_all[n_pts_all - 1] = total_length_;

    ss.resize(n_pts);
    xx.resize(n_pts);
    yy.resize(n_pts);

    s0_ = 0.5;

    ss[0] = 0;
    xx[0] = -s0_;
    yy[0] = 0;

    for (int local_points_it = 1; local_points_it < n_pts; local_points_it++ ) {
        xx[local_points_it] = X_all[local_points_it - 1];
        yy[local_points_it] = Y_all[local_points_it - 1];
        ss[local_points_it] = S_all[local_points_it - 1] + s0_;
    }

    total_length_ = ss[n_pts - 1];

    // Set local spline points
    ref_path_x.set_points(ss,xx);
    ref_path_y.set_points(ss,yy);
}

void MPCC::UpdateLocalRefPath() {

    double ss_start = ss[1];

    // Shift local reference path segments one place
    for (int local_path_it = 0; local_path_it < n_pts - 1; local_path_it++) {
        ss[local_path_it] = ss[local_path_it + 1] - ss_start;
        xx[local_path_it] = xx[local_path_it + 1];
        yy[local_path_it] = yy[local_path_it + 1];
    }

    for (int v_it = 0; v_it < N_local - 1; v_it++){
        vv[v_it] = vv[v_it + 1];
    }

    // Initialize variables to generate clothoid local reference path
    double k, dk, L;

    std::vector<double> X, Y;
    std::vector<double> X_all(3), Y_all(3), S_all(3);

    total_length_ = 0;

    double xroad1, xroad2, yroad1, yroad2, thetaroad1, thetaroad2;

    int cloth_end_i = floor( ((double) (traj_i + N_local - 1))/((double) n_traj_per_cloth) );
    ROS_INFO_STREAM("cloth_end_i = " << cloth_end_i);

    // Deal with last sections of the reference path
    if (cloth_end_i + 2 > X_global.size() ) {
        xroad1 = X_global[X_global.size() - 1];
        xroad2 = xroad1 + 0.1;
        yroad1 = Y_global[X_global.size() - 1];
        yroad2 = yroad1;
        thetaroad1 = Theta_global[X_global.size() - 1];
        thetaroad2 = thetaroad1;
        vv[N_local - 1] = 0;
    }
    else {
        xroad1 = X_global[cloth_end_i];
        xroad2 = X_global[cloth_end_i + 1];
        yroad1 = Y_global[cloth_end_i];
        yroad2 = Y_global[cloth_end_i + 1];
        thetaroad1 = Theta_global[cloth_end_i];
        thetaroad2 = Theta_global[cloth_end_i + 1];
        vv[N_local - 1] = 1;
    }

    // Build clothoid for extending the local reference path
    Clothoid::buildClothoid(xroad1, yroad1, thetaroad1, xroad2, yroad2, thetaroad2, k, dk,L);
    Clothoid::pointsOnClothoid(xroad1, yroad1, thetaroad1, k, dk, L, n_traj_per_cloth + 1, X, Y);

    X_all[0] = X[0];
    Y_all[0] = Y[0];
    S_all[0] = 0;

    X_all[1] = X[1];
    Y_all[1] = Y[1];
    S_all[1] = L/2;

    X_all[2] = xroad2;
    Y_all[2] = yroad2;
    S_all[2] = L;

    ss[n_pts - 1] = ss[n_pts - 2] + S_all[traj_i%n_traj_per_cloth + 1] - S_all[traj_i%n_traj_per_cloth];
    xx[n_pts - 1] = X_all[traj_i%n_traj_per_cloth + 1];
    yy[n_pts - 1] = Y_all[traj_i%n_traj_per_cloth + 1];

    // Set local spline points
    ref_path_x.set_points(ss,xx);
    ref_path_y.set_points(ss,yy);

    total_length_ = ss[n_pts - 1];
    s0_ = ss[1];
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

            // Initialize Constant Online Data variables
            acadoVariables.od[(ACADO_NOD * N_iter) + 38] = r_discs_;                                // radius of car discs
            acadoVariables.od[(ACADO_NOD * N_iter) + 39] = 0; //x_discs_[1];                        // position of the car discs
        }

        traj_i = 0;

		goal_reached_ = false;
		last_poly_ = false;

        InitLocalRefPath();
		publishLocalSplineTrajectory();
        publishGlobalPlan();
    }
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

    obstacle_feed::Obstacles total_obstacles;
    total_obstacles.Obstacles.resize(controller_config_->n_obstacles_);

    total_obstacles.Obstacles = obstacles.Obstacles;

//    ROS_INFO_STREAM("-- Received # obstacles: " << obstacles.Obstacles.size());
//    ROS_INFO_STREAM("-- Expected # obstacles: " << controller_config_->n_obstacles_);

    if (obstacles.Obstacles.size() < controller_config_->n_obstacles_)
    {
        for (int obst_it = obstacles.Obstacles.size(); obst_it < controller_config_->n_obstacles_; obst_it++)
        {
            total_obstacles.Obstacles[obst_it].pose.position.x = 1000;
            total_obstacles.Obstacles[obst_it].pose.position.y = 1000;
            total_obstacles.Obstacles[obst_it].pose.orientation.z = 0;
            total_obstacles.Obstacles[obst_it].major_semiaxis = 0.001;
            total_obstacles.Obstacles[obst_it].minor_semiaxis = 0.001;
        }
    }

    obstacles_.Obstacles.resize(controller_config_->n_obstacles_);

    for (int total_obst_it = 0; total_obst_it < controller_config_->n_obstacles_; total_obst_it++)
    {
        obstacles_.Obstacles[total_obst_it] = total_obstacles.Obstacles[total_obst_it];
    }

//    ROS_INFO_STREAM("-- total_Obst1: [" << total_obstacles.Obstacles[0].pose.position.x << ",  " << total_obstacles.Obstacles[0].pose.position.y << "], Obst2 [" << total_obstacles.Obstacles[1].pose.position.x << ",  " << total_obstacles.Obstacles[1].pose.position.y << "]");
//    ROS_INFO_STREAM("-- Obst1_: [" << obstacles_.Obstacles[0].pose.position.x << ",  " << obstacles_.Obstacles[0].pose.position.y << "], Obst2 [" << obstacles_.Obstacles[1].pose.position.x << ",  " << obstacles_.Obstacles[1].pose.position.y << "]");
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

void MPCC::publishLocalSplineTrajectory(void)
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

        local_spline_traj1_.poses[i].pose.position.x = ref_path_x.m_a[1]*s1*s1*s1+ref_path_x.m_b[1]*s1*s1+ref_path_x.m_c[1]*s1+ref_path_x.m_d[1]; //x
        local_spline_traj1_.poses[i].pose.position.y = ref_path_y.m_a[1]*s1*s1*s1+ref_path_y.m_b[1]*s1*s1+ref_path_y.m_c[1]*s1+ref_path_y.m_d[1]; //y
        local_spline_traj1_.poses[i].header.stamp = ros::Time::now();
        local_spline_traj1_.poses[i].header.frame_id = controller_config_->tracking_frame_;

        local_spline_traj2_.poses[i].pose.position.x = ref_path_x.m_a[2]*s2*s2*s2+ref_path_x.m_b[2]*s2*s2+ref_path_x.m_c[2]*s2+ref_path_x.m_d[2]; //x
        local_spline_traj2_.poses[i].pose.position.y = ref_path_y.m_a[2]*s2*s2*s2+ref_path_y.m_b[2]*s2*s2+ref_path_y.m_c[2]*s2+ref_path_y.m_d[2]; //y
        local_spline_traj2_.poses[i].header.stamp = ros::Time::now();
        local_spline_traj2_.poses[i].header.frame_id = controller_config_->tracking_frame_;

        local_spline_traj3_.poses[i].pose.position.x = ref_path_x.m_a[3]*s3*s3*s3+ref_path_x.m_b[3]*s3*s3+ref_path_x.m_c[3]*s3+ref_path_x.m_d[3]; //x
        local_spline_traj3_.poses[i].pose.position.y = ref_path_y.m_a[3]*s3*s3*s3+ref_path_y.m_b[3]*s3*s3+ref_path_y.m_c[3]*s3+ref_path_y.m_d[3]; //y
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

void MPCC::publishContourError(void){

    // Compute contour and lag error to publish
    double x_path, y_path, dx_path, dy_path, abs_grad, dx_path_norm, dy_path_norm;

    x_path = (ref_path_x.m_a[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_x.m_b[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_x.m_c[traj_i]*(acadoVariables.x[3]-ss[traj_i]) + ref_path_x.m_d[traj_i]);
    y_path = (ref_path_y.m_a[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_y.m_b[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_y.m_c[traj_i]*(acadoVariables.x[3]-ss[traj_i]) + ref_path_y.m_d[traj_i]);
    dx_path = (3*ref_path_x.m_a[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + 2*ref_path_x.m_b[traj_i]*(acadoVariables.x[3]-ss[traj_i]) + ref_path_x.m_c[traj_i]);
    dy_path = (3*ref_path_y.m_a[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + 2*ref_path_y.m_b[traj_i]*(acadoVariables.x[3]-ss[traj_i]) + ref_path_y.m_c[traj_i]);

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
    feedback_msg.freespace_time = te_collison_free_;
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

    feedback_msg.obstacle_distance1 = sqrt(pow(pred_traj_.poses[0].pose.position.x - obstacles_.Obstacles[0].pose.position.x ,2) + pow(pred_traj_.poses[0].pose.position.y - obstacles_.Obstacles[0].pose.position.y,2));
    feedback_msg.obstacle_distance2 = sqrt(pow(pred_traj_.poses[0].pose.position.x - obstacles_.Obstacles[1].pose.position.x ,2) + pow(pred_traj_.poses[0].pose.position.y - obstacles_.Obstacles[1].pose.position.y,2));

    feedback_msg.obstx_0 = obstacles_.Obstacles[0].pose.position.x;
    feedback_msg.obsty_0 = obstacles_.Obstacles[0].pose.position.y;
    feedback_msg.obsth_0 = obstacles_.Obstacles[0].pose.orientation.z;
    feedback_msg.obsta_0 = obstacles_.Obstacles[0].major_semiaxis;
    feedback_msg.obstb_0 = obstacles_.Obstacles[0].minor_semiaxis;

    feedback_msg.obstx_1 = obstacles_.Obstacles[1].pose.position.x;
    feedback_msg.obsty_1 = obstacles_.Obstacles[1].pose.position.y;
    feedback_msg.obsth_1 = obstacles_.Obstacles[1].pose.orientation.z;
    feedback_msg.obsta_1 = obstacles_.Obstacles[1].major_semiaxis;
    feedback_msg.obstb_1 = obstacles_.Obstacles[1].minor_semiaxis;

    //Search window parameters
    feedback_msg.window = window_size_;
    feedback_msg.search_points = n_search_points_;

    feedback_pub_.publish(feedback_msg);
}