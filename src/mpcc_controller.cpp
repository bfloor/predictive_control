
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
		spline_traj_pub_ = nh.advertise<nav_msgs::Path>("spline_traj",1);
		spline_traj_pub2_ = nh.advertise<nav_msgs::Path>("reference_trajectory",1);
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
		spline_traj_.poses.resize(100);
		spline_traj2_.poses.resize(100);
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

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

		// MPCC reference path variables
		X_road.resize(controller_config_->ref_x_.size());
		Y_road.resize(controller_config_->ref_y_.size());
		Theta_road.resize(controller_config_->ref_theta_.size());

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
		acado_initializeSolver( );
        acadoVariables.x[0] = current_state_(0);
        acadoVariables.x[1] = current_state_(1);
        acadoVariables.x[2] = current_state_(2);
        acadoVariables.x[4] = 0.0000001;          //dummy state

        acadoVariables.u[0] = controlled_velocity_.linear.x;
        acadoVariables.u[1] = controlled_velocity_.angular.z;
        acadoVariables.u[2] = 0.0000001;           //slack variable

		if(acadoVariables.x[3] > ss[traj_i+1]) {

		    if (traj_i + 3 == ss.size()){
                last_poly_ = true;
                traj_i++;
                ROS_ERROR_STREAM("LAST POLYNOMIAL");
                ROS_ERROR_STREAM("SWITCH SPLINE " << acadoVariables.x[3]);
		    }
		    else if (traj_i + 3 > ss.size()){
		        goal_reached_ = true;
                ROS_ERROR_STREAM("GOAL REACHED");
            } else{
			    traj_i++;
                //acadoVariables.x[3]-=ss[traj_i];
                ROS_ERROR_STREAM("SWITCH SPLINE " << acadoVariables.x[3]);
//                ComputeCollisionFreeArea();
		    }
        }

        ComputeCollisionFreeArea();

//        ROS_INFO_STREAM("traj_i: " << traj_i);
//        ROS_INFO_STREAM("ss_length: " << ss.size());

        if(idx ==1) {
            double smin;
            smin = spline_closest_point(ss[traj_i], 100, acadoVariables.x[ACADO_NX+3], window_size_, n_search_points_);
            acadoVariables.x[3] = smin;
//            ROS_ERROR_STREAM("smin: " << smin);
        }
        else
            acadoVariables.x[3] = acadoVariables.x[3];


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

            acadoVariables.od[(ACADO_NOD * N_iter) + 8] = ref_path_x.m_a[traj_i + 1];         // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 9] = ref_path_x.m_b[traj_i + 1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 10] = ref_path_x.m_c[traj_i + 1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 11] = ref_path_x.m_d[traj_i + 1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 12] = ref_path_y.m_a[traj_i + 1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 13] = ref_path_y.m_b[traj_i + 1];
            acadoVariables.od[(ACADO_NOD * N_iter) + 14] = ref_path_y.m_c[traj_i + 1];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 15] = ref_path_y.m_d[traj_i + 1];

			acadoVariables.od[(ACADO_NOD * N_iter) + 16] = cost_contour_weight_factors_(0);       // weight factor on contour error
			acadoVariables.od[(ACADO_NOD * N_iter) + 17] = cost_contour_weight_factors_(1);       // weight factor on lag error
			acadoVariables.od[(ACADO_NOD * N_iter) + 18 ] = cost_control_weight_factors_(0);      // weight factor on theta
			acadoVariables.od[(ACADO_NOD * N_iter) + 19 ] = cost_control_weight_factors_(1);      // weight factor on v

            acadoVariables.od[(ACADO_NOD * N_iter) + 20 ] = ss[traj_i];
            acadoVariables.od[(ACADO_NOD * N_iter) + 21 ] = ss[traj_i + 1];

            if (goal_reached_){
                acadoVariables.od[(ACADO_NOD * N_iter) + 22] = 0;
                acadoVariables.od[(ACADO_NOD * N_iter) + 23] = 0;
            }
            else if (last_poly_) {
                acadoVariables.od[(ACADO_NOD * N_iter) + 22] = reference_velocity_;
                acadoVariables.od[(ACADO_NOD * N_iter) + 23] = 0;
            }
            else {
                acadoVariables.od[(ACADO_NOD * N_iter) + 22] = reference_velocity_;
                acadoVariables.od[(ACADO_NOD * N_iter) + 23] = reference_velocity_;
            }

			acadoVariables.od[(ACADO_NOD * N_iter) + 24] = ss[traj_i + 1] + 0.02;

            acadoVariables.od[(ACADO_NOD * N_iter) + 25] = slack_weight_;        // weight on the slack variable
            acadoVariables.od[(ACADO_NOD * N_iter) + 26] = repulsive_weight_;    // weight on the repulsive cost

            acadoVariables.od[(ACADO_NOD * N_iter) + 29] = obstacles_.Obstacles[0].pose.position.x;      // x position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 30] = obstacles_.Obstacles[0].pose.position.y;      // y position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 31] = obstacles_.Obstacles[0].pose.orientation.z;   // heading of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 32] = obstacles_.Obstacles[0].major_semiaxis;       // major semiaxis of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 33] = obstacles_.Obstacles[0].minor_semiaxis;       // minor semiaxis of obstacle 1

            acadoVariables.od[(ACADO_NOD * N_iter) + 34] = obstacles_.Obstacles[1].pose.position.x;      // x position of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 35] = obstacles_.Obstacles[1].pose.position.y;      // y position of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 36] = obstacles_.Obstacles[1].pose.orientation.z;   // heading of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 37] = obstacles_.Obstacles[1].major_semiaxis;       // major semiaxis of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 38] = obstacles_.Obstacles[1].minor_semiaxis;       // minor semiaxis of obstacle 2

            // Set radius for convex collision free circles
            acadoVariables.od[(ACADO_NOD * N_iter) + 39] = collision_free_r1_;
            acadoVariables.od[(ACADO_NOD * N_iter) + 40] = collision_free_r2_;
    }

        acadoVariables.x0[ 0 ] = current_state_(0);
        acadoVariables.x0[ 1 ] = current_state_(1);
        acadoVariables.x0[ 2 ] = current_state_(2);
		acadoVariables.x0[ 3 ] = acadoVariables.x[3];
        acadoVariables.x0[ 4 ] = 0.0000001;             //dummy state

//        ROS_INFO_STREAM("ss[traj_i]: " << ss[traj_i]);
//        ROS_INFO_STREAM("acadoVariables.x[3]: " << acadoVariables.x[3]);

        acado_preparationStep();

        acado_feedbackStep();

        //printf("\tReal-Time Iteration:  Path distance = %.3e\n\n", acadoVariables.x[ACADO_NX+3]);
        //printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());

		int j=1;
        while (acado_getKKT()> 1e-3 && j<n_iterations_){

			acado_preparationStep();

            acado_feedbackStep();

            printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());
			j++;    //        acado_printDifferentialVariables();
        }

        te_ = acado_toc(&t);

		controlled_velocity_.linear.x = acadoVariables.u[0];
		controlled_velocity_.angular.z = acadoVariables.u[1];

        publishPredictedTrajectory();
		publishPredictedCollisionSpace();
        publishPosConstraint();
		publishPredictedOutput();
		publishAnaliticSplineTrajectory();
		broadcastPathPose();
        publishContourError();
		cost_.data = acado_getObjective();
		publishCost();

		if (publish_feedback_){publishFeedback(j,te_);}

		ROS_INFO_STREAM("Solve time " << te_ * 1e6 << " us");

    // publish zero controlled velocity
        if (!tracking_)
        {
            actionSuccess();
        }
	}
    if(!enable_output_) {
		publishZeroJointVelocity();
		idx = 2; // used to keep computation of the mpc and dod not let it move because we set the initial state as the predicted state
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

		//		ROS_INFO_STREAM("s_i: " << s_i << " " << "spline_pos_x_i: " << spline_pos_x_i << " " << "spline_pos_y_i: " << spline_pos_y_i << " " << "dist_i: " << dist_i << " ");

        // QUAINT ERROR IN INDIGO. IF NOT PAUSED, WEIRD SOLUTIONS ARE RETURNED
//	    ros::Duration(0.000000001).sleep();
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

void MPCC::Ref_path(std::vector<double> x,std::vector<double> y, std::vector<double> theta) {

    double k, dk, L;
    std::vector<double> X(10), Y(10);
    std::vector<double> X_all, Y_all, S_all;
    total_length_= 0;
    n_clothoid = controller_config_->n_points_clothoid_;
    n_pts = controller_config_->n_points_spline_;
    S_all.push_back(0);

    for (int i = 0; i < x.size()-1; i++){
        Clothoid::buildClothoid(x[i], y[i], theta[i], x[i+1], y[i+1], theta[i+1], k, dk, L);

        Clothoid::pointsOnClothoid(x[i], y[i], theta[i], k, dk, L, n_clothoid, X, Y);
        if (i==0){
            X_all.insert(X_all.end(), X.begin(), X.end());
            Y_all.insert(Y_all.end(), Y.begin(), Y.end());
        }
        else{
            X.erase(X.begin()+0);
            Y.erase(Y.begin()+0);
            X_all.insert(X_all.end(), X.begin(), X.end());
            Y_all.insert(Y_all.end(), Y.begin(), Y.end());
        }
        total_length_ += L;
        for (int j=1; j < n_clothoid; j++){
            S_all.push_back(S_all[j-1+i*(n_clothoid-1)]+L/(n_clothoid-1));
            ROS_INFO_STREAM("S_all: " << S_all[j]);
        }
        ROS_INFO_STREAM("X_all: " << X_all[i]);
        ROS_INFO_STREAM("Y_all: " << Y_all[i]);
    }

    ref_path_x.set_points(S_all, X_all);
    ref_path_y.set_points(S_all, Y_all);

    ROS_INFO_STREAM("total_length: " << total_length_);
    dist_spline_pts_ = total_length_ / (n_pts - 1);
    ROS_INFO_STREAM("dist_spline_pts_: " << dist_spline_pts_);
    ss.resize(n_pts);
    xx.resize(n_pts);
    yy.resize(n_pts);

    for (int i=0; i<n_pts; i++){
        ss[i] = dist_spline_pts_ * i;
        xx[i] = ref_path_x(ss[i]);
        yy[i] = ref_path_y(ss[i]);
        ROS_INFO_STREAM("ss: " << ss[i]);
        ROS_INFO_STREAM("xx: " << xx[i]);
        ROS_INFO_STREAM("yy: " << yy[i]);
    }

    ref_path_x.set_points(ss,xx);
    ref_path_y.set_points(ss,yy);
}

void MPCC::ConstructRefPath(){

    for (int ref_point_it = 0; ref_point_it < controller_config_->ref_x_.size(); ref_point_it++)
    {
        X_road[ref_point_it] = controller_config_->ref_x_.at(ref_point_it);
        Y_road[ref_point_it] = controller_config_->ref_y_.at(ref_point_it);
        Theta_road[ref_point_it] = controller_config_->ref_theta_.at(ref_point_it);
    }

    Ref_path(X_road, Y_road, Theta_road);
}

void MPCC::ComputeCollisionFreeArea()
{
    acado_timer t;
    acado_tic( &t );

    int x_path_i, y_path_i;
    double x_path, y_path, theta_search, r;

    int search_steps = 10;

    collision_free_r1_ = collision_free_r_max_;
    collision_free_r2_ = collision_free_r_max_;

    ROS_INFO_STREAM("ss[traj_i] = " << ss[traj_i] << " ss[traj_i + 1] = " << ss[traj_i + 1] << " ss[traj_i + 2] = " << ss[traj_i + 2]);

    for (int N_it = 0; N_it < ACADO_N; N_it++)
    {

        x_path = acadoVariables.x[N_it * ACADO_NX + 0];
        y_path = acadoVariables.x[N_it * ACADO_NX + 1];

        x_path_i = (int) round((x_path - environment_grid_.info.origin.position.x)/environment_grid_.info.resolution);
        y_path_i = (int) round((y_path - environment_grid_.info.origin.position.y)/environment_grid_.info.resolution);

        r = searchRadius(x_path_i,y_path_i);

        if (r < collision_free_r1_)
        {
            collision_free_r1_ = r;
//            ROS_INFO_STREAM("Minimum r = " << r);
        }
    }


//    for (int step_it = 0; step_it < search_steps; step_it++)
//    {
////        theta_search = ss[traj_i] + step_it*(ss[traj_i] + ss[traj_i + 1])/search_steps;
//        theta_search = step_it*(ss[traj_i + 1] - ss[traj_i])/search_steps;
//
////        ROS_INFO_STREAM("theta_search = " << theta_search);
//
////        x_path = (ref_path_x.m_a[traj_i]*(theta_search)*(theta_search)*(theta_search) + ref_path_x.m_b[traj_i]*(theta_search)*(theta_search) + ref_path_x.m_c[traj_i]*(theta_search) + ref_path_x.m_d[traj_i]);
////        y_path = (ref_path_y.m_a[traj_i]*(theta_search)*(theta_search)*(theta_search) + ref_path_y.m_b[traj_i]*(theta_search)*(theta_search) + ref_path_y.m_c[traj_i]*(theta_search) + ref_path_y.m_d[traj_i]);
//
//
//
//        x_path_i = (int) round((x_path - environment_grid_.info.origin.position.x)/environment_grid_.info.resolution);
//        y_path_i = (int) round((y_path - environment_grid_.info.origin.position.y)/environment_grid_.info.resolution);
//
////        ROS_INFO_STREAM( "Segment " << traj_i << " : searching around: x = " << x_path << ", y = " << y_path << " at index [" << x_path_i << ", " << y_path_i << "]." );
//
//        r = searchRadius(x_path_i,y_path_i);
//
////        ROS_INFO_STREAM("Found r = " << r);
//
//        if (r < collision_free_r1_)
//        {
//            collision_free_r1_ = r;
////            ROS_INFO_STREAM("Minimum r = " << r);
//        }
//
//    }

//    ROS_INFO_STREAM("ss[traj_i + 1] = " << ss[traj_i + 1] << " ss[traj_i + 2] = " << ss[traj_i + 2]);

//    for (int step_it = 0; step_it < search_steps; step_it++)
//    {
////        theta_search = ss[traj_i + 1] + step_it*(ss[traj_i + 1] + ss[traj_i + 2])/search_steps;
//        theta_search = step_it*(ss[traj_i + 2] - ss[traj_i + 1])/search_steps;
//
////        ROS_INFO_STREAM("theta_search = " << theta_search);
//
//        x_path = (ref_path_x.m_a[traj_i + 1]*(theta_search)*(theta_search)*(theta_search) + ref_path_x.m_b[traj_i + 1]*(theta_search)*(theta_search) + ref_path_x.m_c[traj_i + 1]*(theta_search) + ref_path_x.m_d[traj_i + 1]);
//        y_path = (ref_path_y.m_a[traj_i + 1]*(theta_search)*(theta_search)*(theta_search) + ref_path_y.m_b[traj_i + 1]*(theta_search)*(theta_search) + ref_path_y.m_c[traj_i + 1]*(theta_search) + ref_path_y.m_d[traj_i + 1]);
//
//        x_path_i = (int) round((x_path - environment_grid_.info.origin.position.x)/environment_grid_.info.resolution);
//        y_path_i = (int) round((y_path - environment_grid_.info.origin.position.y)/environment_grid_.info.resolution);
//
////        ROS_INFO_STREAM( "Segment " << (traj_i + 1) << " : searching around: x = " << x_path << ", y = " << y_path << " at index [" << x_path_i << ", " << y_path_i << "]." );
//
//        r = searchRadius(x_path_i,y_path_i);
//
//        if (r < collision_free_r2_)
//        {
//            collision_free_r2_ = r;
////            ROS_INFO_STREAM("Found r = " << r);
//        }
//    }

//    x_path = (ref_path_x.m_a[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_x.m_b[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_x.m_c[traj_i]*(acadoVariables.x[3]-ss[traj_i]) + ref_path_x.m_d[traj_i]);
//    y_path = (ref_path_y.m_a[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_y.m_b[traj_i]*(acadoVariables.x[3]-ss[traj_i])*(acadoVariables.x[3]-ss[traj_i]) + ref_path_y.m_c[traj_i]*(acadoVariables.x[3]-ss[traj_i]) + ref_path_y.m_d[traj_i]);

//    x_path_i = (int) round((x_path - environment_grid_.info.origin.position.x)/environment_grid_.info.resolution);
//    y_path_i = (int) round((y_path - environment_grid_.info.origin.position.y)/environment_grid_.info.resolution);

//    std::cout << "x_path: " << x_path << " y_path: " << y_path << std::endl;
//    std::cout << "x_path_i: " << x_path_i  << " y_path_i: " << y_path_i << std::endl;
//    std::cout << "occupancy: " << getOccupancy(x_path_i,y_path_i) << std::endl;

    ROS_INFO_STREAM( "Segment " << traj_i << " : r1 =  " << collision_free_r1_ << " r2 =  " << collision_free_r2_);

    te_ = acado_toc(&t);
    ROS_INFO_STREAM("Free space solve time " << te_ * 1e6 << " us");
}

double MPCC::searchRadius(int x_i, int y_i)
{

    double r = collision_free_r_max_, r_ = 0;
    int occupied_ = 90;
    int search_radius = 1;

    int search_x_it, search_y_it;

    // Search until an occupied region is found or until the maximum radius is obtained
    while (r == collision_free_r_max_ && search_radius < collision_free_r_max_/environment_grid_.info.resolution)
    {

        for (int search_x_it = -search_radius; search_x_it <= search_radius; search_x_it++ ) {

            if (x_i + search_x_it < 0){search_x_it = -x_i;}
            if (x_i + search_x_it > environment_grid_.info.width){search_x_it = environment_grid_.info.width - x_i;}

            search_y_it = -search_radius;
            if (y_i + search_y_it < 0){search_y_it = -y_i;}
            if (getOccupancy(x_i + search_x_it,y_i + search_y_it) > occupied_)
            {
                r_ = sqrt(pow(search_x_it,2) + pow(search_y_it,2))*environment_grid_.info.resolution;

                if (r_ < r)
                {
                    r = r_;
                }

            }

            search_y_it = search_radius;
            if (y_i + search_y_it > environment_grid_.info.height){search_y_it = environment_grid_.info.height - y_i;}
            if (getOccupancy(x_i + search_x_it,y_i + search_y_it) > occupied_)
            {
                r_ = sqrt(pow(search_x_it,2) + pow(search_y_it,2))*environment_grid_.info.resolution;

                if (r_ < r)
                {
                    r = r_;
                }

            }
        }

        for (int search_y_it = -search_radius; search_y_it <= search_radius; search_y_it++ ) {

            if (y_i + search_y_it < 0){search_y_it = -y_i;}
            if (y_i + search_y_it > environment_grid_.info.height){search_y_it = environment_grid_.info.height - y_i;}

            search_x_it = -search_radius;
            if (x_i + search_x_it < 0){search_x_it = -x_i;}
            if (getOccupancy(x_i + search_x_it,y_i + search_y_it) > occupied_)
            {
                r_ = sqrt(pow(search_x_it,2) + pow(search_y_it,2))*environment_grid_.info.resolution;

                if (r_ < r)
                {
                    r = r_;
                }

            }

            search_x_it = search_radius;
            if (x_i + search_x_it > environment_grid_.info.width){search_x_it = environment_grid_.info.width - x_i;}
            if (getOccupancy(x_i + search_x_it,y_i + search_y_it) > occupied_)
            {
                r_ = sqrt(pow(search_x_it,2) + pow(search_y_it,2))*environment_grid_.info.resolution;

                if (r_ < r)
                {
                    r = r_;
                }

            }
        }

        search_radius++;
//        std::cout << "search radius: " << search_radius << std::endl;
    }

    return r;
}

int MPCC::getOccupancy(int x_i, int y_i)
{
    return environment_grid_.data[environment_grid_.info.width*y_i + x_i];

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
            acadoVariables.od[(ACADO_NOD * N_iter) + 27] = r_discs_;                                // radius of car discs
            acadoVariables.od[(ACADO_NOD * N_iter) + 28] = 0; //x_discs_[1];                        // position of the car discs
        }

        traj_i = 0;
		goal_reached_ = false;
		last_poly_ = false;

        if (map_service_.call(map_srv_))
        {
            ROS_ERROR("Service GetMap succeeded.");
            environment_grid_ = map_srv_.response.map;
        }
        else
        {
            ROS_ERROR("Service GetMap failed.");
        }

        ConstructRefPath();
		publishSplineTrajectory();

        ComputeCollisionFreeArea();

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

	enable_output_ = config.enable_output;
	n_iterations_ = config.n_iterations;
	simulation_mode_ = config.simulation_mode;

    //Search window parameters
    window_size_ = config.window_size;
    n_search_points_ = config.n_search_points;
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
	for (int i = 0; i < 100; i++) // 100 points
	{
		spline_traj_.poses[i].pose.position.x = ref_path_x(i*(n_pts-1)*dist_spline_pts_/100.0); //x
		spline_traj_.poses[i].pose.position.y = ref_path_y(i*(n_pts-1)*dist_spline_pts_/100.0); //y

	}

	ROS_INFO_STREAM("REF_PATH_X size:  " << ref_path_x.m_a.size());
//	for(int i =0; i< ref_path_x.m_a.size();i++){
//		ROS_INFO_STREAM("REF_PATH_Xa:  " << i << "  " << ref_path_x.m_a[i]);
//		ROS_INFO_STREAM("REF_PATH_Xb:  " << i << "  " << ref_path_x.m_b[i]);
//		ROS_INFO_STREAM("REF_PATH_Xc:  " << i << "  " << ref_path_x.m_c[i]);
//		ROS_INFO_STREAM("REF_PATH_Xd:  " << i << "  " << ref_path_x.m_d[i]);
//		ROS_INFO_STREAM("REF_PATH_Ya:  " << i << "  " << ref_path_y.m_a[i]);
//		ROS_INFO_STREAM("REF_PATH_Yb:  " << i << "  " << ref_path_y.m_b[i]);
//		ROS_INFO_STREAM("REF_PATH_Yc:  " << i << "  " << ref_path_y.m_c[i]);
//		ROS_INFO_STREAM("REF_PATH_Yd:  " << i << "  " << ref_path_y.m_d[i]);
//	}

	spline_traj_pub_.publish(spline_traj_);
}

void MPCC::publishAnaliticSplineTrajectory(void)
{
	spline_traj2_.header.stamp = ros::Time::now();
	spline_traj2_.header.frame_id = controller_config_->tracking_frame_;
	double s,si;
    int j=0;
	for (int i = 0; i < 100; i++)
	{
        s= i*(n_pts-1)*dist_spline_pts_/100.0;
        if(s>ss[j+1])
            j+=1;
        si=s-ss[j];
		spline_traj2_.poses[i].pose.position.x = ref_path_x.m_a[j]*si*si*si+ref_path_x.m_b[j]*si*si+ref_path_x.m_c[j]*si+ref_path_x.m_d[j]; //x
		spline_traj2_.poses[i].pose.position.y = ref_path_y.m_a[j]*si*si*si+ref_path_y.m_b[j]*si*si+ref_path_y.m_c[j]*si+ref_path_y.m_d[j]; //y
        spline_traj2_.poses[i].header.stamp = ros::Time::now();
        spline_traj2_.poses[i].header.frame_id = controller_config_->tracking_frame_;
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

void MPCC::publishPosConstraint(){

    visualization_msgs::MarkerArray collision_free;

    for (int i = 0; i < ACADO_N; i++)
    {

//        if (acadoVariables.x[i * ACADO_NX + 3] > ss[traj_i + 1]){
//            ellips2.scale.x = collision_free_r2_*2.0;
//            ellips2.scale.y = collision_free_r2_*2.0;
//            ellips2.pose.position.x = (ref_path_x.m_a[traj_i + 1]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1]) + ref_path_x.m_b[traj_i + 1]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1]) + ref_path_x.m_c[traj_i + 1]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1]) + ref_path_x.m_d[traj_i + 1]);
//            ellips2.pose.position.y = (ref_path_y.m_a[traj_i + 1]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1]) + ref_path_y.m_b[traj_i + 1]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1]) + ref_path_y.m_c[traj_i + 1]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i + 1]) + ref_path_y.m_d[traj_i + 1]);
//        }
//        else
//        {
//            ellips2.scale.x = collision_free_r1_*2.0;
//            ellips2.scale.y = collision_free_r1_*2.0;
//            ellips2.pose.position.x = (ref_path_x.m_a[traj_i]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i]) + ref_path_x.m_b[traj_i]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i]) + ref_path_x.m_c[traj_i]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i]) + ref_path_x.m_d[traj_i]);
//            ellips2.pose.position.y = (ref_path_y.m_a[traj_i]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i]) + ref_path_y.m_b[traj_i]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i])*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i]) + ref_path_y.m_c[traj_i]*(acadoVariables.x[i * ACADO_NX + 3]-ss[traj_i]) + ref_path_y.m_d[traj_i]);
//        }


        ellips2.scale.x = collision_free_r1_*2.0;
        ellips2.scale.y = collision_free_r1_*2.0;
        ellips2.pose.position.x = acadoVariables.x[i * ACADO_NX + 0];
        ellips2.pose.position.y = acadoVariables.x[i * ACADO_NX + 1];

        ellips2.id = 400+i;
        ellips2.pose.orientation.x = 0;
        ellips2.pose.orientation.y = 0;
        ellips2.pose.orientation.z = 0;
        ellips2.pose.orientation.w = 1;
        collision_free.markers.push_back(ellips2);
    }

    collision_free_pub_.publish(collision_free);
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

    feedback_msg.reference_path = spline_traj2_;
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
