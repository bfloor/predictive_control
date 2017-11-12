
//This file containts cost function intsert in to generated trajectory.

#include <predictive_control/predictive_trajectory_generator.h>


predictive_config::predictive_config()
{
	nh =  ros::this_node::getName() ;

	activate_output = false;

	// Chain param
	dof = 7;
	base_link = "arm_base_link";
	tip_link = "arm_7_link";
	root_frame = "arm_base_link";

	position_tolerance_violate = false;
	velocity_tolerance_violate = false;

	min_discretization_steps = max_discretization_steps = discretization_steps = 10;
	min_position_limit = -1.0; max_position_limit = 1.0;
	min_velocity_limit = max_velocity_limit = desired_velocity = 1.5;
	position_tolerance = 1.0; velocity_tolerance = 0.1;

	//limits_tolerance = 10.0;

}

predictive_config::~predictive_config()
{
	jnts_name.clear();
}

void predictive_config::update_config_parameters(predictive_config& new_param)
{
	dof = new_param.dof;
	base_link = new_param.base_link;
	tip_link = new_param.tip_link;
	root_frame = new_param.root_frame;
	jnts_name = new_param.jnts_name;

	min_position_limit = new_param.min_position_limit;
	max_position_limit = new_param.max_position_limit;
	min_velocity_limit = new_param.min_velocity_limit;
	max_velocity_limit = new_param.max_velocity_limit;
	desired_velocity = new_param.desired_velocity;
	position_tolerance = new_param.position_tolerance;
	velocity_tolerance = new_param.velocity_tolerance;

	min_discretization_steps = new_param.min_discretization_steps;
	max_discretization_steps = new_param.max_discretization_steps;
	discretization_steps = new_param.discretization_steps;
}

void predictive_config::choose_discretization_steps()
{
	if (discretization_steps == 0)
	{
		discretization_steps = static_cast<int>( std::ceil((min_discretization_steps + max_discretization_steps)*0.5) );
	}
}

void predictive_config::enforce_position_limit(double current_position, double& corrected_position)
{
	// Current position inside the range.
	if ( (current_position - min_position_limit > 0) || (min_position_limit + position_tolerance < current_position) )
	{
		corrected_position = current_position;
	}

	// Current position outside the range, and also near to minimum limit.
	else if ( ((current_position - min_position_limit < 0)  || (min_position_limit + position_tolerance > current_position))
				&& (std::abs(current_position - min_position_limit) ) < std::abs(max_position_limit - current_position) )
	{
		corrected_position = min_position_limit;
	}

	// Current position inside the range.
	else if ( (  max_position_limit - current_position  > 0) || (max_position_limit - position_tolerance > current_position) )
	{
		corrected_position = current_position;
	}

	// Current position outside the range, and also near to maximum limit.
	else if ( ((max_position_limit - current_position  < 0)  || (max_position_limit - position_tolerance < current_position))
				&& (std::abs(current_position - min_position_limit) ) > std::abs(max_position_limit - current_position) )
	{
		corrected_position = max_position_limit;
	}

	else
	{
		ROS_WARN("No position limit satisfied, need to check code and limit");
	}
}

bool predictive_config::check_position_tolerance_violation(double current_position)
{
	if ( current_position - (min_position_limit - position_tolerance) < 0)
	{
		position_tolerance_violate = true;
		ROS_WARN("Position tolerance violate with current position %f required position %f ... check_position_tolerance_violation",
					current_position, (min_position_limit - position_tolerance) );
		return true;
	}
	else if ( current_position - (max_position_limit + position_tolerance) > 0)
	{
		position_tolerance_violate = true;
		ROS_WARN("Position tolerance violate with current position %f required position %f ... check_position_tolerance_violation",
					current_position, (max_position_limit + position_tolerance) );
		return true;
	}

	return false;
}

void predictive_config::enforce_velocity_limit(double current_velocity, double& corrected_velocity)
{
	// Current position inside the range.
	if ( (current_velocity - min_position_limit > 0) || (min_position_limit + position_tolerance < current_velocity) )
	{
		corrected_velocity = current_velocity;
	}

	// Current position outside the range, and also near to minimum limit.
	else if ( ((current_velocity - min_position_limit < 0)  || (min_position_limit + position_tolerance > current_velocity))
				&& (std::abs(current_velocity - min_position_limit) ) < std::abs(max_position_limit - current_velocity) )
	{
		corrected_velocity = min_position_limit;
	}

	// Current position inside the range.
	else if ( (  max_position_limit - current_velocity  > 0) || (max_position_limit - position_tolerance > current_velocity) )
	{
		corrected_velocity = current_velocity;
	}

	// Current position outside the range, and also near to maximum limit.
	else if ( ((max_position_limit - current_velocity  < 0)  || (max_position_limit - position_tolerance < current_velocity))
				&& (std::abs(current_velocity - min_position_limit) ) > std::abs(max_position_limit - current_velocity) )
	{
		corrected_velocity = max_position_limit;
	}

	else
	{
		ROS_WARN("No velocity limit satisfied, need to check code and limit ... enforce_velocity_limit");
	}
}

bool predictive_config::check_velocity_tolerance_violation(double current_velocity)
{
	if ( current_velocity - (min_velocity_limit - velocity_tolerance) < 0)
	{
		velocity_tolerance_violate = true;
		ROS_WARN("Velocity tolerance violate with current velocity %f required velocity %f ... check_velocity_tolerance_violation",
				current_velocity, (min_velocity_limit - velocity_tolerance) );
		return true;
	}
	else if ( current_velocity - (max_velocity_limit + velocity_tolerance) > 0)
	{
		velocity_tolerance_violate = true;
		ROS_WARN("Velocity tolerance violate with current velocity %f required velocity %f ... check_velocity_tolerance_violation",
				current_velocity, (max_velocity_limit + velocity_tolerance) );
		return true;
	}

	return false;
}
