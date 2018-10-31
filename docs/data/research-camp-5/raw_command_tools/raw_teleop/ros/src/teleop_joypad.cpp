/*
 * teleop_joypad.cpp
 *
 *  Created on: May 27, 2012
 *      Author: Frederik Hegger
 */

#include "teleop_joypad.h"

TeleOpJoypad::TeleOpJoypad(ros::NodeHandle &nh)
{
	is_in_soft_joint_limits_ = false;
	button_deadman_pressed_ = false;
	button_deadman_pressed_prev_ = false;
	button_run_pressed_ = false;
    button_print_arm_states_ = false;
    button_print_arm_states_prev_ = false;

	double param = 0;
	ros::param::param<double>("~base_max_linear_x_vel", param, 0.3);
	base_factor_.linear.x =  param / MAX_JOYPAD;
	ros::param::param<double>("~base_max_linear_y_vel", param, 0.3);
	base_factor_.linear.y =  param / MAX_JOYPAD;
	ros::param::param<double>("~base_max_angular_vel", param, 0.5);
	base_factor_.angular.z	 =  param / MAX_JOYPAD;
	ros::param::param<double>("~arm_max_vel", param, 0.2);
	arm_max_vel_ =  param / MAX_JOYPAD;
	ros::param::param<double>("~soft_joint_limit_threshold", soft_joint_limit_threshold_, 0.05);

	// read joint names
	XmlRpc::XmlRpcValue param_list;
	nh.getParam("/arm_1/arm_controller/joints", param_list);
	ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for (int32_t i = 0; i < param_list.size(); ++i)
	{
	  ROS_ASSERT(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
	  arm_joint_names_.push_back(static_cast<std::string>(param_list[i]));
	}

	//read joint limits
	for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
	{
		arm_navigation_msgs::JointLimits limit;
		limit.joint_name = arm_joint_names_[i];
		nh.getParam("/arm_1/arm_controller/limits/" + arm_joint_names_[i] + "/min", limit.min_position);
		nh.getParam("/arm_1/arm_controller/limits/" + arm_joint_names_[i] + "/max", limit.max_position);
		arm_joint_limits_.push_back(limit);

	}


	arm_vel_.velocities.clear();
	for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
	{
		brics_actuator::JointValue joint_value;

		joint_value.timeStamp = ros::Time::now();
		joint_value.joint_uri = arm_joint_names_[i];
		joint_value.unit = to_string(boost::units::si::radian_per_second);
		joint_value.value = 0.0;

		arm_vel_.velocities.push_back(joint_value);
	}

	sub_joy_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleOpJoypad::cbJoy, this);
	sub_joint_states_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &TeleOpJoypad::cbJointStates, this);

	pub_base_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_arm_vel = nh.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);

	srv_arm_motors_on = nh.serviceClient<std_srvs::Empty>("arm_1/switchOnMotors");
	srv_arm_motors_off = nh.serviceClient<std_srvs::Empty>("arm_1/switchOffMotors");
}

void TeleOpJoypad::cbJoy(const sensor_msgs::Joy::ConstPtr& command)
{
	//general
	button_deadman_pressed_prev_ = button_deadman_pressed_;
	button_deadman_pressed_ = (bool)command->buttons[BUTTON_DEADMAN];
	button_run_pressed_ = (bool)command->buttons[BUTTON_RUN];

    button_print_arm_states_prev_ = button_print_arm_states_;
    button_print_arm_states_ = (bool)command->buttons[BUTTON_PRINT_ARM_JOINT_STATES];

	//base
	if(!button_run_pressed_)
		speed_factor_ = 0.5;
	else
		speed_factor_ = 1.0;

	base_vel_.linear.x = command->axes[AXES_BASE_LINEAR_X_SPEED] * base_factor_.linear.x * speed_factor_;
	base_vel_.linear.y = command->axes[AXES_BASE_LINEAR_Y_SPEED] * base_factor_.linear.y * speed_factor_;
	base_vel_.angular.z = command->axes[AXES_BASE_ANGULAR_SPEED] * base_factor_.angular.z * speed_factor_;

	if(fabs(base_vel_.linear.x) < 0.05)
		base_vel_.linear.x = 0;
	if(fabs(base_vel_.linear.y) < 0.05)
		base_vel_.linear.y = 0;
	if(fabs(base_vel_.angular.z) < 0.01)
		base_vel_.angular.z = 0;

	//arm
	if((bool)command->buttons[BUTTON_ARM_MOTORS_ON])
		turnOnArmMotorsOn();
	else if((bool)command->buttons[BUTTON_ARM_MOTORS_OFF])
		turnOnArmMotorsOff();

	if((bool)command->buttons[BUTTON_ARM_MOTOR_1_2])
	{
		arm_vel_.velocities[0].value = command->axes[AXES_ARM_1] * arm_max_vel_ * speed_factor_ * (-1.0);
		arm_vel_.velocities[1].value = command->axes[AXES_ARM_2] * arm_max_vel_ * speed_factor_;
	}
	else if((bool)command->buttons[BUTTON_ARM_MOTOR_3_4])
	{
		arm_vel_.velocities[2].value = command->axes[AXES_ARM_1] * arm_max_vel_ * speed_factor_;
		arm_vel_.velocities[3].value = command->axes[AXES_ARM_2] * arm_max_vel_ * speed_factor_;
	}
	else if((bool)command->buttons[BUTTON_ARM_MOTOR_5])
		arm_vel_.velocities[4].value = command->axes[AXES_ARM_1] * arm_max_vel_ * speed_factor_ * (-1.0);

}

void TeleOpJoypad::cbJointStates(const sensor_msgs::JointState::ConstPtr& state_msg)
{
	current_joint_states_ = *state_msg;
}

void TeleOpJoypad::turnOnArmMotorsOn()
{
	std_srvs::Empty empty;

	if(srv_arm_motors_on.call(empty))
		ROS_INFO("Turned ON arm motors");
	else
		ROS_ERROR("Could not turn ON arm motors");

	sleep(1);
}

void TeleOpJoypad::turnOnArmMotorsOff()
{
	std_srvs::Empty empty;

	if(srv_arm_motors_off.call(empty))
		ROS_INFO("Turned OFF arm motors");
	else
		ROS_ERROR("Could not turn OFF arm motors");

	sleep(1);
}

void TeleOpJoypad::setSingleArmJointVel(double motor_vel, std::string joint_name)
{
	for(unsigned int i=0; i < arm_vel_.velocities.size(); ++i)
	{
		if(arm_vel_.velocities[i].joint_uri == joint_name)
		{
			arm_vel_.velocities[i].timeStamp = ros::Time::now();
			arm_vel_.velocities[i].value = motor_vel;
		}
	}
}

void TeleOpJoypad::setAllArmJointVel(double motor_vel)
{
	for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
		setSingleArmJointVel(motor_vel, arm_joint_names_[i]);
}

void TeleOpJoypad::printArmJointStates()
{
    std::string joint_name_list = "";

	std::cout << "[";
    
    for(unsigned int i =0; i < arm_joint_limits_.size(); i++)
	{
		for(unsigned int j=0; j < current_joint_states_.name.size(); ++j)
		{
			if(current_joint_states_.name[j] == arm_joint_limits_[i].joint_name)
			{
				std::cout << current_joint_states_.position[j];
				joint_name_list += current_joint_states_.name[j];

				if(i < (arm_joint_limits_.size() - 1))
				{
					std::cout << ", ";
					joint_name_list += ", ";
				}
			}
		}
	}
    std::cout << "] \t # current arm joint values (" << joint_name_list << ")" << std::endl;
        
}

void TeleOpJoypad::checkArmJointLimits()
{
	for(unsigned int i =0; i < arm_joint_limits_.size(); i++)
	{
		for(unsigned int j = 0; j < current_joint_states_.name.size(); ++j)
		{
            if(current_joint_states_.name[j] == arm_joint_limits_[i].joint_name)
            {
                if(((current_joint_states_.position[j] < (arm_joint_limits_[i].min_position + soft_joint_limit_threshold_)) && (arm_vel_.velocities[j].value < 0)) || 
                   ((current_joint_states_.position[j] > (arm_joint_limits_[i].max_position - soft_joint_limit_threshold_)) && (arm_vel_.velocities[j].value > 0)))
                {
    				//ROS_ERROR_STREAM("arm joint <<" << arm_joint_limits_[i].joint_name << ">> at soft joint limit: " << current_joint_states_.position[j] << " rad");
    				arm_vel_.velocities[i].value = 0.0;
    			}
            }
		}
	}
}

void TeleOpJoypad::publishCommands()
{
	if(button_deadman_pressed_)
	{
		checkArmJointLimits();

		//pub_arm_vel.publish(arm_vel_);
		pub_base_vel.publish(base_vel_);
	}

	else
	{
		if(button_deadman_pressed_prev_)
		{
			pub_base_vel.publish(base_zero_vel_);
			//setAllArmJointVel(0.0);
			//pub_arm_vel.publish(arm_vel_);
		}
	}

    if(button_print_arm_states_prev_ == false && button_print_arm_states_ == true)
        printArmJointStates();
}

