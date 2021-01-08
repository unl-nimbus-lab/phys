/*
 * Copyright (c) 2012, Brennand Pierce
 * Bren@tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "ros/assert.h"
#include "sensor_msgs/JointState.h"

#include "bioloid_interface/bioloid_msg.h"

extern "C" {
	#include "dynamixel_SDK/dynamixel.h"
}

#include <XmlRpcValue.h>

#include <string>
#include <vector>
#include <math.h>

//Function prototypes
void desiredCallback(const sensor_msgs::JointState::ConstPtr& msg);
bool init_dynamixel();
template <typename T> void getParamVector (ros::NodeHandle n, const std::string Var, std::vector<T>* const Vec);

//Definition of the bits set in the dynamixels ( see Robotis Dynamixel Wiki )
#define P_ID   					3
#define P_TORQUE_ENABLED		24
#define P_GOAL_POSITION_L   	30
#define P_GOAL_POSITION_H   	31
#define P_GOAL_SPEED_L			32
#define P_GOAL_SPEED_H			33
#define P_GOAL_TORQUELIMIT_L	34
#define P_GOAL_TORQUELIMIT_H	35
#define P_PRESENT_POSITION_L  	36
#define P_PRESENT_POSITION_H  	37
#define P_PRESENT_SPEED_L    	38
#define P_PRESENT_SPEED_H    	39
#define P_PRESENT_LOAD_L    	40
#define P_PRESENT_LOAD_H    	41
#define P_MOVING       			46

//Defines to calculate from tick to SI units
#define TICK2RAD				*300*2*M_PI/360/1023 // convert Ticks to Radiant
#define RAD2TICK				/300/2/M_PI*360*1023 // convert Radiant to Ticks
#define TICKSPSEC2RADPSEC		*2*M_PI/60*0.111 //convert Ticks/sec to Radiant/sec
#define RADPSEC2TICKSPSEC		/2/M_PI*60/0.111 // convert Radiant/sec to Ticks/sec

//Values from the para server.
std::vector<std::string> name; //Stores joint names

std::vector<int> servo_number; //The servo number, used to communicate with servo
std::vector<int> joint_encoder_offset; //The offset of the servo to make the standard home position
std::vector<double> angle_max; //Max angle, this could be used to limit the range of motor
std::vector<double> angle_min; //Min angle, this could be used to limit the range of motor

//Vales to send to the bioloid:
std::vector<int> des_pos; //tick
std::vector<int> des_vel; //tick/s
std::vector<double> des_pos_rad; //rad
std::vector<double> des_vel_rad; //rad/s

//Values to publish:
std::vector<double> pos; //rad
std::vector<double> vel; //rad/s
std::vector<double> eff; //not used yet, could get motor current.

std::vector<double> raw_pos; //tick
std::vector<double> raw_vel; //tick/s
std::vector<double> raw_eff; //not used yet, could get motor current.

std::vector<bool> motor_torque;

bool torque; // this turns the motors on and off;
bool sim; //simulation mode on/off


void bioloidMsgCallback(const bioloid_interface::bioloid_msg::ConstPtr& msg){
	//check msg size
	if(msg->motor_torque.size() == motor_torque.size()){
		//do the actual copy between the two vectors
		for(unsigned int i=0;i<motor_torque.size();i++)
			motor_torque[i] = msg->motor_torque[i];
	}
}

/******************************************************************
 *
 * 			This function is called when we receive a desired message, in
 *			the case of the bioloid this is rad or rad/s
 *
 ******************************************************************/
void desiredCallback(const sensor_msgs::JointState::ConstPtr& msg) {

	// compare joint names (std::strings) to set the new values to correct joint numbers
	// iterates through requested joints
	if (msg->name.size() > 0) {
		for (uint i = 0; i < msg->name.size(); i++) {
			std::string joint_name = msg->name[i];

			unsigned int j = static_cast<unsigned int>(std::find(name.begin(), name.end(), joint_name) - name.begin());
			if (i < msg->position.size() && j < des_pos_rad.size()) {
				des_pos_rad[j] = msg->position[i];
				double pos_rad = des_pos_rad[j];

				if (pos_rad > angle_max[j]) {
					pos_rad = angle_max[j];
				} else if (pos_rad < angle_min[j]) {
					pos_rad = angle_min[j];
				}

				int p = (int) (pos_rad RAD2TICK ) + joint_encoder_offset[j]; //We recieve rad, we convert to ticks
				if (p > 1023) {
					des_pos[j] = 1023;
				} else if (p < 0) {
					des_pos[j] = 0;
				} else {
					des_pos[j] = p;
				}
			}

			// For desired velocity
			if (i < msg->velocity.size() && j < des_vel_rad.size()) {
				des_vel_rad[j] = msg->velocity[i];
				int v = (int) (des_vel_rad[j] RADPSEC2TICKSPSEC ); //We receive rad/s, we convert to tick/s.
				if (v > 1023) {
					des_vel[j] = 1023;
				} else if (v < 0) {
					des_vel[j] = 0;
				} else {
					des_vel[j] = v;
				}
			}

			// So people can turn each motor on and off
			if (i < msg->effort.size() && j < motor_torque.size()) {
				if(msg->effort[i] == 1){
					motor_torque[j] = false;
				}else{
					motor_torque[j] = true;
				}
			}


		}
	}
}

/******************************************************************
 *
 * 			Main loop.
 *
 ******************************************************************/
int main(int argc, char **argv) {

	ros::init(argc, argv, "bioloid_interface");

	ros::NodeHandle n("~");
	//This is the msg listener and the msg caster.
	ros::Publisher js_pub = n.advertise<sensor_msgs::JointState> ("state", 1000);
	ros::Publisher raw_js_pub = n.advertise<sensor_msgs::JointState> ("raw_state", 1000);

	ros::Subscriber sub = n.subscribe("/bioloid_interface/command", 1000, desiredCallback);
	ros::Subscriber sub2 = n.subscribe("bioloid_msg", 1000, bioloidMsgCallback);

	/******************************************************************
	 * 			Get all the variables from the parameter server.
	 ******************************************************************/
	getParamVector<std::string>(n, "/bioloid/joints/name", &name);
	n.getParam("/bioloid/joints/torque", torque);

	getParamVector<int>(n, "/bioloid/joints/servo_number", &servo_number);
	getParamVector<int>(n, "/bioloid/joints/joint_encoder_offset", &joint_encoder_offset);

	getParamVector<double>(n, "/bioloid/joints/angle_max", &angle_max);
	getParamVector<double>(n, "/bioloid/joints/angle_min", &angle_min);

	if (!n.getParam("/bioloid/joints/sim", sim)) {
		ROS_INFO("getPara(sim) failed. Enter simulation mode by default.");
		sim = true;
	}

	ROS_ASSERT(	  (	name.size() &
					servo_number.size() &
					joint_encoder_offset.size() &
					angle_max.size() 	&
					angle_min.size() ) == name.size() );

	// Make the code more readable by using number_of_joints for loops.
	int number_of_joints = name.size();

	/******************************************************************
	 * 			Setup ROS
	 ******************************************************************/
	//Communicate at 200Hz
	ros::Rate loop_rate(200);
	//Initialize the time.
	ros::Time::init();
	//ROS counter
	int counter = 0;

	ROS_INFO("Number of joints: %d", number_of_joints);
	ROS_INFO("Bioloid interface started.");

	/******************************************************************
	 * 			Initialize all the vectors so that number of joints = vector size.
	 ******************************************************************/
	pos.resize(number_of_joints,0.0);
	pos.resize(number_of_joints,0.0);
	vel.resize(number_of_joints,0.0);
	eff.resize(number_of_joints,0.0);

	raw_pos.resize(number_of_joints,0.0);
	raw_vel.resize(number_of_joints,0.0);
	raw_eff.resize(number_of_joints,0.0);

	des_pos.resize(number_of_joints,0.0);
	std::vector<int> temp_joint_encoder_(joint_encoder_offset);
	des_pos.swap(temp_joint_encoder_); //home position
//	std::cout << "Joint encoder offset: ";
//	std::copy(joint_encoder_offset.begin(), joint_encoder_offset.end(), std::ostream_iterator<int>(std::cout, " "));
//	std::cout << std::endl;

	des_vel.resize(number_of_joints,0);
	des_pos_rad.resize(number_of_joints,0.0);
//	//init des_pos_rad
//	for(int i=0;i<number_of_joints;i++){
//		des_pos_rad[i] = des_pos[i] TICK2RAD;
//	}
	des_vel_rad.resize(number_of_joints,0.0);
	motor_torque.resize(number_of_joints,torque);

	/******************************************************************
	 * 			Connect to bioloid
	 ******************************************************************/
	if (!sim) {
		if (init_dynamixel() == true) {
			ROS_INFO("Connected to bioloid");
		} else {
			ROS_INFO("Failed to connected bioloid");
			ros::shutdown();
			return 0;
		}
	} else {
		ROS_INFO("In sim model, so map command to state, for joints");
	}

	if(!torque){
		ROS_INFO( "Motors are turned off." );
		do {
			dxl_write_byte(254, P_TORQUE_ENABLED, 0);
		} while (dxl_get_result() != COMM_RXSUCCESS);
	}

	/******************************************************************
	 *
	 * 				Main control loop.
	 *
	 ******************************************************************/
	while (ros::ok()) {

		//Send Joint states
		counter++;
		sensor_msgs::JointState js;
		js.header.seq = counter;
		js.header.stamp = ros::Time::now();
		js.header.frame_id = "/world";

		/******************************************************************
		 * 				Get Dynamixel position and speed
		 ******************************************************************/
		if (sim) {
			pos = des_pos_rad;
			vel = des_vel_rad;

		} else {

			//ROS_INFO("connected to robot");
			for (int i = 0; i < number_of_joints; i++) {

				//Get raw position from dynamixel
				do {
					raw_pos.at(i) = (double) dxl_read_word(servo_number.at(i), P_PRESENT_POSITION_L);
				} while (dxl_get_result() != COMM_RXSUCCESS);


				//get raw velocity from dynamixel
				do {
					raw_vel.at(i) = (double) dxl_read_word(servo_number.at(i), P_PRESENT_SPEED_L);
				} while (dxl_get_result() != COMM_RXSUCCESS);


				//Work out if vel direction, 
				//If a value is in the rage of 0~1023, it means that the motor rotates to the CCW direction.
				//If a value is in the rage of 1024~2047, it means that the motor rotates to the CW direction.

				if (raw_vel.at(i) > 1024.0) {
					raw_vel.at(i) = -(raw_vel.at(i) - 1024.0);
				}

				//convert the raw into real number.
				vel.at(i) = (raw_vel.at(i) TICKSPSEC2RADPSEC );
				pos.at(i) = ((raw_pos.at(i) - joint_encoder_offset.at(i)) TICK2RAD );

			}

			/******************************************************************
			 * 				Send the desired position to the Dynamixel
			 ******************************************************************/
			for (int i = 0; i < number_of_joints; i++) {
				if(motor_torque.at(i)){

					do {
						dxl_write_word(servo_number.at(i), P_GOAL_SPEED_L,(unsigned) des_vel.at(i));
					} while (dxl_get_result() != COMM_RXSUCCESS);

					do {
						dxl_write_word(servo_number.at(i), P_GOAL_POSITION_L,(unsigned) (des_pos.at(i)));
					} while (dxl_get_result() != COMM_RXSUCCESS);
					//if not turn the motors off
				} else {
					do {
						dxl_write_byte(servo_number.at(i), P_TORQUE_ENABLED, 0);
					} while (dxl_get_result() != COMM_RXSUCCESS);
				}
			}

		}

		/******************************************************************
		 * 				Publish the raw and real positon on the joints
		 ******************************************************************/
		js.name = name;
		js.position = pos;
		js.velocity = vel;
		js.effort = eff;

		js_pub.publish(js);

		js.position = raw_pos;
		js.velocity = raw_vel;
		js.effort = raw_eff;

		raw_js_pub.publish(js);

		ros::spinOnce();
		loop_rate.sleep();
	}

	/******************************************************************
	 *
	 * 			Before exit turn everything off.
	 *
	 ******************************************************************/
	if (!sim) {
		//turn the torque of the servo off
		do {
			dxl_write_byte(254, P_TORQUE_ENABLED, 0);
		}
		while (dxl_get_result() != COMM_RXSUCCESS);

		//close the dynamical port.
		dxl_terminate();
	}

	return 0;
}




/******************************************************************
 *
 * 			try and connect to the bioloid if succesful return true, other wish return false.
 *
 ******************************************************************/
bool init_dynamixel() {

	// setup initial data for the Dynamixels
	int baudnum = 1;

	for (unsigned int deviceIndex = 0; deviceIndex < 10; deviceIndex++) {
		if (dxl_initialize(deviceIndex, baudnum) == 0) {
			// not possible to connect to dynamixels
			ROS_INFO( "[initializeDynamixel] Failed to open USB2Dynamixel on /dev/ttyUSB%d", deviceIndex );

		} else {
			// connection succesfull
			ROS_INFO( "[initializeDynamixel] Succeed to open USB2Dynamixel on /dev/ttyUSB%d", deviceIndex );
			return true;
		}
//
//		if (deviceIndex == 9) {
//			return false;
//		}
	}

	return false;

}

/******************************************************************
 *
 * 		It returns the parameter server value of type T.
 *
 ******************************************************************/
template <typename T> void getParamVector (ros::NodeHandle n, const std::string Var, std::vector<T>* const Vec){
	XmlRpc::XmlRpcValue gainList;
	n.getParam(Var, gainList);
	ROS_ASSERT(gainList.getType() == XmlRpc::XmlRpcValue::TypeArray);

	//create a dummy object for sanity check
	T* dummy = new T;
	XmlRpc::XmlRpcValue obj(*dummy);

	for (int index = 0; index < gainList.size(); index++) {
		ROS_ASSERT(gainList[index].getType() == obj.getType());
		Vec->push_back(static_cast<T> (gainList[index]));
	}

	//clean-up
	delete dummy;
}
