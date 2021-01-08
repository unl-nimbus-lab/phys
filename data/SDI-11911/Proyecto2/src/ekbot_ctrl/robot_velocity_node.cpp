#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include <math.h>
#include <stdio.h>
//#include <turtlesim/Pose.h>
#include <Eigen/Geometry>
#include "gazebo_msgs/ApplyJointEffort.h"
#include <math.h>

using namespace Eigen;

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Twist robot_position;
geometry_msgs::Twist velocity_msg;

//rate_hz assignment
double rate_hz = 1;

// Transform robot velocities (x,y,w) to motor velocities (m1,m2,m3,m4)
double* getMotorValue(int x_velocity, int y_velocity, int w_velocity){
	double deg1 = 3 * M_PI / 10;
	double deg2 = 3 * M_PI / 10;
	double deg3 = 7 * M_PI / 30;
	double deg4 = 7 * M_PI / 30;
	double s1 = sin(deg1), c1 = cos(deg1);
	double s2 = sin(deg2), c2 = cos(deg2);
	double s3 = sin(deg3), c3 = cos(deg3);
	double s4 = sin(deg4), c4 = cos(deg4);
	double R = 8.5;
	double r = 3.4;

	double velXMod = x_velocity * 3.5 / ( 2 * M_PI * r );
    double velYMod = y_velocity * 3.5 / ( 2 * M_PI * r );
    double velWMod = w_velocity * 3.5 / ( 2 * M_PI * r );

    velWMod = R * velWMod;
    double velMots[4];

    velMots[0] = (double)( s1 * velXMod + c1 * velYMod + velWMod);
    velMots[1] = (double)(-s2 * velXMod + c2 * velYMod + velWMod);
    velMots[2] = (double)(-s3 * velXMod - c3 * velYMod + velWMod);
    velMots[3] = (double)( s4 * velXMod - c4 * velYMod + velWMod);

    return velMots;
}

void get_vel_vec(const geometry_msgs::Twist& msg) {
	velocity_msg.linear.x = msg.linear.x;
	velocity_msg.linear.y = msg.linear.y;
    velocity_msg.angular.z = msg.linear.z; 
}


int main(int argc, char **argv){
	ros::init(argc,argv,"robot_velocity_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("robot_velocity_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());
	
	// Suscribe to Gazebo service ApplyJointEffor
	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort eff_msg[4];
	

	ros::Subscriber sub_vel = nh.subscribe("/target_vel_topic", 1000, &get_vel_vec);

	double tiempo = 0;

    //define the max speed
	double cruise_speed = 50;

    //define the rate
	ros::Rate rate(rate_hz);
	ros::Time start_time ;
	ros::Duration duration ;

	double effort[4];
	
	while (ros::ok())
	{
	double* velMots = getMotorValue(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.angular.z);
	
	effort [0] = velMots[0];
	effort [1] = velMots[1];
	effort [2] = velMots[2];
	effort [3] = velMots[3];
  
		if(client.exists()){
			
			start_time.sec = 0;
			start_time.nsec = 0;
			duration.sec = 1/rate_hz;
			duration.nsec = 0;

			// Wheel-Joint 1
			eff_msg[0].request.joint_name = "chassis_JOINT_1";
			eff_msg[0].request.duration = duration;
			eff_msg[0].request.effort = effort[0];
			eff_msg[0].request.start_time = start_time;

			// Wheel-Joint 2
			eff_msg[1].request.joint_name = "chassis_JOINT_2";
			eff_msg[1].request.duration = duration;
			eff_msg[1].request.effort = effort[1];
			eff_msg[1].request.start_time = start_time;

			// Wheel-Joint 3
			eff_msg[2].request.joint_name = "chassis_JOINT_3";
			eff_msg[2].request.duration = duration;
			eff_msg[2].request.effort = effort[2];
			eff_msg[2].request.start_time = start_time;

			// Wheel-Joint 4
			eff_msg[3].request.joint_name = "chassis_JOINT_4";
			eff_msg[3].request.duration = duration;
			eff_msg[3].request.effort = effort[03];
			eff_msg[3].request.start_time = start_time;

			client.call(eff_msg[0]);
			client.call(eff_msg[1]);																																																																								
			client.call(eff_msg[2]);
			client.call(eff_msg[3]);
			ROS_INFO_STREAM("Joints ==> 1: " << ((eff_msg[0].response.success == 1) ? "TRUE" : "FALSE") <<
			" 2: " << ((eff_msg[1].response.success == 1) ? "TRUE" : "FALSE") <<
			" 3: " << ((eff_msg[2].response.success == 1) ? "TRUE" : "FALSE") <<
			" 4: " << ((eff_msg[3].response.success == 1) ? "TRUE" : "FALSE"));
		}																																																																																																
		
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}
