#include "vectornav/vectornav.h"
#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include "geometry_msgs/Vector3Stamped.h"

using namespace std;

const char* const COM_PORT = "/dev/ttyUSB1";
const int BAUD_RATE = 115200;


int main(int argc, char **argv)
{
	///Variable Declerations
	Vn100 vn100;
	VnYpr ypr;
	
	
	///ROS Node Initialization
	ros::init(argc, argv, "VN_100_Heading_Extractor"); 
	ros::NodeHandle imu;
	ros::Publisher heading_publisher = imu.advertise<geometry_msgs::Vector3Stamped>("/heading", 1);


	///Connection to Vectornav
	int connect = 0;
	ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	ROS_INFO("Connected ...");
	while(connect != 0){
		ROS_ERROR("Not Connected ...");
		usleep(999999);
 	}
	ROS_INFO("Server initialized...");
	
	
	///Publishing Loops
	int count=0;
	ros::Rate loop_rate(20);
	ros::Time current_time; 
	geometry_msgs::Vector3Stamped heading_;
	
	while (ros::ok()) {
		/*Getting Data*/
		if(vn100.isConnected){		
			ros::Time imu_time = ros::Time::now();	
			vn100_getYawPitchRoll(&vn100, &ypr);
			current_time=ros::Time::now();
			ROS_INFO("Yaw: %f", ypr.yaw);
		}else ROS_INFO("Connection Error...");
		
		/*Filling in Data*/
			//Header Info//
			heading_.header.stamp=current_time;
			heading_.header.seq=count;
			//Data Values//
			heading_.vector.x=ypr.yaw; heading_.vector.y=ypr.pitch; heading_.vector.z=ypr.roll;
	
		/*Publishing Data*/
		heading_publisher.publish(heading_);;
	
		/*Other Stuff*/
		count++;
		loop_rate.sleep(); 
	}
}
	
