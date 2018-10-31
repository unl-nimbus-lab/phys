#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <unistd.h>
#include "vectornav/vectornav.h"

using namespace std;
#include <iostream>

const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 57600;
float heading;		/*Floats for the returned values*/
float trans[3][3];

int main(int argc, char **argv)
{
	Vn100 vn100;
	VnYpr ypr;
	VnQuaternion quaternion;
	VnVector3 magnetic;
	VnVector3 acceleration;
	VnVector3 angularRate;

	int connect = 0;
	ros::init(argc, argv, "IMU_Interface"); //init the driver 
	ros::NodeHandle imu; //create a handle for the node - this does the init and cleans up the node on destruction
	ros::NodeHandle n;

	tf::TransformBroadcaster prcompensator_broadcaster;
	//init the vectornav IMU
	ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	ROS_INFO("Connected ...", connect);
	while(connect != 0){ //vn100 Functions return 0 if everything is ok
		ROS_ERROR("Not Connected ...");
		usleep(999999);
 	 //Connection failed in some way, abort
	}

	ROS_INFO("Server initialized...");

	ros::Publisher value_pub = imu.advertise<std_msgs::Float32MultiArray>("head", 1000);
	ros::Rate loop_rate(20);
	
	while (ros::ok())
  	{
	ros::Time imu_time = ros::Time::now();
	//vn100_getYawPitchRoll(&vn100, &ypr);
	//vn100_getQuaternionMagnetic( &vn100, &quaternion, &magnetic);
	vn100_getYawPitchRollMagneticAccelerationAngularRate( &vn100, &ypr, &magnetic, &acceleration, &angularRate);
	//vn100_getYawPitchRollTrueBodyAccelerationAngularRate(&vn100, &ypr, &acceleration, &angularRate);
	std_msgs::Float32MultiArray msg;

geometry_msgs::Quaternion prcompensator_quat=tf::createQuaternionMsgFromRollPitchYaw(ypr.roll*3.14159265359/180,ypr.pitch*3.14159265359/180,-0.05);

    geometry_msgs::TransformStamped prcompensator;
    prcompensator.header.stamp = imu_time;
    prcompensator.header.frame_id = "x_y";
    prcompensator.child_frame_id = "imu";

    prcompensator.transform.translation.x = 0.0;
    prcompensator.transform.translation.y = 0.0;
    prcompensator.transform.translation.z = 0.0;
    prcompensator.transform.rotation = prcompensator_quat;

	acc.header.frame_id = "imu";
	acc.header.stamp = ros::Time();
	acc.vector.x = acceleration.c0;
	acc.vector.y = acceleration.c1;
	acc.vector.z = acceleration.c2;

	prcompensator_broadcaster.sendTransform(prcompensator);

	float head =  (atan2(magnetic.c0,magnetic.c1))*180/3.14;
	float omega = angularRate.c2;
	
	geometry_msgs::Vector3Stamped acc;

    	msg.data.push_back(head);
    	msg.data.push_back(compensated_acc.vector.x);
    	msg.data.push_back(compensated_acc.vector.y);
    	msg.data.push_back(compensated_acc.vector.z);
    	msg.data.push_back(omega);
	value_pub.publish(msg);

	ros::spinOnce();
    	loop_rate.sleep();
	}

	ROS_INFO("Disconnecting...");
	usleep(999999);
	vn100_disconnect(&vn100);
	return 0;
}

