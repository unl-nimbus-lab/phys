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
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

using namespace std;
#include <iostream>

const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;
float heading;		/*Floats for the returned values*/
float trans[3][3];

int main(int argc, char **argv)
{
	Vn100 vn100;
	VnYpr ypr;
	VnVector3 magnetic;
	VnVector3 acceleration;
	VnVector3 angularRate;
	VN_BOOL tared;

	int connect = 0;
	ros::init(argc, argv, "IMU_mag_Interface"); //init the driver 
	ros::NodeHandle imu; //create a handle for the node - this does the init and cleans up the node on destruction
	ros::NodeHandle n;

	//necessary for establishing connection with imu
	ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	ROS_INFO("Connected ...", connect);
	while(connect != 0){ //vn100 Functions return 0 if everything is ok
		ROS_ERROR("Not Connected ...");
		connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
		usleep(999999);
 	 //Connection failed in some way, abort
	}
	vn100_tare( &vn100, tared);
	ROS_INFO("Server initialized...");
	
	ros::Publisher imu_glo = n.advertise<sensor_msgs::MagneticField>("/imu_mag", 1);
	ros::Rate loop_rate(25);

 while (ros::ok())
 {
	ros::Time imu_time = ros::Time::now();
	
	vn100_getYawPitchRollMagneticAccelerationAngularRate( &vn100, &ypr, &magnetic, &acceleration, &angularRate);

 	sensor_msgs::MagneticField imu_mag;
 	
 	imu_mag.header.stamp = imu_time;
 	imu_mag.header.frame_id = "/imu";
 	
	imu_mag.magnetic_field.x = magnetic.c0;
	imu_mag.magnetic_field.y = magnetic.c1;
	imu_mag.magnetic_field.z = magnetic.c2;

	imu_glo.publish(imu_mag);
        ros::spinOnce();
    	loop_rate.sleep(); 
 }
}
	
