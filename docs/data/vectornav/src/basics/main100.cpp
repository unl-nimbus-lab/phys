#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <stdio.h>
#include <unistd.h>
#include "vectornav/vectornav.h"


using namespace std;
#include <iostream>

#define LOOP_RATE 5000

const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 57600;
float heading, pitch, roll;		/*Floats for the returned values*/

int main(int argc, char **argv)
{
	Vn100 vn100;
	VnYpr ypr;
	int connect = 0;
	ros::init(argc, argv, "IMU_Interface"); //init the driver 
	ros::NodeHandle imu; //create a handle for the node - this does the init and cleans up the node on destruction

	//init the vectornav IMU
	ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	ROS_INFO("Connected ...");
	while(connect != 0){ //vn100 Functions return 0 if everything is ok
		ROS_ERROR("Not Connected");
		usleep(999999);
 	 //Connection failed in some way, abort
	}

	
	ROS_INFO("Server initialized...");
	 int i;
	for (i = 0; i < 10; i++) {
		vn100_getYawPitchRoll(&vn100, &ypr);
		printf("YPR: %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
	}
	

	ROS_INFO("Shutting Down....");
	usleep(999999);
	vn100_disconnect(&vn100);
	return 0;
}



