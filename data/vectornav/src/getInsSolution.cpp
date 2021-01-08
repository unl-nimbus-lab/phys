#include "vectornav/vectornav.h"
#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include "geometry_msgs/Vector3Stamped.h"

const int SERIAL_BAUD_RATE = 115200;
const char* COM_PORT = "/dev/ttyUSB0";

using namespace std;

int main(int argc, char* argv[]){

	///Variable Declerations
	Vn200 vn200;
	Vn200CompositeData composedData;
	double gpsTime;
	unsigned short gpsWeek, status;
	VnVector3 ypr, latitudeLongitudeAltitude, nedVelocity;
	float attitudeUncertainty, positionUncertainty, velocityUncertainty;


	
	///ROS Node Initializations
	ros::init(argc, argv, "IMU_Data_Extractor");
	ros::NodeHandle n;
	ros::Publisher LLA_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/LLA", 1);
	ros::Publisher ypr_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/ypr", 1);
	ros::Publisher ned_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/ned", 1);
	ros::Rate loop_rate(20);


	
	///Connection to Vectornav
	int connect;
	ROS_INFO("Attempting Connection to IMU");
	connect = vn200_connect(&vn200, COM_PORT, SERIAL_BAUD_RATE);
	while(connect!=0){
		ROS_INFO("Error encountered:  %d\n Attempting again.", connect);
		usleep(1000);
		connect = vn200_connect(&vn200, COM_PORT, SERIAL_BAUD_RATE);
	}
	if(vn200.isConnected) ROS_INFO("Successfully connected!\n\n");
	

	
	
	///Debugging Data Output Type
	unsigned int type;
//	vn200_setAsynchronousDataOutputType(&vn200, 22, 1);
	vn200_getAsynchronousDataOutputType(&vn200, &type);
	ROS_INFO("Asynchronous Data Output Type Set to: %d\n\n", type);
	sleep(1);
	
	
	
	///Publishing Loop
	int count=0;
	ros::Time current_time; 
	geometry_msgs::Vector3Stamped ypr_, LLA_, ned_;
	
	ypr_.header.frame_id="ypr"; LLA_.header.frame_id="LLA"; ned_.header.frame_id="ned";
	
	while(ros::ok()){
		
		/*Getting Data*/
		if(vn200.isConnected){
			vn200_getInsSolution(&vn200, &gpsTime, &gpsWeek, &status, &ypr, &latitudeLongitudeAltitude, &nedVelocity, &attitudeUncertainty, &positionUncertainty, &velocityUncertainty);
			current_time=ros::Time::now();
			ROS_INFO("Yaw: %f", ypr.c0); 
			ROS_INFO("Latitude: %f", latitudeLongitudeAltitude.c0);
			ROS_INFO("North Velocity: %f\n", nedVelocity.c0);
		}
		else ROS_INFO("Connection Error\n----------------------------\n");
				
		/*Filling in Data*/
			//Header Info//
			ypr_.header.stamp=current_time; LLA_.header.stamp=current_time; ned_.header.stamp=current_time;
			ypr_.header.seq=count; LLA_.header.seq=count; ned_.header.seq=count;
			//Data Values//
			ypr_.vector.x=ypr.c0; ypr_.vector.y=ypr.c1; ypr_.vector.z=ypr.c2;
			LLA_.vector.x=latitudeLongitudeAltitude.c0; LLA_.vector.y=latitudeLongitudeAltitude.c1; LLA_.vector.z=latitudeLongitudeAltitude.c2;
			ned_.vector.x=nedVelocity.c0; ned_.vector.y=nedVelocity.c1; ned_.vector.z=nedVelocity.c2;
		
		/*Publishing Data*/
		ypr_publisher.publish(ypr_);
		LLA_publisher.publish(LLA_);
		ned_publisher.publish(ned_);
		
		/*Other Stuff*/
		count++;
		loop_rate.sleep();
	}	

	return 0;
}
