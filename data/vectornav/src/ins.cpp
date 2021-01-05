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
#include "sensor_msgs/NavSatFix.h"

using namespace std;
#include <iostream>

const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;
sensor_msgs::NavSatFix ins_data;
float positionUncertainty;
ros::Publisher ins;
int gps;

float principal(float xyz)
{
	if (xyz <= -180.0)
	{
		xyz += 360.0;
	} 
	if (xyz > 180.0)
	{
		xyz -= 360.0;
	} 
	return xyz;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& fix)
{
	gps = fix->status.status;
	ins_data.status.status = gps + 1;
	if(gps != -1)
	{
		ROS_INFO("kalman filter");
		double gx = fix->latitude; 
		double gy = fix->longitude;
		double sgx = fix->position_covariance[0];
		double sgy = fix->position_covariance[4];
		
		ins_data.latitude = (double)((sgx*ins_data.latitude + positionUncertainty*gx)/(sgx + positionUncertainty));
		ins_data.longitude = (double)((sgy*ins_data.longitude + positionUncertainty*gy)/(sgy + positionUncertainty));

		ins_data.position_covariance[0] = (double)(positionUncertainty*sgx/(positionUncertainty + sgx));
		ins_data.position_covariance[4] = (double)(positionUncertainty*sgy/(positionUncertainty + sgy)); 

		ins.publish(ins_data);              
	}
	else
		ins.publish(ins_data);
}

int main(int argc, char **argv)
{
	Vn200 vn200;
	double gpsTime;
	unsigned short gpsWeek, status;
	VnVector3 ypr, latitudeLongitudeAltitude, nedVelocity;
	float attitudeUncertainty, velocityUncertainty;
	VN_BOOL tared;

	int connect = 0;
	ros::init(argc, argv, "IMU_Interface"); //init the driver 
	ros::NodeHandle imu; //create a handle for the node - this does the init and cleans up the node on destruction
	ros::NodeHandle n;

	//necessary for establishing connection with imu
	ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	ROS_INFO("Initializing...");

	connect = vn200_connect(&vn200, COM_PORT, BAUD_RATE);

	ROS_INFO("Connected ...", connect);

	while(connect != 0){ //vn100 Functions return 0 if everything is ok
		ROS_ERROR("Not Connected ...");
		usleep(999999);
 	 //Connection failed in some way, abort
	}

	ROS_INFO("Server initialized...");

	//vn200_tare( &vn200, tared);
	
	tf::TransformBroadcaster tfB;
	ins = imu.advertise<sensor_msgs::NavSatFix>("/fix_ins", 1);
	ros::Publisher imu_glo = n.advertise<sensor_msgs::MagneticField>("/imu_mag", 1);
	ros::Publisher imu_ins = n.advertise<sensor_msgs::NavSatFix>("/ins_data", 1);
	ros::Subscriber sub3 = n.subscribe("/fix", 1, gpsCallback);
	ros::Rate loop_rate(100);

 while (ros::ok())
 {
	ros::Time imu_time = ros::Time::now();
	
	vn200_getInsSolution(&vn200, &gpsTime, &gpsWeek, &status, &ypr, &latitudeLongitudeAltitude, &nedVelocity, &attitudeUncertainty,	&positionUncertainty, &velocityUncertainty);
	ROS_INFO("%f", ypr.c2);
    geometry_msgs::TransformStamped gravity_comp;
    gravity_comp.header.stamp = imu_time;
    gravity_comp.header.frame_id = "x_y";
    gravity_comp.child_frame_id = "imu";

    gravity_comp.transform.translation.x = 0.0;
    gravity_comp.transform.translation.y = 0.0;
    gravity_comp.transform.translation.z = 0.0;
    geometry_msgs::Quaternion qt = tf::createQuaternionMsgFromRollPitchYaw(ypr.c2*M_PI/180, ypr.c1*M_PI/180, ypr.c0*M_PI/180);
    gravity_comp.transform.rotation = qt;
    
    tfB.sendTransform(gravity_comp);
    
 	sensor_msgs::MagneticField imu_mag;

	ins_data.status.status = status;
	ins_data.latitude = latitudeLongitudeAltitude.c0;
	ins_data.longitude = latitudeLongitudeAltitude.c1;
	ins_data.altitude = latitudeLongitudeAltitude.c2;

	ins_data.position_covariance[0] = positionUncertainty;
	ins_data.position_covariance[4] = positionUncertainty;
	ins_data.position_covariance[8] = positionUncertainty;

	imu_mag.magnetic_field.x = 1;
	imu_mag.magnetic_field.y = 1;
	imu_mag.magnetic_field.z = 1;
	imu_mag.magnetic_field_covariance[0] = 1;
	imu_mag.magnetic_field_covariance[1] = principal(ypr.c0 + 44.5);
	imu_mag.magnetic_field_covariance[4] = 1;
	imu_mag.magnetic_field_covariance[8] = 1;

	imu_glo.publish(imu_mag);
	imu_ins.publish(ins_data);
  
	ros::spinOnce();
   	loop_rate.sleep(); 
 }
}
	
