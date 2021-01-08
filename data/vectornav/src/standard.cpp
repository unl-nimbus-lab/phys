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
int call = 1;

double minimum(double a, double b)
{
	double c;
	if(a<b){c = a;}
	else{c = b;}

return c;
}

bool filter(double theta, double back_theta)
{
	double grad = 20*minimum(abs(back_theta - theta), 2*M_PI - (abs(back_theta - theta)))/call;

	if(grad > 2*M_PI){
		call++;
		ROS_INFO("jhasfdlkj %f", grad*180/M_PI);
		return true;
		}

	else return false;
}

int main(int argc, char **argv)
{
	Vn100 vn100;
	VnYpr ypr;
	VnQuaternion quaternion;
	VnVector3 magnetic;
	VnVector3 acceleration;
	VnVector3 angularRate;
	VnVector3 angularRateVariance;
	VnVector3 magneticVariance;
	VnVector3 accelerationVariance;
	double angularWalkVariance;
	VN_BOOL tared;

	int connect = 0;
	ros::init(argc, argv, "IMU_Interface"); //init the driver 
	ros::NodeHandle imu; //create a handle for the node - this does the init and cleans up the node on destruction
	ros::NodeHandle n;

	//necessary for establishing connection with imu
	ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	ROS_INFO("Connected ...", connect);
	while(connect != 0){ //vn100 Functions return 0 if everything is ok
		ROS_ERROR("Not Connected ...");
		usleep(999999);
 	 //Connection failed in some way, abort
	}
	//vn100_tare( &vn100, tared);
	ROS_INFO("Server initialized...");
	
	tf::TransformBroadcaster tfB;
	ros::Publisher imu_msg = imu.advertise<sensor_msgs::Imu>("/imu_data", 1);
	ros::Publisher imu_glo = n.advertise<sensor_msgs::MagneticField>("/imu_mag", 1);
	ros::Rate loop_rate(100);

 while (ros::ok())
 {
	ros::Time imu_time = ros::Time::now();
	
	//vn100_getQuaternionMagneticAccelerationAngularRate( &vn100, &quaternion, &magnetic, &acceleration, & angularRate);
	vn100_getYawPitchRollMagneticAccelerationAngularRate( &vn100, &ypr, &magnetic, &acceleration, &angularRate);
	ROS_INFO("%f", angularRate.c0);
	//vn100_getYawPitchRollTrueBodyAccelerationAngularRate( &vn100, &ypr, &acceleration, &angularRate);
	vn100_getFilterMeasurementVarianceParameters( &vn100, &angularWalkVariance, &angularRateVariance, &magneticVariance, &accelerationVariance);
	
    geometry_msgs::TransformStamped gravity_comp;
    gravity_comp.header.stamp = imu_time;
    gravity_comp.header.frame_id = "x_y";
    gravity_comp.child_frame_id = "imu";

    gravity_comp.transform.translation.x = 0.0;
    gravity_comp.transform.translation.y = 0.0;
    gravity_comp.transform.translation.z = 0.0;
    geometry_msgs::Quaternion qt = tf::createQuaternionMsgFromRollPitchYaw(ypr.roll*M_PI/180, ypr.pitch*M_PI/180, ypr.yaw*M_PI/180);
    gravity_comp.transform.rotation = qt;
    
    tfB.sendTransform(gravity_comp);
    
 	sensor_msgs::Imu imu_nav;
 	sensor_msgs::MagneticField imu_mag;
 	
 	imu_nav.header.stamp = imu_time;
 	imu_nav.header.frame_id = "/imu";
 	imu_mag.header.stamp = imu_time;
 	imu_mag.header.frame_id = "/imu";
 	
	imu_nav.linear_acceleration.x = acceleration.c0;
 	imu_nav.linear_acceleration.y = acceleration.c1;
 	imu_nav.linear_acceleration.z = acceleration.c2;
	imu_nav.linear_acceleration_covariance[0] = accelerationVariance.c0;
	imu_nav.linear_acceleration_covariance[4] = accelerationVariance.c1;
	imu_nav.linear_acceleration_covariance[8] = accelerationVariance.c2;
 	
 	imu_nav.angular_velocity.x = angularRate.c0;
 	imu_nav.angular_velocity.y = angularRate.c1;
 	imu_nav.angular_velocity.z = angularRate.c2;
	imu_nav.angular_velocity_covariance[0] = angularRateVariance.c0;
	imu_nav.angular_velocity_covariance[4] = angularRateVariance.c1;
	imu_nav.angular_velocity_covariance[8] = angularRateVariance.c2;

	double back_th = atan2(imu_mag.magnetic_field.y, imu_mag.magnetic_field.x);
	double th = atan2(magnetic.c1, magnetic.c0);

	double x = imu_mag.magnetic_field.x;
	double y = imu_mag.magnetic_field.y;
	imu_mag.magnetic_field.x = magnetic.c0;
	imu_mag.magnetic_field.y = magnetic.c1;
	imu_mag.magnetic_field.z = magnetic.c2;
	imu_mag.magnetic_field_covariance[0] = magneticVariance.c0;
	imu_mag.magnetic_field_covariance[4] = magneticVariance.c1;
	imu_mag.magnetic_field_covariance[8] = magneticVariance.c2;
	imu_mag.magnetic_field_covariance[1] = ypr.yaw;
	//imu_mag.magnetic_field_covariance[2] = ypr.roll;
	//imu_mag.magnetic_field_covariance[3] = ypr.pitch;
	
	/*if(minimum(abs(theta - back_theta), 2*M_PI - (abs(back_theta - theta))) > 20*M_PI/180)
	{
		imu_mag.magnetic_field.x = x;
		imu_mag.magnetic_field.y = y;
	}*/

	/*if(filter(th, back_th)){
	ROS_INFO("jhasfdlkj %f", (double)(minimum(abs(th - back_th), 2*M_PI - (abs(back_th - th)))*180/M_PI));
		imu_mag.magnetic_field.x = x;
		imu_mag.magnetic_field.y = y;
	}
	else{
		call = 1;
	}*/


	//ROS_INFO("x/y %f", (atan2(magnetic.c0,magnetic.c1))*180/3.14);
	//ROS_INFO("-y/x %f", (-1*atan2(magnetic.c1,magnetic.c0))*180/3.14);
	
	imu_msg.publish(imu_nav);
	imu_glo.publish(imu_mag);
        ros::spinOnce();
    	loop_rate.sleep(); 
 }
}
	
