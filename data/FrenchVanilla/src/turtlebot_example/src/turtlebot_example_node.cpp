//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos. 2012 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>


double X=0.0;

double Y=0.0;
double yaw_degrees=0.0;
int state=0;
double err_x=0.0,err_y=0.0,err_d=0.0,err_yaw=0.0;
double vel_x=0.0,vel_y=0.0,ang_z=0.0;

void goal_d(double x_t,double y_t, double t)
{
	err_x=x_t-X;
	err_y=y_t-Y;
	err_d=sqrt(err_x*err_x+err_y+err_y);
	err_yaw=t-yaw_degrees;
	//if(err_yaw<0)err_yaw=err_yaw+360;
	
	if(err_x>0.1||err_y>0.1)
	{vel_x=0.3;ang_z=0.0;}//ROS_INFO("1..S=%d",state);}
	else if(err_yaw>2)
	{ang_z=0.15;vel_x=0.0;}//ROS_INFO("2..S=%d",state);}
	else
	{
		state=(state+1)%8;
		ang_z=0.0;vel_x=0.0;
		ROS_INFO("3..S=%d",state);
	}
	
}
	
//Callback function for the Position topic 
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	//This function is called when a new pose message is received

	X = msg.pose.pose.position.x; // Robot X psotition
	Y = msg.pose.pose.position.y; // Robot Y psotition
	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	yaw_degrees = Yaw * 180.0 / M_PI; // conversion to degrees
	if( yaw_degrees < 0 ) yaw_degrees += 360.0; // convert negative to positive angles
	ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", X, Y, yaw_degrees);
	
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"main_control");
	ros::NodeHandle n;

	//Subscribe to the desired topics and assign callbacks
	ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

	//Setup topics to Publish from this node
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    
	//Velocity control variable
	geometry_msgs::Twist vel;

	//Set the loop rate
	ros::Rate loop_rate(20);    //20Hz update rate

	
	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages
    
		//Main loop code goes here:
		switch(state){
		case 0:goal_d(1,0,0);break;
		case 1:goal_d(1,0,90);break;
		case 2:goal_d(1,1,90);break;
		case 3:goal_d(1,1,180);break;
		case 4:goal_d(0,1,180);break;
		case 5:goal_d(0,1,270);break;
		case 6:goal_d(0,0,270);break;
		case 7:goal_d(0,0,358);break;
		default: ROS_INFO("DEfaultt");}
		vel.linear.x = vel_x; // set linear speed
		vel.angular.z = ang_z; // set angular speed

		
		velocity_publisher.publish(vel); // Publish the command velocity
		ROS_DEBUG("Main - Velocity commands: v - %f, w - %f", vel.linear.x, vel.angular.z);
 
	}

	return 0;
}
