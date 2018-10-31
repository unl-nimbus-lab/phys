#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

bool KeepGoing = true;
bool ShouldMeasureInitialPosition = true;
float InitialPosition = 0;
float xpos;

void subFunc(nav_msgs::Odometry eventMsg)
{
	
	//raw data
	float x_odom = eventMsg.pose.pose.position.x;
	
	//first time running program or robot
	if (ShouldMeasureInitialPosition == true){
		InitialPosition = x_odom;
		ShouldMeasureInitialPosition = false;
	}
	
	//xpos should start in zero
	xpos = x_odom - InitialPosition;
	
	std::cout << "\n x: " << xpos;
	
	if(xpos >= 1){
		KeepGoing = false;
		if(xpos < 1.11){
		std::cout << " stop variable updated .. ";
		}
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "wheelodom");
	ros::NodeHandle nodeName;
	ros::Rate loop_rate(10);
	
	ros::Publisher pubName = nodeName.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	
		ros::Subscriber subName = nodeName.subscribe("/odom", 100, subFunc);
		
	while (ros::ok())
  	{
  	
  		if(KeepGoing == true){
  		geometry_msgs::Twist twistMsg;
		twistMsg.linear.x=0.1;
		twistMsg.linear.y=0.0;
		twistMsg.linear.z=0.0;
		twistMsg.angular.x=0.0;
		twistMsg.angular.y=0.0;
		twistMsg.angular.z=0.0;
	  	
	  	pubName.publish(twistMsg);
	  	} else {
	  	std::cout << " Really Stopped!!!";
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

}
