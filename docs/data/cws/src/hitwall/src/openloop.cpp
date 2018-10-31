#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "openloop");
	ros::NodeHandle nodeName;
	ros::Rate loop_rate(100);
	
	ros::Publisher pubName = nodeName.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	
	int count=0;

	while (ros::ok())
  	{
  		count++;
  		
  		//V=0.1m/s
  		//d=1m
  		//t=d/V=10s
  		//Ts=1/100
  		//t=k*Ts
  		//k=t/Ts
  		//k=10*100
  		
  		if (count < 1000){
  		geometry_msgs::Twist twistMsg;
		twistMsg.linear.x=-0.1;
		twistMsg.linear.y=0.0;
		twistMsg.linear.z=0.0;
		twistMsg.angular.x=0.0;
		twistMsg.angular.y=0.0;
		twistMsg.angular.z=0.0;
	  	
	  	pubName.publish(twistMsg);
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

}
