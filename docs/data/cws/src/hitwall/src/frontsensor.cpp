#include <iostream>
#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h" 
#include "geometry_msgs/Twist.h"

bool KeepGoing = true;

void subFunc(kobuki_msgs::BumperEvent eventMsg)
{
	//center bumper = 1
	//pressed = 1
	
	std::cout << "\n event: \n";
	if(eventMsg.bumper == 1 && eventMsg.state ==0){
		std::cout << "center bumper released \n";
	}
	
	if(eventMsg.bumper == 1 && eventMsg.state ==1){
		KeepGoing = false;
		std::cout << "center bumper pressed \n";
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "frontsensor");
	ros::NodeHandle nodeName;
	ros::Rate loop_rate(10);
	
	ros::Publisher pubName = nodeName.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	
		ros::Subscriber subName = nodeName.subscribe("/mobile_base/events/bumper", 100, subFunc);
		
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
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

}
