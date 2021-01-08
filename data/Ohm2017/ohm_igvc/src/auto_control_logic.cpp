#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <boost/chrono.hpp>
#include <string>

//https://stackoverflow.com/a/4974588 but use boost
typedef boost::chrono::high_resolution_clock Clock;
typedef boost::chrono::milliseconds milliseconds;

ros::Publisher autoPub;
Clock::time_point lastOverride = Clock::now();
double overrideDuration;
bool enableLogging;

void pidCallback(const geometry_msgs::Twist::ConstPtr& msg){
    if(boost::chrono::duration_cast<milliseconds>(Clock::now() - lastOverride).count() > overrideDuration){
        autoPub.publish(*msg);

        if(enableLogging) ROS_INFO("Auto Control: pid linear.x=%f angular.z=%f", msg->linear.x, msg->angular.z);
    }
}

void overrideCallback(const geometry_msgs::Twist::ConstPtr& msg){
    lastOverride = Clock::now();
	autoPub.publish(*msg);

	if(enableLogging) ROS_INFO("Auto Control: override linear.x=%f angular.z=%f", msg->linear.x, msg->angular.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "auto_control_logic");

	ros::NodeHandle n;

	n.param("auto_control_logic_enable_logging", enableLogging, false);
    n.param("obstacleOverrideDurationMillis", overrideDuration, 1000.0);

	autoPub = n.advertise<geometry_msgs::Twist>("autoControl", 5);

	ros::Subscriber pidSub = n.subscribe("ohmPid", 5, pidCallback);
    ros::Subscriber overrideSub = n.subscribe("avoidanceOverride", 5, overrideCallback);

	ros::spin();
	
	return 0;
}