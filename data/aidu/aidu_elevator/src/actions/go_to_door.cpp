#include <ros/ros.h>
#include <aidu_elevator/actions/go_to_door.h>
#include <geometry_msgs/Twist.h>
#include <aidu_vision/DistanceSensors.h>

using namespace aidu::elevator;

GoToDoor::GoToDoor(ros::NodeHandle* nh) : Action::Action(nh) {
    //publisher
    position_pub = nh->advertise<geometry_msgs::Twist>("/pos",1,true);
    //subscriber
    sensor_sub = nh->subscribe("/sensors", 1, &GoToDoor::sensorcallback, this);
    this->door_open = false;
    distance=0.0;
}

GoToDoor::~GoToDoor() {
    
}

void GoToDoor::execute() {
  double depthOfDoor=0.375; // depth of elevator door in meters
  double buttonToDoor=0.95; // sideways distance from buttons to center of the door
  bool door_open=false;
  ROS_INFO("Executing Go to door action");
  sleep(1);
  
  geometry_msgs::Twist position;
  position.angular.z=0.0;
  position.linear.x=-0.4;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(3);
  position.angular.z=-1.57075;
  position.linear.x=0.0;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(3);
  position.angular.z=0.0;
  position.linear.x=buttonToDoor-0.15;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(4);
  position.angular.z=1.57075;
  position.linear.x=0.0;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(3);
  position.angular.z=0.0;
  position.linear.x=depthOfDoor+0.1;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(3);
  ros::Rate loopRate(10);
  distance = 0.0;
  while (distance<0.75 && ros::ok()){
        ros::spinOnce();
	loopRate.sleep();
  }
  this->door_open=true;
  ROS_INFO("door open");
}

bool GoToDoor::finished() {
  return this->door_open;
}

void GoToDoor::sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg){
  distance=(dist_msg->Frontleft+dist_msg->Frontright)/2000.0;
  //ROS_INFO("distance:%f",distance);
  //ROS_INFO("distance:%f",distance);
}