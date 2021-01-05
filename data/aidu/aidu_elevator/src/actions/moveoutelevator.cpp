#include <ros/ros.h>
#include <aidu_elevator/actions/moveoutelevator.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <aidu_vision/DistanceSensors.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace aidu::elevator;

MoveOutElevator::MoveOutElevator(ros::NodeHandle* nh) : Action::Action(nh) {
    // Publishers
    positionPublisher = nh->advertise<geometry_msgs::Twist>("/pos",1,true);
    armPublisher = nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions",1);
    // Subscribers
    distanceSubscriber = nh->subscribe("/sensors", 1, &MoveOutElevator::distanceCallback, this);
    done = false;
}

MoveOutElevator::~MoveOutElevator() {
    
}

void MoveOutElevator::execute() {
  ROS_INFO("Executing move out elevator action");
  
  // Wait for doors
  ros::Rate loop(5);
  while(distance < 0.6 && ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }
  
  // Move outside
  sleep(1);
  this->moveBase(1.5, 0.0);
  sleep(5);
  
  aidu_robotarm::robot_arm_positions arm_position;
  arm_position.translation = 0.0;
  arm_position.rotation = 1.57;
  arm_position.extention = 0.0;
  armPublisher.publish(arm_position);
  ros::spinOnce();
  sleep(3);
  
  ROS_INFO("MoveOutElevator complete");
  this->done = true;
}

void MoveOutElevator::distanceCallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg){
    distance = (dist_msg->Frontleft+dist_msg->Frontright)/2000.0;
}

void MoveOutElevator::moveBase(double linear, double angular) {
    geometry_msgs::Twist position;
    position.angular.z = angular;
    position.linear.x = linear;
    positionPublisher.publish(position);
    ros::spinOnce();
}

bool MoveOutElevator::finished() {
  return this->done;
}
