#include <ros/ros.h>
#include <aidu_elevator/actions/reposition.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <aidu_vision/DistanceSensors.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace aidu::elevator;

Reposition::Reposition(ros::NodeHandle* nh) : Action::Action(nh) {
    // Publishers
    positionPublisher = nh->advertise<geometry_msgs::Twist>("/pos",1,true);
    armPublisher = nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions",1);
    // Subscribers
    distanceSubscriber = nh->subscribe("/sensors", 1, &Reposition::distanceCallback, this);
    armSubscriber = nh->subscribe("/arm_state", 1, &Reposition::armCallback, this);
    done = false;
}

Reposition::~Reposition() {
    
}

void Reposition::execute() {
  ROS_INFO("Executing reposition action");
  double dist = distance;
  sleep(1);
  moveArm(translation, 1.2, 0.0);
  sleep(2);
  this->moveBase(0.0, 1.57/2.0);
  sleep(2);
  this->moveBase(-0.2, 0.0);
  sleep(2);
  this->moveBase(0.0, -1.57/2.0);
  sleep(2);
  this->moveBase(0.15,0.0);
  
  double x = cos( (1.57 / 2.0) ) * 0.2 + 0.15;
  double y = dist + sin( (1.57 / 2.0) ) * 0.2;
  double angle = -atan(y / x);
  ROS_INFO("Moving arm to angle: %.4f, x: %.4f, y: %.4f", angle, x, y);
  moveArm(translation-0.04, angle, 0.0);
  sleep(2);
  
  ROS_INFO("Reposition complete");
  this->done = true;
}

void Reposition::distanceCallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg){
    distance = dist_msg->arm/1000.0;
}

void Reposition::armCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){
    translation = joint_msg->position[0];
}

void Reposition::moveBase(double linear, double angular) {
    geometry_msgs::Twist position;
    position.angular.z = angular;
    position.linear.x = linear;
    positionPublisher.publish(position);
    ros::spinOnce();
}

void Reposition::moveArm(double translation, double rotation, double extention) {
  aidu_robotarm::robot_arm_positions arm_position;
  arm_position.translation = translation;
  arm_position.rotation = rotation;
  arm_position.extention = extention;
  armPublisher.publish(arm_position);
  ros::spinOnce();
  
}

bool Reposition::finished() {
  return this->done;
}
