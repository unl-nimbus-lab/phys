#ifndef ACTIONS_H
#define ACTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <vector>

class WaypointNavigator
{
public:
  WaypointNavigator(ros::NodeHandle nh);
  
private:
  ros::NodeHandle n_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;  
  
  std::vector<geometry_msgs::Point> waypoints_;
  double lin_vel_, look_ahead_;
  int next_waypoint_, last_waypoint_;
  
  void cb_odom(nav_msgs::Odometry msg);
  double distance(geometry_msgs::Point p1, geometry_msgs::Point p2);
};

#endif
