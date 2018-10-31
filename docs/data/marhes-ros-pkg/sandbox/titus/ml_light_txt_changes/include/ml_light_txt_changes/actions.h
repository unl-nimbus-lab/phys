#ifndef ACTIONS_H
#define ACTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

class Actions
{
public:
  Actions(ros::NodeHandle nh);
  void Move(int action);
  int GetNumActions(void);
  void Start(void);
  void Stop(void);
  geometry_msgs::Twist GetVel(void);
  int GetCurrentActionFromOdom(nav_msgs::Odometry odom_msg);
  
private:  
  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  ros::Timer tmr_vel_;
  geometry_msgs::Twist vel_msg_;
  int num_actions_;
  double ang_vel_lim_, ang_inc_, lin_vel_;
  std::vector<double> ang_vels_;
  bool publish_;
  
  void timer_cb(const ros::TimerEvent& event);
};

#endif
