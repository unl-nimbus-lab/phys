#ifndef ACTIONS_H
#define ACTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

class Actions
{
public:
  Actions(ros::NodeHandle nh);
  void Move(int lin_action, int ang_action);
  void Move(int action);
  int GetNumActionsLin(void);
  int GetNumActionsAng(void);
  void Start(void);
  void Stop(void);
  
private:  
  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  ros::Timer tmr_vel_;
  geometry_msgs::Twist vel_msg_;
  int num_lin_actions_, num_ang_actions_;
  double lin_vel_lim_, ang_vel_lim_, lin_inc_, ang_inc_;
  std::vector<double> lin_vels_, ang_vels_;
  bool publish_;
  
  void timer_cb(const ros::TimerEvent& event);
};

#endif
