#ifndef STATES_H
#define STATES_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include <cmath>

class States
{
public:
  States(ros::NodeHandle nh);
  int GetState(void);
  int GetReward(void);
  double GetDistance(void);
  int GetNumDistStates(void);
  int GetNumAngStates(void);
  int GetNumStates(void);

  enum {FLLS, FRLS, RLLS, RRLS};
  
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_odom_1_, sub_odom_2_;
  ros::Publisher reward_pub_, vis_pub_;
  ros::Timer tmr_state_;
  nav_msgs::Odometry odom_msg_1_, odom_msg_2_;
  visualization_msgs::Marker marker_, reward_marker_;
  
  int state_, state_dist_, state_ang_, reward_, num_dist_states_, num_ang_states_;
  double set_dist_, set_ang_, thresh_dist_, thresh_ang_, dist_, phi_;

  void cb_tmr_state(const ros::TimerEvent& event);
  void cb_odom_1(nav_msgs::Odometry msg);
  void cb_odom_2(nav_msgs::Odometry msg);
};

#endif
