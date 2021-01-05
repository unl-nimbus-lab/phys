#ifndef MOVE_SIMPLE_H_
#define MOVE_SIMPLE_H_

#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include <cmath>

class MoveSimple
{
public:
  MoveSimple(ros::NodeHandle nh);
	
private:
  ros::NodeHandle n_;
  ros::Subscriber move_cmd_sub_, odom_sub_;
  ros::Publisher vel_pub_, move_done_pub_;
  geometry_msgs::Pose goal_;
  int state_;
  double max_lin_vel_, max_ang_vel_;
  enum {MOVE_NONE, MOVE_HEADING, MOVE_POSITION, MOVE_ORIENTATION};

  void cb_cmd(geometry_msgs::Pose msg);
  void cb_odom(nav_msgs::Odometry msg);
};

#endif /* MOVE_SIMPLE_H_ */
