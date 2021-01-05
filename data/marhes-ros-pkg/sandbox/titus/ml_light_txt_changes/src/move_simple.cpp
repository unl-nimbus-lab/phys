#include "ml_light_txt_changes/move_simple.h"

MoveSimple::MoveSimple(ros::NodeHandle nh):n_(nh)
{
  ros::NodeHandle n_private("~");
  n_private.param("max_lin_vel", max_lin_vel_, 0.5);
  n_private.param("max_ang_vel", max_ang_vel_, 1.0);

  move_cmd_sub_ = n_.subscribe("move_cmd", 1, &MoveSimple::cb_cmd, this);
  odom_sub_ = n_.subscribe("base_pose_ground_truth", 1, &MoveSimple::cb_odom, this);
  vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  move_done_pub_ = n_.advertise<std_msgs::Bool>("move_done", 1);

  state_ = MOVE_NONE;
}

void MoveSimple::cb_cmd(geometry_msgs::Pose msg)
{
  goal_ = msg;
  state_ = MOVE_HEADING;
}

void MoveSimple::cb_odom(nav_msgs::Odometry msg)
{
  double dist_err, head_err, xdist, ydist;
  geometry_msgs::Twist vel_msg;
  std_msgs::Bool bool_msg;
  
  switch(state_)
  {
  case MOVE_NONE:
      // Publish move_stop msg
      bool_msg.data = false;
      move_done_pub_.publish(bool_msg);
    break;
  case MOVE_HEADING:
      xdist = goal_.position.x - msg.pose.pose.position.x;
      ydist = goal_.position.y - msg.pose.pose.position.y;
      head_err = atan2(ydist, xdist) - tf::getYaw(msg.pose.pose.orientation);
      while (head_err > M_PI)
        head_err -= 2 * M_PI;
      while (head_err < -M_PI)
        head_err += 2 * M_PI;
        
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = head_err;
      if (vel_msg.angular.z > max_ang_vel_)
        vel_msg.angular.z = max_ang_vel_;
      if (vel_msg.angular.z < -max_ang_vel_)
        vel_msg.angular.z = -max_ang_vel_;
      
      if (std::abs(head_err) < 0.05)
      {
        state_ = MOVE_POSITION;
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
      }
      vel_pub_.publish(vel_msg);
            
      // Publish move_stop msg
      bool_msg.data = false;
      move_done_pub_.publish(bool_msg);
    break;
  case MOVE_POSITION:
      xdist = goal_.position.x - msg.pose.pose.position.x;
      ydist = goal_.position.y - msg.pose.pose.position.y;
      dist_err = sqrt(pow(xdist,2) + pow(ydist,2));
      head_err = atan2(ydist, xdist) - tf::getYaw(msg.pose.pose.orientation);
      while (head_err > M_PI)
        head_err -= 2 * M_PI;
      while (head_err < -M_PI)
        head_err += 2 * M_PI;
      //ROS_INFO("Goal: %f, %f, %f", goal_.position.x, goal_.position.y, tf::getYaw(goal_.orientation));
      //ROS_INFO("Dist Error: %f, Head Error: %f", dist_err, head_err);
      
      vel_msg.linear.x = max_lin_vel_;
      vel_msg.angular.z = head_err;
      if (vel_msg.angular.z > max_ang_vel_)
        vel_msg.angular.z = max_ang_vel_;
      if (vel_msg.angular.z < -max_ang_vel_)
        vel_msg.angular.z = -max_ang_vel_;
      
      if (dist_err < 0.1)
      {
        state_ = MOVE_ORIENTATION;
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
      }
      vel_pub_.publish(vel_msg);
              
      // Publish move_stop msg
      bool_msg.data = false;
      move_done_pub_.publish(bool_msg);
    break;
  case MOVE_ORIENTATION:
      head_err = tf::getYaw(goal_.orientation) - tf::getYaw(msg.pose.pose.orientation);
      while (head_err > M_PI)
        head_err -= 2 * M_PI;
      while (head_err < -M_PI)
        head_err += 2 * M_PI;
        
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = head_err;
      if (vel_msg.angular.z > max_ang_vel_)
        vel_msg.angular.z = max_ang_vel_;
      if (vel_msg.angular.z < -max_ang_vel_)
        vel_msg.angular.z = -max_ang_vel_;

      if (std::abs(head_err) < 0.05)
      {
        state_ = MOVE_NONE;
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        
        // Publish move_stop msg
        bool_msg.data = true;
      }
      else
      {
        // Publish move_stop msg
        bool_msg.data = false;
      }
      vel_pub_.publish(vel_msg);
      move_done_pub_.publish(bool_msg);
    break;
  default:
    break;
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_simple");
	ros::NodeHandle n;

  MoveSimple* s = new MoveSimple(n);
  ros::spin();

  return 0;
}
