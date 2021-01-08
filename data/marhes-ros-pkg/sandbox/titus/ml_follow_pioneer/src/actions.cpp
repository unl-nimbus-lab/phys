#include "ml_follow_pioneer/actions.h"

Actions::Actions(ros::NodeHandle nh)
{
  n_ = nh;

  ros::NodeHandle n_private("~");
  n_private.param("num_ang_actions", num_ang_actions_, 3);
  n_private.param("num_lin_actions", num_lin_actions_, 3);
  n_private.param("ang_vel_lim", ang_vel_lim_, 1.0);
  n_private.param("lin_vel_lim", lin_vel_lim_, 0.5);

  vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  tmr_vel_ = n_.createTimer(ros::Duration(1 / 20), &Actions::timer_cb, this);
  
  ang_vels_ = std::vector<double>(num_ang_actions_);
  ang_inc_ = 2 * ang_vel_lim_ / (num_ang_actions_ - 1);
  for (int i = 0; i < num_ang_actions_; i++)
    ang_vels_[i] = -ang_vel_lim_ + i * ang_inc_;

  lin_vels_ = std::vector<double>(num_lin_actions_);
  lin_inc_ = lin_vel_lim_ / (num_lin_actions_ - 1);
  for (int i = 0; i < num_lin_actions_; i++)
    lin_vels_[i] = i * lin_inc_;
    
  vel_msg_.linear.x = 0.0;
  vel_msg_.angular.z = 0.0;
}

void Actions::Move(int lin_action, int ang_action)
{
  if (lin_action >= 0 && lin_action < num_lin_actions_ &&
      ang_action >= 0 && ang_action < num_ang_actions_)
  {
    vel_msg_.linear.x = lin_vels_[lin_action];
    vel_msg_.angular.z = ang_vels_[ang_action];
  }
}

// Actions are numbered linear in rows, angular in columns
// As follows: 1  2  3
//             4  5  6 ...
void Actions::Move(int action)
{
  if (action >= 0 && action < num_lin_actions_ * num_ang_actions_)
  {
    vel_msg_.linear.x = lin_vels_[action / num_ang_actions_];
    vel_msg_.angular.z = ang_vels_[action % num_ang_actions_];
  }
}

void Actions::Start(void)
{
  publish_ = true;
}

void Actions::Stop(void)
{
  publish_ = false;
}

int Actions::GetNumActionsLin(void)
{
  return num_lin_actions_;
}

int Actions::GetNumActionsAng(void)
{
  return num_ang_actions_;
}

void Actions::timer_cb(const ros::TimerEvent& event)
{
  if (publish_)
    vel_pub_.publish(vel_msg_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "actions_test");
  ros::NodeHandle n;
  int i = 0;

  Actions* r = new Actions(n);
  ros::Rate loop_rate(1);

  while(ros::ok())
  {
    r->Move(i);
    ROS_INFO("Chose Action %d", i);
    i++;
    if (i > r->GetNumActionsLin() * r->GetNumActionsAng())
      i = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

