#include "ml_light_pioneer/actions.h"

Actions::Actions(ros::NodeHandle nh)
{
  n_ = nh;

  ros::NodeHandle n_private("~");
  n_private.param("num_actions", num_actions_, 3);
  n_private.param("ang_vel_lim", ang_vel_lim_, 1.0);
  n_private.param("lin_vel", lin_vel_, 0.3);

  vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  tmr_vel_ = n_.createTimer(ros::Duration(1 / 20), &Actions::timer_cb, this);
  
  ang_vels_ = std::vector<double>(num_actions_);
  ang_inc_ = 2 * ang_vel_lim_ / (num_actions_ - 1);
  for (int i = 0; i < num_actions_; i++)
    ang_vels_[i] = -ang_vel_lim_ + i * ang_inc_;
    
  vel_msg_.linear.x = 0.0;
  vel_msg_.angular.z = 0.0;
}

void Actions::Move(int action)
{
  if (action >= 0 && action < num_actions_)
  {
    vel_msg_.linear.x = lin_vel_;
    vel_msg_.angular.z = ang_vels_[action];
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

int Actions::GetNumActions(void)
{
	return num_actions_;
}

geometry_msgs::Twist Actions::GetVel(void)
{
  return vel_msg_;
}

void Actions::timer_cb(const ros::TimerEvent& event)
{
  if (publish_)
    vel_pub_.publish(vel_msg_);
}

/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "actions_test");
  ros::NodeHandle n;
  int i = 0;

  Actions* r = new Actions(n);
  ros::Rate loop_rate(1);

  while(ros::ok())
  {
    r->Move((Actions::moveType)i);
    i++;
    if (i > 5)
      i = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
*/
