#include "ml_follow_pioneer/states_stage.h"
               
States::States(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  geometry_msgs::Point p;

  n_private.param("num_dist_states", num_dist_states_, 3);
  n_private.param("num_ang_states", num_ang_states_, 3);
  n_private.param("set_distance", set_dist_, 1.5);
  n_private.param("set_angle", set_ang_, 0.0);
  n_private.param("threshold_distance", thresh_dist_, 0.5);
  n_private.param("threshold_angle", thresh_ang_, M_PI / 6);

  sub_odom_1_ = n_.subscribe("/robot_0/base_pose_ground_truth", 1, &States::cb_odom_1, this);
  sub_odom_2_ = n_.subscribe("/robot_1/base_pose_ground_truth", 1, &States::cb_odom_2, this);
  tmr_state_ = n_.createTimer(ros::Duration(0.1), &States::cb_tmr_state, this);
  vis_pub_ = n_.advertise<visualization_msgs::Marker>("light_marker", 1);
  reward_pub_ = n_.advertise<visualization_msgs::Marker>("reward_marker", 1);

  // Make reward visualization
  reward_marker_.header.frame_id = "robot_0/base_footprint";
  reward_marker_.header.stamp = ros::Time::now();
  reward_marker_.ns = "reward_markers";
  reward_marker_.id = 0;
  reward_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  reward_marker_.action = visualization_msgs::Marker::ADD;
  reward_marker_.pose.position.x = 0;
  reward_marker_.pose.position.y = 0;
  reward_marker_.pose.position.z = 0;
  reward_marker_.pose.orientation.x = 0.0;
  reward_marker_.pose.orientation.y = 0.0;
  reward_marker_.pose.orientation.z = 0.0;
  reward_marker_.pose.orientation.w = 1.0;
  reward_marker_.scale.x = 0.01;
  reward_marker_.scale.y = 0.01;
  reward_marker_.scale.z = 0.01;
  reward_marker_.color.r = 0.0f;
  reward_marker_.color.g = 1.0f;
  reward_marker_.color.b = 0.0f;
  reward_marker_.color.a = 1.0f;
  p.x = -(set_dist_ - thresh_dist_);
  p.y = p.x * tan(set_ang_ + thresh_ang_);
  reward_marker_.points.push_back(p);
  p.x = -(set_dist_ - thresh_dist_);
  p.y = p.x * tan(set_ang_ - thresh_ang_);
  reward_marker_.points.push_back(p);
  p.x = -(set_dist_ + thresh_dist_);
  p.y = p.x * tan(set_ang_ - thresh_ang_);
  reward_marker_.points.push_back(p);
  p.x = -(set_dist_ + thresh_dist_);
  p.y = p.x * tan(set_ang_ + thresh_ang_);
  reward_marker_.points.push_back(p);
  p.x = -(set_dist_ - thresh_dist_);
  p.y = p.x * tan(set_ang_ + thresh_ang_);
  reward_marker_.points.push_back(p);
  reward_pub_.publish( reward_marker_ );

  reward_ = 0;
  state_ = 0;
}

int States::GetState(void)
{
  return state_; 
}

int States::GetReward(void)
{
	return reward_;
}

int States::GetNumDistStates(void)
{
	return num_dist_states_;
}

int States::GetNumAngStates(void)
{
	return num_ang_states_;
}

int States::GetNumStates(void)
{
  return num_dist_states_ * num_ang_states_;
}

void States::cb_tmr_state(const ros::TimerEvent& event)
{
  // Calculate the bearing and the distance between the robots
  double x1, y1, x2, y2, theta1, theta2;
  int param_dist, param_ang, state_weight;
  
  x1 = odom_msg_1_.pose.pose.position.x;
  y1 = odom_msg_1_.pose.pose.position.y;
  theta1 = tf::getYaw(odom_msg_1_.pose.pose.orientation);
  while (theta1 > M_PI)
    theta1 -= 2 * M_PI;
  while (theta1 < -M_PI)
    theta1 += 2 * M_PI;
  x2 = odom_msg_2_.pose.pose.position.x;
  y2 = odom_msg_2_.pose.pose.position.y;
  theta2 = tf::getYaw(odom_msg_2_.pose.pose.orientation);
  while (theta2 > M_PI)
    theta2 -= 2 * M_PI;
  while (theta2 < -M_PI)
    theta2 += 2 * M_PI;
    
  dist_ = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
  phi_ = theta1 - atan2((y2 - y1), (x2 - x1)) + M_PI;
  while (phi_ > M_PI)
    phi_ -= 2 * M_PI;
  while (phi_ < -M_PI)
    phi_ += 2 * M_PI;
  
  ROS_INFO("Dist: %f, Phi: %f", dist_, phi_);
  
  if ((dist_ > (set_dist_ + thresh_dist_)) || (dist_ < (set_dist_ - thresh_dist_)))
    param_dist = 1;
  else
    param_dist = 0;
  if ((phi_ > (set_ang_ + thresh_ang_)) || (phi_ < (set_ang_ - thresh_ang_)))
    param_ang = 1;
  else
    param_ang = 0;

  state_weight = param_dist + param_ang;
  if (state_weight == 0)
    reward_ = 2;
  else if (state_weight == 1)
    reward_ = 1;
  else
    reward_ = -1;
    
  ROS_INFO("Reward: %d", reward_);
  
  // Determine the state
  if (dist_ <= set_dist_ - thresh_dist_)
    state_dist_ = 0;
  else if ((dist_ > set_dist_ - thresh_dist_) && (dist_ < set_dist_ + thresh_dist_))
    state_dist_ = 1;
  else 
    state_dist_ = 2;
    
  if (phi_ <= set_ang_ - thresh_ang_)
    state_ang_ = 0;
  else if ((phi_ > set_ang_ - thresh_ang_) && (phi_ < set_ang_ + thresh_ang_))
    state_ang_ = 1;
  else 
    state_ang_ = 2;
    
  state_ = state_dist_ * num_ang_states_ + state_ang_;
    
  ROS_INFO("State: %d", state_);

  // update following area marker
  reward_pub_.publish(reward_marker_);
}
  
void States::cb_odom_1(nav_msgs::Odometry msg)
{
  odom_msg_1_ = msg;
}

void States::cb_odom_2(nav_msgs::Odometry msg)
{
  odom_msg_2_ = msg;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "states_test");
	ros::NodeHandle n;
  double reward;

  States* s = new States(n);
  //ros::spin();
  
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
//    reward = s->GetReward();
//    ROS_INFO("%f", reward);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

