#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

class OdomToPath
{
public:
  OdomToPath(ros::NodeHandle nh);
private:
  nav_msgs::Path path_;
  
  ros::NodeHandle n_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  
  void odomCB(nav_msgs::Odometry msg);
};

OdomToPath::OdomToPath(ros::NodeHandle nh)
{
  n_ = nh;

  odom_sub_ = n_.subscribe<nav_msgs::Odometry>("/odom", 10, &OdomToPath::odomCB, this);
  path_pub_ = n_.advertise<nav_msgs::Path>("/path", 10);
  
  path_.header.frame_id = "/odom";
  path_.header.stamp = ros::Time::now();
}

void OdomToPath::odomCB(nav_msgs::Odometry msg)
{
  geometry_msgs::PoseStamped pose;
  path_.header.stamp = ros::Time::now();
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/odom";
  pose.pose = msg.pose.pose;
  path_.poses.push_back(pose);
  path_pub_.publish(path_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "OdomToPath");
	ros::NodeHandle n;
	
	OdomToPath * op = new OdomToPath(n);
	
	ros::spin();
	
  return 0;	
}
