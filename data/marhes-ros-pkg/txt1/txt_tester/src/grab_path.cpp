#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "std_srvs/Empty.h"

class GrabPath
{
private:
  ros::NodeHandle n_;
  ros::Subscriber path_sub_;
  ros::Publisher path_pub_;
  ros::ServiceServer grab_path_;
    
  nav_msgs::Path last_path_;
  
  void pathCB(nav_msgs::Path msg);  
  bool grabCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
public:
  GrabPath(ros::NodeHandle n);
};

GrabPath::GrabPath(ros::NodeHandle n)
{
  n_ = n;
  path_sub_ = n_.subscribe<nav_msgs::Path>("path", 1, &GrabPath::pathCB, this);
  path_pub_ = n_.advertise<nav_msgs::Path>("path_grab", 1);
  grab_path_ = n_.advertiseService("grab", &GrabPath::grabCB, this);
}

void GrabPath::pathCB(nav_msgs::Path msg)
{
  last_path_ = msg;
}

bool GrabPath::grabCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  path_pub_.publish(last_path_);
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "GrabPath");
	ros::NodeHandle n;
	
	GrabPath * gp = new GrabPath(n);
	
	ros::spin();
	
  return 0;	
}

