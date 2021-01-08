#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>

int i = 1;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  for (unsigned i = 0; i < msg->axes.size(); ++i) {
    ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("Direction", 1000, chatterCallback);

  ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%

