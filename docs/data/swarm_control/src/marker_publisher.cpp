#include <stdlib.h>
#include "MarkerPublisher.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "marker_publisher");
  MarkerPublisher node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
