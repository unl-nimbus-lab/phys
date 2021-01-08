#include <stdlib.h>
#include "UavControllerNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "uav_controller_node");
  UavControllerNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
