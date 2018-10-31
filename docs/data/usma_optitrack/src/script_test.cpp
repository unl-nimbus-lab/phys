#include <ros/ros.h>

#include "mocap_optitrack/setpoint_scripts.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "script_test");
  ros::NodeHandle n;
  ros::Rate rate(FRAMES_PER_SEC);

  ROS_INFO("Waiting 1.0 seconds...");

  sleep(1.0);

  ROS_INFO("Initializing commands...");

  Takeoff takeoff(0.5);

  SetPoseControlled controlled(-0.5, 0, 1, 0, 0, 0, -1);
  controlled.startAfter(&takeoff);


  // FollowAbove follow("/object_pose", 1);
  // follow.startAfter(&takeoff);

  FollowLand land("/object_pose");
  land.startAfter(&controlled);
  //
  ROS_INFO("Executing...");

  // ros::spin();

  while(n.ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
