#ifndef KINOVA_DRIVER_JACO_GRIPPER_ACTION_H
#define KINOVA_DRIVER_JACO_GRIPPER_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

//#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <control_msgs/FollowJointTrajectoryFeedback.h>
//#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/GripperCommandAction.h>
//#include <sensor_msgs/JointState.h>

#include "kinova_driver/jaco_comm.h"

namespace kinova
{

class JacoGripperActionServer
{
public:
  JacoGripperActionServer(JacoComm &, const ros::NodeHandle &n);
  ~JacoGripperActionServer();

  void actionCallback(const control_msgs::GripperCommandGoalConstPtr &);

private:

  ros::NodeHandle node_handle_;
  JacoComm &arm_comm_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server_;

  ros::Time last_nonstall_time_;
  kinova::FingerAngles last_nonstall_finger_positions_;

  // Parameters
  double stall_interval_seconds_;
  double stall_threshold_;
  double rate_hz_;
  float tolerance_;
  double                                       encoder_to_radian_ratio_;
  double                                       radian_to_encoder_ratio_;
};

} // namespace

#endif  // JACO_DRIVER_JACO_GRIPPER_ACTION_H
