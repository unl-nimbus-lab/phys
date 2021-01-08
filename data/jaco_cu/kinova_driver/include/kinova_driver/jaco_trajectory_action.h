#ifndef KINOVA_DRIVER_JACO_TRAJECTORY_ACTION_H
#define KINOVA_DRIVER_JACO_TRAJECTORY_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "kinova_driver/jaco_comm.h"

namespace kinova
{

class JacoTrajectoryActionServer {
public:
  JacoTrajectoryActionServer(JacoComm &, ros::NodeHandle &n);
  ~JacoTrajectoryActionServer();

  void actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &);

private:

  void convertDHAnglesToPhysical(AngularInfo &angles);
  void convertPhysicalAnglesToDH(AngularInfo &angles);
  AngularInfo computeError(AngularInfo &goal, AngularInfo &current);
  float getShortestAngleDistance(float &reference, float &current);
  void normalizeAngles(AngularInfo &angles);
  void printAngles(const char* desc, AngularInfo &angles);
  double normalize(const double value, const double start, const double end);
  bool areValuesClose(float first, float second, float tolerance);

  ros::NodeHandle node_handle_;
  JacoComm &arm_comm_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;
  double goal_tolerance_;
  double kp_gain_;
};

// -------------------------------------------------------------------------------------------------
// Helper Functions
// -------------------------------------------------------------------------------------------------
bool getDoubleParameter(ros::NodeHandle &nh, const std::string &param_name, double &value);

}
#endif  // JACO_DRIVER_JACO_TRAJECTORY_ACTION_H
