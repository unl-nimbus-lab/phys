#ifndef ROMEOSIMULATOR_H
#define ROMEOSIMULATOR_H

#include <ros/ros.h>
#include <naoqi_bridge_msgs/JointTrajectoryAction.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <boost/thread.hpp>

// This class is a fix to be able to have the state of the robot in rviz always updated
// and to pass the orders of moveit controller to the robot
class RomeoSimulator
{
private:
    //ros::Subscriber joints_sub_;
    ros::Subscriber traj_sub_;
    //ros::Publisher robot_state_pub_;

    naoqi_bridge_msgs::JointTrajectoryGoal traj_goal_;
    //moveit_msgs::DisplayRobotState romeo_state_msg_;

    //boost::shared_ptr<boost::thread> pub_thread_;

    //bool have_robot_state_;

    //void callbackUpdateState(sensor_msgs::JointState data);
    void callbackUpdateTrajectory(moveit_msgs::RobotTrajectory data);

    //void publishState();
public:
    RomeoSimulator(ros::NodeHandle nh, std::string trajectory_topic);

    bool executeTrajectory(std::string action_topic);
};

#endif // ROMEOSIMULATOR_H
