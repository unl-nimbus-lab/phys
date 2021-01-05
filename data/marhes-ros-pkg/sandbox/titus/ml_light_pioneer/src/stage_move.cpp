#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Pose.h"
#include "stage_light_ml/move_robot.h"

class StageMove
{
public:
  StageMove(ros::NodeHandle n)
  {
    n_ = n;
    move_srv_ = n_.advertiseService("move_robot", &StageMove::move_cb, this);
    
    //tell the action client that we want to spin a thread by default
    ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);

    //wait for the action server to come up
    while(!ac_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");  
    }
  }
private:
  ros::NodeHandle n_;
  ros::Subscriber odom_sub_;
  ros::ServiceServer move_srv_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac_;
  
  bool move_cb(stage_light_ml::move_robot::Request &req,
               stage_light_ml::move_robot::Response &res )
  {
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position = req.pose.position;
//    goal.target_pose.pose.position.y = req.pose.position.y;
//    goal.target_pose.pose.position.z = req.pose.position.z;
    goal.target_pose.pose.orientation = req.pose.orientation;
//    goal.target_pose.pose.orientation.y = req.pose.orientation.y;
//    goal.target_pose.pose.orientation.z = req.pose.orientation.z;
//    goal.target_pose.pose.orientation.w = req.pose.orientation.w;

    ROS_INFO("Sending goal: %f, %f", goal.target_pose.pose.position.x, 
      goal.target_pose.pose.position.y);
    ac_->sendGoal(goal);

    ac_->waitForResult();
    if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      res.result = true;
      ROS_INFO("Hooray, reached goal");
      return true;
    }
    else
    {
      res.result = false;
      ROS_INFO("The base failed to move forward 1 meter for some reason");
      return false;
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "stage_move");
  ros::NodeHandle n;
  StageMove* sm = new StageMove(n);
  ros::spin();
  return 0;
}
