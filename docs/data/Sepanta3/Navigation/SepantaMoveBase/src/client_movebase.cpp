#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sepanta_msgs/MasterAction.h>


actionlib::SimpleActionClient<sepanta_msgs::MasterAction> * ac;


void thread_logic()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    //ac->cancelGoal ();

}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_movebase");

  // create the action client
  // true causes the client to spin its own thread
 
  boost::thread _thread_(&thread_logic);

  ROS_INFO("Waiting for action server to start.");

  ac = new actionlib::SimpleActionClient<sepanta_msgs::MasterAction>("SepantaMoveBaseAction", true);
  // wait for the action server to start
  ac->waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sepanta_msgs::MasterGoal goal;
  goal.action = "exe";
  goal.id = "point1";

  ac->sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac->waitForResult(ros::Duration(1000));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

    sepanta_msgs::MasterResult::ConstPtr _res = ac->getResult();

     ROS_INFO("Action result : %s",_res->result.c_str());

  }
  else
  {
     ac->cancelGoal ();
     ROS_INFO("Action did not finish before the time out.");
  }
   

   _thread_.interrupt();
   _thread_.join();

  //exit
  return 0;
}