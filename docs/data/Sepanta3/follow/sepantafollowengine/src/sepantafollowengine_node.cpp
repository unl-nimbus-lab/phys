#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <sepanta_msgs/command.h>
#include <sepanta_msgs/MasterAction.h>
#include <actionlib/server/simple_action_server.h>
#include <SepantaFollow.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

SepantaFollowEngine *_SepantaFollowEngine;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Sepanta_Follow_Engine");
    ROS_INFO("SepantaFollowEngine Node Started");
    ros::Time::init();

    ros::NodeHandle n;
   
    _SepantaFollowEngine = new SepantaFollowEngine();
  
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
