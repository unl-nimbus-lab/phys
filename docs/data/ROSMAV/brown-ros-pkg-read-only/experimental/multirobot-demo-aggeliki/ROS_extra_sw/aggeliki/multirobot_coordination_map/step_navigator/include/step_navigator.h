#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <position_tracker/Position.h>
#include <position_tracker/SetPosition.h>
#include <path_navigator/getNextWaypoint.h>
#include <math.h>
#define PI 3.14159265

using namespace std;

ros::NodeHandle *n1;
ros::Publisher vel_pub;
ros::Subscriber pos_sub;
ros::Rate loop_rate(1);
position_tracker::Position cur_pos;
double goal_pos_x;
double goal_pos_y;
int ok_to_drive;

void positionCallback(const position_tracker::PositionConstPtr& msg);
void error(char *msg);



