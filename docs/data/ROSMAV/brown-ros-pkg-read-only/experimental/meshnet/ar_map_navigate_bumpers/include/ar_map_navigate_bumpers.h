#include "ros/ros.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <list>
#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <position_tracker/Position.h>
#include <position_tracker/SetPosition.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <math.h>
#define PI 3.14159265
using namespace std;

class Edge{
  public:
   double Ax;
   double Ay;
   double Bx;
   double By;
   double theta; //slope in the (x,y) coordinate system

  public:
   Edge(double cAx, double cAy, double cBx, double cBy, double ctheta)
   {
       Ax = cAx;
       Ay = cAy;
       Bx = cBx;
       By = cBy;
       theta = ctheta;
   }
};

ros::Publisher vel_pub;
ros::Publisher pospub;
ros::Subscriber pos_sub;
ros::Subscriber bump_sub;
ros::Rate loop_rate(1);
position_tracker::Position cur_pos;
position_tracker::Position prev_pos;
irobot_create_2_1::SensorPacket cur_sensors;
list<Edge *> map4;
double Tdist = 0.1;     
double goal_pos_x; // = 1;
double goal_pos_y; // = 1.5;
list<position_tracker::Position *> waypoints; //this should come from the path planner ROS node
int prevBump = 0;
position_tracker::Position latestBumpPos; 
int ok_to_drive = 1;

void loadMap();
void loadWaypoints();
void positionCallback(const position_tracker::PositionConstPtr& msg);
void bumperCallback(const irobot_create_2_1::SensorPacketConstPtr& msg);
position_tracker::Position getClosestLeftLineProjection();
position_tracker::Position getClosestRightLineProjection();
void error(char *msg);
void substring(const char* text, int start, int stop, char *new_string);
string char_to_string(char *input_p);
double det(double A[2][2]);
void inv(double A[2][2], double IA[2][2]);
void solve(double A[2][2], double C[2], double S[2]);

