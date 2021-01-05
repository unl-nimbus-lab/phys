/*************************************************************************
 *
 * FADA-CATEC
 * __________________
 *
 *  [2013] FADA-CATEC
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of FADA-CATEC and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to FADA-CATEC
 * and its suppliers and may be covered by Europe and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from FADA-CATEC.
 *
 * Created on: 23-Oct-2012
 * Engineer: Jonathan Ruiz Páez
 * Email: jruiz@catec.aero
 */
#include <ros/ros.h>
#include <iostream>

#include <catec_actions_msgs/TakeOffAction.h>
#include <catec_actions_msgs/LandAction.h>
#include <catec_msgs/UALStateStamped.h>
#include <catec_msgs/ControlReferenceRwStamped.h>
#include <actionlib/server/action_server.h>
#include<common.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#define PI 3.1415926535897932384626433832795028841971693993751058f
using namespace std;
using namespace geometry_msgs;

/*
 * Maximun values of uav for simulation
 * */
double amax = 0, vmax = 0, anglemax = 0, transition_time = 0;

/*!
 * \brief The number of the UAV node. Its range will be 0 to 10.
 */
string uavID;
int uavIntID;

/*!
 * \brief Type of the UAV node. 1: Rw; 2: Fw.
 */
string uavType;

/*!
 * \brief Last pose of UAV, should be updated by link function.
 */
geometry_msgs::Pose lastPose;

/*!
 * \brief Last velocity of UAV, should be updated by link function.
 */
Twist lastVel;

/*!
 * \brief Actual state of UAV.
 */
StateUAV actual_state;

/*!
 * \brief Last control reference received
 */
catec_msgs::ControlReferenceRwStamped lastControlReferenceRw;
//catec_msgs::ControlReferenceFwStamped lastControlReferenceFw;

/*!
 * \brief Time where receive commandFlag.
 */
ros::Time cmdFlagTime;
ros::Duration cmdFlagDuration;

//Frequency of the main loop
double timerate = 1000;

/*
 * Implemented in simulation_functions.cpp/real_functions.cpp
 * */
void ControlReferencesRwCallback(const catec_msgs::ControlReferenceRwStamped::ConstPtr& fp);

//void ControlReferencesFwCallback(const catec_msgs::ControlReferenceFwStamped::ConstPtr& fp);

#include "actions/take_off_action.h"
#include "actions/land_action.h"

#if defined(STAGE_SIM)
#include "link/stage_sim.cpp"
#elif defined(GAZEBO_SIM)
#include "link/gazebo_sim.cpp"
#elif defined(GAZEBO_HECTOR_SIM)
#include "link/gazebo_hector_sim.cpp"
#elif defined(CATEC_SIM)
#include "link/catec_sim.cpp"
#elif defined(QNX_HUMMINGBIRD)
#include "link/qnx_hummingbird.cpp"
#endif

double xHovering;
double yHovering;
bool refHovering = false;

/*
 * Actions Objects
 * */
LandActionClass *land_action;
TakeOffActionClass *take_off_action;

/*
 * TakeOfAction CommandFlags Callback
 * */
void CommandFlagsCallback(const uint land_takeoff);

int main(int argc, char** argv)
{
   if (argc < 2)
   {
      cout << "This program has two input parameter.\n"<<
              "The first input parameter is the number of the UAV." << endl <<
              "The second input parameter is the type of the UAV." << endl;
      return -1;
   }
   // The UAV ID is stored in a global variable
   uavID="uav_";
   uavID.append(string(argv[1]));
   uavType = string(argv[2]);

   if(uavType == "1"){
      cout << "UAV type: Rw;" << endl;
   }else if(uavType == "2"){
      cout << "UAV type: Fw;" << endl;
   }else
   {
      cout << "Unknow uav type;" << endl;
      return 1;
   }

   try
   {
      uavIntID = boost::lexical_cast<int>(argv[1]);
   }
   catch(boost::bad_lexical_cast const&)
   {
      perror("The first argument is not a number.");
      return 1;
   }

   lastPose.position.x = -5;
   lastPose.position.y = uavIntID -3;

   string topicname;
   //Recopilamos parametros de ros para nuestro uav
   topicname = uavID;
   topicname.append("/amax");
   ros::param::get(topicname.c_str(), amax);

   topicname = uavID;
   topicname.append("/vmax");
   ros::param::get(topicname.c_str(), vmax);

   topicname = uavID;
   topicname.append("/anglemax");
   ros::param::get(topicname.c_str(), anglemax);
   anglemax *= PI/180; 						// Conversión grados-->radianesas

   topicname = uavID;
   topicname.append("/transition_time");
   ros::param::get(topicname.c_str(), transition_time);

   amax = 150;
   vmax = 10;
   anglemax = 90;

   ros::init(argc,argv,uavID);

   ros::NodeHandle n;
   ros::Subscriber subFlightPlan;
   if(uavType=="1")
   {
      cout << "Suscripcion iniciada..." << endl;
      topicname = uavID;
      topicname.append("/control_references_rw");
      subFlightPlan = n.subscribe(topicname.c_str(), 0, ControlReferencesRwCallback);
   } else
   {
      /*topicname = uavID;
      topicname.append("/control_references_fw");
      ros::Subscriber subFlightPlan = n.subscribe(topicname.c_str(), 0, ControlReferencesRwCallback);*/
   }

   initialize_link(uavID,n);

   string take_name = uavID;
   take_name.append("/take_off_action");
   TakeOffActionClass take_off_action2(take_name,uavID,&CommandFlagsCallback);
   take_off_action = &take_off_action2;

   string land_name = uavID;
   land_name.append("/land_action");
   LandActionClass land_action2(land_name,uavID,&CommandFlagsCallback);
   land_action = &land_action2;

   actual_state = LANDED;

   ros::Timer timer = n.createTimer(ros::Duration(1/timerate), link_loop);

   ros::spin();
   return 0;
}

void ControlReferencesRwCallback(
      const catec_msgs::ControlReferenceRwStamped::ConstPtr& fp)
{
   if(actual_state==FLYING)
   {
      lastControlReferenceRw = *fp;

   } else if(actual_state==LANDED && !refHovering){
      xHovering = fp.get()->c_reference_rw.position.x;
      yHovering = fp.get()->c_reference_rw.position.y;
      refHovering = true;

   } else if(actual_state==TAKING_OFF) {
      lastControlReferenceRw = *fp;
      lastControlReferenceRw.c_reference_rw.position.x = xHovering;
      lastControlReferenceRw.c_reference_rw.position.y = yHovering;
      lastControlReferenceRw.c_reference_rw.position.z = 0.5;
      lastControlReferenceRw.c_reference_rw.cruise = 0.1;
      lastControlReferenceRw.c_reference_rw.heading = 0.0;
   }
}
/*
void ControlReferencesFwCallback(const catec_msgs::ControlReferenceFwStamped::ConstPtr& fp)
{
   if(actual_state==FLYING)
   {
      lastControlReferenceFw = *fp;
   }
}*/
