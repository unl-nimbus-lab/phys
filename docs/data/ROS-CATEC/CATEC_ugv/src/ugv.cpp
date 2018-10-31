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
#include <nav_msgs/Odometry.h>

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
bool initControl = false;
catec_msgs::ControlReferenceRwStamped lastControlReferenceRw;
/*
 * Implemented in simulation_functions.cpp/real_functions.cpp
 * */
void ControlReferencesRwCallback(const catec_msgs::ControlReferenceRwStamped::ConstPtr& fp);
bool initState = false;
nav_msgs::Odometry odomlastPose;
void LastPoseCallback(const nav_msgs::Odometry::ConstPtr& fp);

/*!
 * \brief Time where receive commandFlag.
 */
ros::Time cmdFlagTime;
ros::Duration cmdFlagDuration;

//Frequency of the main loop
double timerate = 50.0;

void initializeMatlabModel();

#include "link/gazebo_sim.cpp"

int main(int argc, char** argv)
{
   if (argc < 1)
   {
      cout << "This program has two input parameter.\n"<<
              "The first input parameter is the number of the UAV." << endl;
      return -1;
   }

   // The UAV ID is stored in a global variable
   uavID="ugv_";
   uavID.append(string(argv[1]));

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

   amax = 2;
   vmax = 2;
   anglemax = 90;

   ros::init(argc,argv,uavID);

   string topicname;
   topicname = uavID;
   topicname.append("/control_references_rw");
   ros::NodeHandle n;
   ros::Subscriber subFlightPlan;
   subFlightPlan = n.subscribe(topicname.c_str(), 0, ControlReferencesRwCallback);
   ros::Subscriber subOdom;
   string odomtopicname = uavID+"/odom";
   subOdom = n.subscribe(odomtopicname.c_str(), 0, LastPoseCallback);

   initialize_link(uavID,n);

   initializeMatlabModel();

   ros::Rate r(timerate);
   ros::Timer timer = n.createTimer(ros::Duration(1/timerate), link_loop);
   ros::AsyncSpinner as(0);
   as.start();
   ROS_INFO("Main loop");
   while(ros::ok())
   {
       //External mode
       r.sleep();
   }

   return 0;
}

void ControlReferencesRwCallback(const catec_msgs::ControlReferenceRwStamped::ConstPtr& fp)
{
   initControl = true;
   lastControlReferenceRw = *fp;
   lastControlReferenceRw.c_reference_rw.position.z = 0.5;
//   lastControlReferenceRw.c_reference_rw.cruise = 0.1;
//   lastControlReferenceRw.c_reference_rw.heading = 0.0;
}

void LastPoseCallback(const nav_msgs::Odometry::ConstPtr& fp)
{
   initState = true;
   odomlastPose=*fp;
}

void initializeMatlabModel(){
    const char* 		ext_mode_argv[3];
    char 				arg1[] = "-port\0";
    char 				arg2[64];
    //Matlab Initialization
    /* External mode */
    memset(arg2, 0, sizeof(arg2));
    sprintf(arg2, "%d", EXT_MODE_PORT);
    ext_mode_argv[0] = 0;
    ext_mode_argv[1] = arg1;
    ext_mode_argv[2] = arg2;
    rtParseArgsForExtMode(3, ext_mode_argv);

    /* Initialize model */
    car_controller_initialize();

    /* External mode */
    rtSetTFinalForExtMode(&rtmGetTFinal(car_controller_M));
    rtExtModeCheckInit(1);
    {
        boolean_T rtmStopReq = FALSE;
        rtExtModeWaitForStartPkt(car_controller_M->extModeInfo, 1, &rtmStopReq);
        if (rtmStopReq) {
            rtmSetStopRequested(car_controller_M, TRUE);
        }
    }

    rtERTExtModeStartMsg();
}

