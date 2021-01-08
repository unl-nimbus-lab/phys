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
#include "simulation_functions.cpp"
#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include <nav_msgs/Odometry.h>

#include <gazebo/SetModelState.h>
#include <gazebo/GetModelState.h>
#include <gazebo/SetLinkState.h>
#include <gazebo/GetLinkState.h>
#include <gazebo/ApplyJointEffort.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <art_msgs/CarDriveStamped.h>
#include "CarControl.h"

using namespace std;

extern string uavID;

ros::Publisher state_pub;

ros::ServiceClient setState;
ros::ServiceClient getstate;

void UpdateGazeboState();

ros::Publisher pubCarDrive;

void GazeboProcessState(/*const gazebo::GetModelState& state*/) {

   lastPose.position.x = odomlastPose.pose.pose.position.x;
   lastPose.position.y = odomlastPose.pose.pose.position.y;
   lastPose.position.z = odomlastPose.pose.pose.position.z;

   lastPose.orientation.x = odomlastPose.pose.pose.orientation.x;
   lastPose.orientation.y = odomlastPose.pose.pose.orientation.y;
   lastPose.orientation.z = odomlastPose.pose.pose.orientation.z;
   lastPose.orientation.w = odomlastPose.pose.pose.orientation.w;

   //   lastVel.angular.x = state.response.twist.angular.x;
   //   lastVel.angular.y = state.response.twist.angular.y;
   //   lastVel.angular.z = state.response.twist.angular.z;

   //   lastVel.linear.x = state.response.twist.linear.x;
   //   lastVel.linear.y = state.response.twist.linear.y;
   //   lastVel.linear.z = state.response.twist.linear.z;

   catec_msgs::UALStateStamped res;

   res.header.frame_id = uavID;
   res.header.stamp = ros::Time::now();

   res.ual_state.cpu_usage = 50;
   res.ual_state.memory_usage = 50;

   res.ual_state.dynamic_state.position.valid = 1;

   btScalar yaw,pitch,roll;
   btQuaternion q(lastPose.orientation.x,
                  lastPose.orientation.y,
                  lastPose.orientation.z,
                  lastPose.orientation.w);
   btMatrix3x3 m(q);
   m.getEulerYPR(yaw,pitch,roll);

   res.ual_state.dynamic_state.position.x = lastPose.position.x;
   res.ual_state.dynamic_state.position.y = lastPose.position.y;
   res.ual_state.dynamic_state.position.z = lastPose.position.z;

   res.ual_state.dynamic_state.orientation.valid = 1;
   res.ual_state.dynamic_state.orientation.x = roll;
   res.ual_state.dynamic_state.orientation.y = pitch;
   res.ual_state.dynamic_state.orientation.z = tf::getYaw(lastPose.orientation);

   //   res.ual_state.dynamic_state.velocity.valid = 1;
   //   res.ual_state.dynamic_state.velocity.x = lastVel.linear.x;
   //   res.ual_state.dynamic_state.velocity.y = lastVel.linear.y;
   //   res.ual_state.dynamic_state.velocity.z = lastVel.linear.z;

   state_pub.publish(res);
}

void link_loop(const ros::TimerEvent& te)
{
   GazeboProcessState();
   //stepExternalMode();

   if(initState && initControl)
   {
      //Update Matlab inputs
      car_controller_U.Yaw = tf::getYaw(odomlastPose.pose.pose.orientation);
      car_controller_U.PosX = odomlastPose.pose.pose.position.x;
      car_controller_U.PosY = odomlastPose.pose.pose.position.y;

      car_controller_U.XRef = lastControlReferenceRw.c_reference_rw.position.x;
      car_controller_U.YRef = lastControlReferenceRw.c_reference_rw.position.y;
      car_controller_U.VelRef = lastControlReferenceRw.c_reference_rw.cruise;

      /* Step the model */
      car_controller_step();

      art_msgs::CarDriveStamped carD;
      carD.control.speed = car_controller_Y.Power;
      carD.control.steering_angle = car_controller_Y.Direction;
      pubCarDrive.publish(carD);
   }
}

void initialize_link(string id, ros::NodeHandle n_)
{
   srand((unsigned)time(0));
   string topicname;

   topicname =uavID;
   topicname.append("/ual_state");
   state_pub = n_.advertise<catec_msgs::UALStateStamped> (topicname.c_str(), 0);

   setState = n_.serviceClient<gazebo::SetModelState>("/gazebo/set_model_state");
   getstate = n_.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");

   string carTop = uavID+"/pilot/drive";
   pubCarDrive = n_.advertise<art_msgs::CarDriveStamped> (carTop.c_str(), 0);
}

void stepExternalMode(){
   /* External mode */
   {
      boolean_T rtmStopReq = FALSE;
      rtExtModeOneStep(car_controller_M->extModeInfo, 1, &rtmStopReq);
      if (rtmStopReq) {
         printf(" Stop Requests.\n");
         rtmSetStopRequested(car_controller_M, TRUE);

         //Stop external mode
         rtExtModeShutdown(1);

         //Restart externalMode
         rtERTExtModeStartMsg();
      }
   }

   static boolean_T OverrunFlag = 0;

   /* Disable interrupts here */

   /* Check for overrun */
   if (OverrunFlag) {
      rtmSetErrorStatus(car_controller_M, "Overrun");
      return;
   }

   OverrunFlag = FALSE;
}


