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
#include "../actions/battery_recharge_action.h"
#include <batterymodel.h>
#include <catec_msgs/ScientificInfo.h>
#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include <nav_msgs/Odometry.h>

#include <gazebo/SetModelState.h>
#include <gazebo/GetModelState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

using namespace std;

extern LandActionClass *land_action;
extern TakeOffActionClass *take_off_action;

extern string uavID;

ros::Publisher state_pub;
ros::Publisher modelState_pub;
ros::Subscriber modelState_sub;
ros::Subscriber sub_scientific_info_;

catec_msgs::ScientificInfo last_scientific_info_;

ros::ServiceClient setState;
ros::ServiceClient getstate;

BatteryModel *battery_model_;
BatteryActionWrapper *battery_action_wraper_;

int countS = 0;

void UpdateGazeboState();

void link_loop(const ros::TimerEvent& te)
{
   // UpdateGazeboState();

   Twist res;
   Update(&res);

   //Intercambiamos linear x por y, para que se corresponda con el testbed
   double x,y;
   x = res.linear.x;
   y = res.linear.y;
   res.linear.x = x;
   res.linear.y = y;

   gazebo::ModelState srv;
   srv.model_name = uavID;
   srv.pose = lastPose;
   srv.twist = res;
   //   if(actual_state!=LANDED)
   //   {
   //      tf::Quaternion q;
   //      int r,p;
   //      r = (rand()%5)-2.5;
   //      p = (rand()%5)-2.5;
   //      q = tf::createQuaternionFromRPY(r/500.0,
   //                                      p/500.0,
   //                                      tf::getYaw(lastPose.orientation));
   //      srv.pose.orientation.x = q.x();
   //      srv.pose.orientation.y = q.y();
   //      srv.pose.orientation.z = q.z();
   //      srv.pose.orientation.w = q.w();
   //   }
   srv.reference_frame = "world";
   modelState_pub.publish(srv);
}

void GazeboProcessState(const gazebo::ModelState& state) {

   lastPose.position.x = state.pose.position.x;
   lastPose.position.y = state.pose.position.y;
   lastPose.position.z = state.pose.position.z;

   lastPose.orientation.x = state.pose.orientation.x;
   lastPose.orientation.y = state.pose.orientation.y;
   lastPose.orientation.z = state.pose.orientation.z;
   lastPose.orientation.w = state.pose.orientation.w;

   lastVel.angular.x = state.twist.angular.x;
   lastVel.angular.y = state.twist.angular.y;
   lastVel.angular.z = state.twist.angular.z;


   lastVel.linear.x = state.twist.linear.x;
   lastVel.linear.y = state.twist.linear.y;
   lastVel.linear.z = state.twist.linear.z;

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

   res.ual_state.dynamic_state.velocity.valid = 1;
   res.ual_state.dynamic_state.velocity.x = lastVel.linear.x;
   res.ual_state.dynamic_state.velocity.y = lastVel.linear.y;
   res.ual_state.dynamic_state.velocity.z = lastVel.linear.z;

   battery_action_wraper_->uavUpdateState(actual_state);
   //Actualizamos el estado en las actions
   land_action->uavUpdateState(actual_state);
   take_off_action->uavUpdateState(actual_state,
                                   battery_action_wraper_->getHasgoal());

   res.ual_state.remaining_battery =
         battery_model_->update(last_scientific_info_.enable,
                                actual_state!=LANDED);
   res.ual_state.flying_state = actual_state;

   countS++;
   if(countS==10)
   {
      state_pub.publish(res);
      countS = 0;
   }
}

void scientificInfoCallback(const catec_msgs::ScientificInfoConstPtr &scientific_info)
{
   last_scientific_info_ = *scientific_info;
}

void initialize_link(string id, ros::NodeHandle n_)
{
   srand((unsigned)time(0));
   string topicname;

   topicname =uavID;
   topicname.append("/ual_state");
   state_pub = n_.advertise<catec_msgs::UALStateStamped> (topicname.c_str(), 0);

   topicname =uavID;
   topicname.append("/set_model_state");
   modelState_pub = n_.advertise<gazebo::ModelState> (topicname.c_str(), 0);

   topicname =uavID;
   topicname.append("/get_model_state");
   modelState_sub = n_.subscribe(topicname.c_str(), 0,
                                 GazeboProcessState);

   battery_model_ = new BatteryModel(n_, uavID);
   battery_model_->setBatterytime(ros::Duration(MAX_BATTERY_TIME));

   battery_action_wraper_ = new BatteryActionWrapper(&n_, battery_model_);

   sub_scientific_info_ = n_.subscribe("/scientific_info", 0,
                                       scientificInfoCallback);
}

//void UpdateGazeboState() {
//   gazebo::GetModelState state;

//   state.model_name = uavID;
//   state.relative_entity_name = "world";

//   getstate.call(state);

//   GazeboProcessState(state);
//}

