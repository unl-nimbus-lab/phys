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

#include "../actions/battery_recharge_action.h"
#include <batterymodel.h>
#include <catec_msgs/ScientificInfo.h>
#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <CUDPClient.h>

#pragma pack(4)
#include <Interfaces.h>
#include <common.h>

#define PORT 	   5225
#define QNX_SERVER "10.0.0.40" //Qnx
//Qnx Tierra 10.0.0.40:5225

using namespace std;

extern string uavID;

//Actions
extern LandActionClass *land_action;
extern TakeOffActionClass *take_off_action;


//ros::Subscriber sub_stage;
ros::Publisher state_pub;

//last cmdFlags received
ROS_control_datagram last_cmdFlags;

//last control reference received
ROS_control_datagram last_controlReferences;

//UDP client from CATEC
CUDPClient udp_client;

BatteryModel *battery_model_;
BatteryActionWrapper *battery_action_wraper_;
bool has_goal_battery_recharge_action_;
ros::Subscriber sub_scientific_info_;
catec_msgs::ScientificInfo last_scientific_info_;

//Receiver from uav state
ros::Subscriber sub_state;
void testbedStateCallback(const catec_msgs::UALStateStamped::ConstPtr& fp);

void link_loop(const ros::TimerEvent& te)
{
   last_controlReferences.control_ref.state.uiDataOk = 0xFF;
   last_controlReferences.control_ref.state.uiIdInterface = 13;

   last_controlReferences.control_ref.posRef.dPosRefLat = lastControlReferenceRw.c_reference_rw.position.x;
   last_controlReferences.control_ref.posRef.dPosRefLon = lastControlReferenceRw.c_reference_rw.position.y;
   last_controlReferences.control_ref.posRef.dPosRefH = lastControlReferenceRw.c_reference_rw.position.z;

   last_controlReferences.control_ref.velControlRef.dVelControlRef = lastControlReferenceRw.c_reference_rw.cruise;

   last_controlReferences.control_ref.headingControlRef.dHeadingControlRef = lastControlReferenceRw.c_reference_rw.heading;

   last_controlReferences.control_ref.posRef.dTypePosRef = 1;
   last_controlReferences.control_ref.heartBeat.uiHeartBeat++;


   udp_client.send(&last_controlReferences, sizeof(lastControlReferenceRw));
   //cerr << "Sent data size (control references): "<< sizeof(last_controlReferences) << endl;
   udp_client.send(&last_cmdFlags, sizeof(last_cmdFlags));
   //cerr << "Sent data size (command flags): "<< sizeof(last_cmdFlags) << endl;
   //cerr << "Take off flag: "<< (int)last_cmdFlags.command_flags.cmdFlagsRw.bFlag1 << endl;
   //cerr << "Z: "<< last_controlReferences.control_ref.posRef.dPosRefH << endl;

}

void scientificInfoCallback(const catec_msgs::ScientificInfoConstPtr &scientific_info)
{
   last_scientific_info_ = *scientific_info;
}

void initialize_link(string id, ros::NodeHandle n_)
{
   string topicname;
   topicname =uavID;
   topicname.append("/ual_state");
   state_pub = n_.advertise<catec_msgs::UALStateStamped> (topicname.c_str(), 0);

   //Initialize structures
   last_controlReferences.struct_type= 0;
   last_controlReferences.uav_id = uavIntID;
   last_controlReferences.control_ref.heartBeat.uiHeartBeat = 0;

   last_cmdFlags.struct_type = 1;
   last_cmdFlags.uav_id = uavIntID;

   //Initialize socket
   udp_client.connect((char*)QNX_SERVER,PORT);

   //Subscribe to state topic from testbed.
   char buf[5];
   sprintf(buf,"%d",uavIntID);
   topicname = "qnx_hb_state_";
   topicname.append(buf);
   sub_state = n_.subscribe(topicname.c_str(), 0, testbedStateCallback);

   battery_model_ = new BatteryModel(n_, uavID);
   battery_model_->setBatterytime(ros::Duration(MAX_BATTERY_TIME));

   battery_action_wraper_ = new BatteryActionWrapper(&n_, battery_model_);

   sub_scientific_info_ = n_.subscribe("/scientific_info", 0,
                                       scientificInfoCallback);
}

void CommandFlagsCallback(const uint land_takeoff)
{
   if(land_takeoff==1) //LAND
   {
      last_cmdFlags.command_flags.cmdFlagsRw.bFlag1=0;
      last_cmdFlags.command_flags.cmdFlagsRw.bFlag2=1;
   }
   else if(land_takeoff==2)//TAKE_OFF
   {
      last_cmdFlags.command_flags.cmdFlagsRw.bFlag1=1;
      last_cmdFlags.command_flags.cmdFlagsRw.bFlag2=0;
   }
   last_cmdFlags.command_flags.state.uiDataOk = 0x3;
   last_cmdFlags.command_flags.state.uiIdInterface = 12;
   return;
}

void testbedStateCallback(const catec_msgs::UALStateStamped::ConstPtr& fp)
{
   catec_msgs::UALStateStamped state = (*fp);

   if(actual_state!=(StateUAV)state.ual_state.flying_state &&
         ((StateUAV)state.ual_state.flying_state) == FLYING &&
         lastControlReferenceRw.c_reference_rw.position.x==0 &&
         lastControlReferenceRw.c_reference_rw.position.y==0 &&
         lastControlReferenceRw.c_reference_rw.position.z==0)
   {
      lastControlReferenceRw.c_reference_rw.position.x =
            lastPose.position.x;
      lastControlReferenceRw.c_reference_rw.position.y =
            lastPose.position.y;
      lastControlReferenceRw.c_reference_rw.position.z =
            lastPose.position.z;
      //TODO: Warning it was 0.
      //cerr << "Changing......." << endl;
      lastControlReferenceRw.c_reference_rw.cruise = 0.1;
   }

   actual_state = (StateUAV)state.ual_state.flying_state;

   lastPose.orientation.x = state.ual_state.dynamic_state.orientation.x;
   lastPose.orientation.y = state.ual_state.dynamic_state.orientation.y;
   lastPose.orientation.z = state.ual_state.dynamic_state.orientation.z;

   lastPose.position.x = state.ual_state.dynamic_state.position.x;
   lastPose.position.y = state.ual_state.dynamic_state.position.y;
   lastPose.position.z = state.ual_state.dynamic_state.position.z;

   lastVel.linear.x = state.ual_state.dynamic_state.velocity.x;
   lastVel.linear.y = state.ual_state.dynamic_state.velocity.y;
   lastVel.linear.z = state.ual_state.dynamic_state.velocity.z;

   battery_action_wraper_->uavUpdateState(actual_state);
   //Actualizamos el estado en las actions
   land_action->uavUpdateState(actual_state);
   take_off_action->uavUpdateState(actual_state,
                                   battery_action_wraper_->getHasgoal());

   state.ual_state.remaining_battery =
         battery_model_->update(last_scientific_info_.enable,
                                actual_state!=LANDED);

   state.header.frame_id = uavID;
   //The UAV state is published with modification from the received state.
   state_pub.publish(state);
}

