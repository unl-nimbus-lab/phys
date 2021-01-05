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
#include <actionlib/server/action_server.h>
class TakeOffActionClass
{

public:
   TakeOffActionClass(std::string name, std::string uavID, void (*command_flags)(const uint land_takeoff)) :
      as_(nh_,name,false),
      is_recharging_(false),
      action_name_(name)
   {
      hasGoal =false;
      last_state = LANDED;
      phase = catec_actions_msgs::TakeOffFeedback::STOPPED;

      as_.registerGoalCallback(boost::bind(&TakeOffActionClass::goalCB, this, _1));
      as_.registerCancelCallback(boost::bind(&TakeOffActionClass::preemptCB, this, _1));

      command_flags_cb = command_flags;
      uavID_ = uavID;
      as_.start();
   }

   ~TakeOffActionClass()
   {

   }
   void goalCB(actionlib::ServerGoalHandle<catec_actions_msgs::TakeOffAction> goal_handle)
   {
      catec_actions_msgs::TakeOffResult res;

      if(last_state==FLYING)
      {
         res.result = catec_actions_msgs::TakeOffResult::ALREADY_FLYING;
         goal_handle.setRejected();
      }else if(last_state==TAKING_OFF)
      {
         //Ignore...
      }else if(last_state==LANDING)
      {
         res.result = catec_actions_msgs::TakeOffResult::LANDING;
         goal_handle.setRejected();
      }else
      {
         if(!is_recharging_)
         {
            command_flags_cb(2);
            //std::cerr << "Taking off..." << std::endl;
            goal_handle_ = goal_handle;
            hasGoal = true;
            goal_handle.setAccepted();
         }else
         {
            //std::cerr << "Rejecting take off..." << std::endl;
            goal_handle.setRejected();
         }
      }

   }
   void preemptCB(actionlib::ServerGoalHandle<catec_actions_msgs::TakeOffAction> goal_handle)
   {
      //Try to cancel callback
      //Ignore, actually cant cancel take off action.
   }
   void uavUpdateState(StateUAV actual_state, bool isRecharging)
   {
      //Receive state of uav
      last_state = actual_state;
      is_recharging_ = isRecharging;

      if(hasGoal)
      {
         if(actual_state==FLYING)
         {
            hasGoal = false;

            catec_actions_msgs::TakeOffResult res;
            res.result = catec_actions_msgs::TakeOffResult::FLYING;

            goal_handle_.setSucceeded(res);

         }
      }
   }
protected:

   ros::NodeHandle nh_;
   actionlib::ActionServer<catec_actions_msgs::TakeOffAction> as_;
   actionlib::ServerGoalHandle<catec_actions_msgs::TakeOffAction> goal_handle_;
   bool hasGoal;
   bool is_recharging_;
   std::string action_name_;
   catec_actions_msgs::TakeOffFeedback feedback_;
   catec_actions_msgs::TakeOffResult result_;
   StateUAV last_state;
   void (*command_flags_cb)(const uint land_takeoff);
   unsigned char phase;
   std::string uavID_;
};
