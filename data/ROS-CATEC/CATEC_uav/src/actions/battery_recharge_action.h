#ifndef _BATTERY_ACTION_WRAPPER_
#define _BATTERY_ACTION_WRAPPER_

#include<ros/ros.h>
#include<batterymodel.h>
#include<actionlib/server/action_server.h>
#include<catec_actions_msgs/BatteryRechargeAction.h>
#include<common.h>

class BatteryActionWrapper
{

public:
   BatteryActionWrapper(ros::NodeHandle *n, BatteryModel *batteryModel):
      as_(*n,"/battery_recharge",false),
      battery_seconds_(0),
      hasGoal(false),
      batteryModel_(batteryModel)
   {
      //Know if have a goal running now
      hasGoal =false;

      //Register callback of the action server.
      as_.registerGoalCallback(boost::bind(
                                  &BatteryActionWrapper::goalCB, this, _1));
      as_.registerCancelCallback(boost::bind(
                                    &BatteryActionWrapper::preemptCB, this, _1));

      main_loop_timer_ = n->createTimer(ros::Duration(1.0),
                                        boost::bind(
                                           &BatteryActionWrapper::mainLoop,
                                           this, _1));

      main_loop_timer_.start();

      as_.start();
   }
   ~BatteryActionWrapper()
   {

   }
   void uavUpdateState(StateUAV current_state)
   {
      //Receive state of uav
      last_state = current_state;
   }
   bool getHasgoal()
   {
      return hasGoal;
   }
   unsigned int getLastBatteryLevel()
   {
      return battery_seconds_;
   }

private:
   void goalCB(actionlib::ServerGoalHandle
               <catec_actions_msgs::BatteryRechargeAction> goal_handle)
   {
      if(!hasGoal || last_state != LANDED)
      {
         hasGoal = true;
         battery_seconds_ = 0.0;
         this->goal_handle_ = goal_handle;
         goal_handle_.setAccepted();

         this->last_goal_ = *goal_handle.getGoal().get();
         feedback_.state = feedback_.MINIMUM_BATTERY_RECHARGE;
         goal_handle.publishFeedback(feedback_);

         batteryModel_->setBatterytime(ros::Duration(0.0));
      }
      else
      {
         goal_handle.setRejected(result_);
      }

   }
   void preemptCB(actionlib::ServerGoalHandle
                  <catec_actions_msgs::BatteryRechargeAction> goal_handle)
   {
      result_.battery_level = (battery_seconds_/(MAX_BATTERY_TIME)*100);
      if(feedback_.state==feedback_.EXTRA_BATTERY_RECHARGE)
      {
         goal_handle.setCanceled(result_);
         hasGoal = false;
      } else {
         goal_handle.setRejected(result_,
                                 "Minimum battery level, it should be recharged.");
      }
   }

   void mainLoop(const ros::TimerEvent& te)
   {
      if(hasGoal)
      {
         if(battery_seconds_ >= (MAX_BATTERY_TIME))
         {
            result_.battery_level = 100;
            goal_handle_.setSucceeded(result_);
            hasGoal = false;
         }
         else if(feedback_.state ==
                 feedback_.MINIMUM_BATTERY_RECHARGE &&
                 batteryModel_->getCurrentLevel() >= MINIMUM_RECHARGE_LEVEL)
         {
            feedback_.state = feedback_.EXTRA_BATTERY_RECHARGE;
            battery_seconds_ = ((MAX_BATTERY_TIME)/100.0) *
                  batteryModel_->getCurrentLevel();
            battery_recharge_start_ = ros::Time::now();
            goal_handle_.publishFeedback(feedback_);
         }
         else if (feedback_.state == feedback_.EXTRA_BATTERY_RECHARGE)
         {
            if(batteryModel_->getCurrentLevel() < MINIMUM_RECHARGE_LEVEL){
               feedback_.state = feedback_.MINIMUM_BATTERY_RECHARGE;
               goal_handle_.publishFeedback(feedback_);
            } else {
               battery_seconds_ =  (batteryModel_->getCurrentLevel()/100.0) *
                     MAX_BATTERY_TIME +  (te.current_real- te.last_real).toSec()*6;

               if(battery_seconds_> MAX_BATTERY_TIME)
                  battery_seconds_ = MAX_BATTERY_TIME;

               std_msgs::Float64 batLevel;
               batLevel.data =
                     ((battery_seconds_/(MAX_BATTERY_TIME))*100);
               result_.battery_level = batLevel.data;
               batteryModel_->setBatteryLevel(batLevel);
            }
         }
      }
   }

protected:
   actionlib::ActionServer<catec_actions_msgs::BatteryRechargeAction> as_;
   actionlib::ServerGoalHandle<catec_actions_msgs::BatteryRechargeAction> goal_handle_;
   catec_actions_msgs::BatteryRechargeGoal last_goal_;
   catec_actions_msgs::BatteryRechargeResult result_;
   catec_actions_msgs::BatteryRechargeFeedback feedback_;
   float battery_seconds_;
   bool hasGoal;
   ros::Time battery_recharge_start_;
   StateUAV last_state;
   ros::Timer main_loop_timer_;
   BatteryModel *batteryModel_;
};
#endif //_BATTERY_ACTION_WRAPPER_
