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
#ifndef _LAND_ACTION_CLASS
#define _LAND_ACTION_CLASS

class LandActionClass
{

public:
	LandActionClass(std::string name, std::string uavID, void (*command_flags)(const uint land_takeoff)) :
		as_(nh_,name,false),
		action_name_(name)
	{
		hasGoal =false;
		last_state = FLYING;
		phase = catec_actions_msgs::LandFeedback::STOPPED;

		as_.registerGoalCallback(boost::bind(&LandActionClass::goalCB, this, _1));
		as_.registerCancelCallback(boost::bind(&LandActionClass::preemptCB, this, _1));

		command_flags_cb = command_flags;
		uavID_ = uavID;
		as_.start();

	}

	~LandActionClass()
	{

	}
	void goalCB(actionlib::ServerGoalHandle<catec_actions_msgs::LandAction> goal_handle)
	{
		catec_actions_msgs::LandResult res;
		//TakeOff advertise;
		if(last_state==LANDED)
		{
			res.result = catec_actions_msgs::LandResult::ALREADY_LANDED;
			goal_handle.setRejected(res);
		}else if(last_state==LANDING)
		{
			//Ignore...
		}else if(last_state==TAKING_OFF)
		{
			res.result = catec_actions_msgs::LandResult::TAKINGOFF;
			goal_handle.setRejected(res);
		}else
		{

			command_flags_cb(1);

			goal_handle_ = goal_handle;
			hasGoal = true;
			goal_handle.setAccepted();
		}

	}
	void preemptCB(actionlib::ServerGoalHandle<catec_actions_msgs::LandAction> goal_handle)
	{
		//Try to cancel callback
		//Ignore actually cant cancel take off action.
	}
	void uavUpdateState(StateUAV actual_state)
	{
		//Receive state of uav
		last_state = actual_state;

		if(hasGoal)
		{
			if(actual_state==LANDED)
			{
				hasGoal = false;
				catec_actions_msgs::LandResult res;
				res.result = catec_actions_msgs::LandResult::LANDED;
				goal_handle_.setSucceeded(res);
			}
		}
	}
protected:

	ros::NodeHandle nh_;
	actionlib::ActionServer<catec_actions_msgs::LandAction> as_;
	actionlib::ServerGoalHandle<catec_actions_msgs::LandAction> goal_handle_;
	bool hasGoal;
	std::string action_name_;
	catec_actions_msgs::LandFeedback feedback_;
	catec_actions_msgs::LandResult result_;
	StateUAV last_state;
	void (*command_flags_cb)(const uint land_takeoff);
	unsigned char phase;
	std::string uavID_;
};
#endif
