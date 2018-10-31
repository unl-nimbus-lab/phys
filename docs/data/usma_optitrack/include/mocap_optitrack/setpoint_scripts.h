#ifndef __SETPOINT_SCRIPTS_H__
#define __SETPOINT_SCRIPTS_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>

#include "mocap_optitrack/constants_config.h"

#include <queue>
#include <string>
#include <cmath>

#define TAKEOFF_POSITION_MODE 1
#define TAKEOFF_VELOCITY_MODE 2
#define POSESTAMPED_CONSTPTR geometry_msgs::PoseStamped::ConstPtr&

// PARAMETERS - - - - - - - - - - - - - - - - - - - - - - - - - -
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Set takeoff method. Position mode prevents spinning but will
// be very jerky while correcting orientation
const int TKOFF_METHOD = TAKEOFF_POSITION_MODE;

// Ascent velocity during takeoff scripts in velocity mode
const double TKOFF_ASCENT_VEL = 0.5;
const double LND_MAX_VEL = 0.5;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Base class. Don't use instances of this
class SetPointScript {
public:
  // Constructors should initialize subs/pubs with correct topics
  SetPointScript();

  // Only start publishing once prevScript is completed
  void startAfter(SetPointScript* prevScript_in);

  // Makes this the ending script; will not terminate after completion
  void makeEndingScript();

protected:
  ros::NodeHandle node;
  ros::Publisher pub;
  ros::Subscriber sub;

  // Callback fn for subscriber
  virtual void callback(const POSESTAMPED_CONSTPTR msg) = 0;

  // Whether or not this is allowed to attempt running
  // Checks starting conditions, not whether it has finished running
  bool canStart();

  // Whether or not this script has finished publishing/running
  bool completed;

  // Whether or not this is the first script
  bool isStartingScript;

  // If not a starting script, only run after this script is finished
  SetPointScript* prevScript;

  // Whether or not this script has a terminating condition
  bool isEndingScript;

  // Derived classes will most likely also implement an
  // ending condition check that triggers completed
  // and returns completed
};

// Subclass scripts
class Takeoff : public SetPointScript {
public:
  // Constructor sets up pub and sub and takeoff height
  Takeoff(double height_in);

protected:
  // Additional pub for velocity
  ros::Publisher vel_pub;
  ros::Subscriber state_sub;

  double start_z;
  bool initialized;
  bool in_air;
  geometry_msgs::PoseStamped savedPose;

  void state_call(const mavros_msgs::State::ConstPtr& state);

  // Callback for subscriber
  virtual void callback(const POSESTAMPED_CONSTPTR msg);

  // Checks and then returns whether the move has been completed
  bool takeoffCompleted(geometry_msgs::Pose currentPose);

  // Height that copter should ascend to before moving to next script
  double takeoff_height;

  bool armed;
};

class Land : public SetPointScript {
public:
  Land();

protected:
  // Additional sub for velocity feedback
  ros::Subscriber vel_sub;

  // ServiceClient for disarming this if ending script
  ros::ServiceClient disarm;
  bool disarmed;

  // Callback for vel_sub
  virtual void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  // Callback for position sub, publishing done in here
  virtual void callback(const POSESTAMPED_CONSTPTR msg);

  bool landCompleted(geometry_msgs::Pose currentPose);

  bool locked;
  double z_drop;
  geometry_msgs::PoseStamped lockedPose;

  double currentZVel;
};

class SetPose : public SetPointScript {
public:
  // Can take a Pose or a PoseStamped or 7 doubles
  SetPose(geometry_msgs::Pose pose_in);
  SetPose(geometry_msgs::PoseStamped posestamped_in);
  SetPose(double px, double py, double pz,
          double ox, double oy, double oz, double ow);

protected:
  // Callback for sub
  // Publishing done in sub so no position is set if local_position breaks
  virtual void callback(const POSESTAMPED_CONSTPTR msg);

  // Checks and then returns whether the move has been completed
  bool moveToPoseCompleted(geometry_msgs::Pose currentPose);

  // Destination that copter should reach before continuing
  geometry_msgs::PoseStamped destPose;
};

class SetPoseControlled : public SetPointScript {
public:
  // Can take a Pose or a PoseStamped or 7 doubles
  SetPoseControlled(geometry_msgs::Pose pose_in);
  SetPoseControlled(geometry_msgs::PoseStamped posestamped_in);
  SetPoseControlled(double px, double py, double pz,
          double ox, double oy, double oz, double ow);

protected:
  // Listener for wand and stored pose
  ros::Subscriber wand_sub;
  geometry_msgs::Pose wandPose;

  // Callback for sub
  // Publishing done in sub so no position is set if local_position breaks
  virtual void callback(const POSESTAMPED_CONSTPTR msg);

  void wand_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // Checks and then returns whether the move has been completed
  bool moveToPoseCompleted(geometry_msgs::Pose currentPose);

  // Whether the drone has touched destPose at least once
  bool touched_dest;

  // Destination that copter should reach before continuing
  geometry_msgs::PoseStamped destPose;
};

// Follows 1m above or at specified height
class FollowAbove : public SetPointScript {
public:
  // Sets to follow 0.75m above
  FollowAbove(std::string follow_topic);

  // Sets to follow at specified height
  FollowAbove(std::string follow_topic, double z_in);

protected:
  ros::Subscriber quad_sub;
  ros::Subscriber wand_sub;

  virtual void callback(const POSESTAMPED_CONSTPTR msg);

  void wand_callback(const POSESTAMPED_CONSTPTR msg);
  void quad_callback(const POSESTAMPED_CONSTPTR msg);

  bool abovePlatform(const geometry_msgs::Pose platPose);

  bool wand_ok;
  geometry_msgs::Pose quadPose;

  double z_fol;
};

// Follows 1m above or at specified height
class FollowLand : public SetPointScript {
public:
  // Sets to follow 0.75m above and then land on
  FollowLand(std::string follow_topic);

protected:
  // ServiceClient for disarming this if ending script
  ros::ServiceClient disarm;
  bool disarmed;
  bool dist_trigger;

  // Callbacks
  ros::Subscriber quad_sub;
  void quad_callback(const POSESTAMPED_CONSTPTR msg);
  virtual void callback(const POSESTAMPED_CONSTPTR msg);

  bool landCompleted(const geometry_msgs::Pose platPose);

  bool abovePlatform(const geometry_msgs::Pose platPose);

  double set_z_above;
  geometry_msgs::Pose quadPose;

  bool queue_initialized;
  std::queue<geometry_msgs::Pose> platform_queue;
  std::queue<geometry_msgs::Pose> quad_queue;

  // 1 / dt for calculating velocity (higher = smaller dt)
  double per_sec;

  // number of seconds to project destPose into the future
  double secs_ahead;
};


#endif
