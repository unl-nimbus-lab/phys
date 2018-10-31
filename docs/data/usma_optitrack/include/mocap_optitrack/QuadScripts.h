#ifndef __QUADSCRIPTS_H__
#define __QUADSCRIPTS_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>

#include "mocap_optitrack/constants_config.h"
#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>

namespace {
  const int WAND_ROTATE_TOP = 51;
  const int WAND_ROTATE_BOTTOM = 52;
}

// Forward declare to prevent circular dependencies
struct QuadData;
class Quad;

// Base class - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Control flow is done from the Quad class, no internal links
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class QuadScript {
public:
  // Subclasses have to initialize any new variables they have
  QuadScript();

  void give_data(QuadData* data_in);

  // MUST IMPLEMENT
  // Initializes publisher with correct type and topic name, and sets initial data
  // for any new member variables or Poses.
  virtual void init() = 0;

  // MUST IMPLEMENT
  // Returns whether or not the script is finished, based on conditions of the
  // parent Quad's data and possibly a wand check.
  virtual bool completed() const = 0;

  // MUST IMPLEMENT
  // Calculates the needed setpoint instructions and publishes it.
  virtual void publish_topic() = 0;

  // Sets whether or not this script needs a wand check
  void set_needsWandCheck(bool input);

protected:
  // Generic publisher, type and topic published may change per script
  ros::Publisher pub;

  // Pointer to parent Quad's data struct
  QuadData* data;

  // General use wand check. Passes when wand rotated up above a threshhold z.
  bool standardWandCheckPasses() const;

  // Whether or not needs to wait for stdwandCheck to pass, default false.
  bool needsWandCheck;
};

// ===============
// DERIVED CLASSES
// ===============

// Gradually increases height until quad reaches z = tkoff_height
class Takeoff : public QuadScript {
public:
  Takeoff(double height_in);

  virtual void init();
  virtual bool completed() const;
  virtual void publish_topic();

protected:
  // Height at end of takeoff
  double tkoff_height;

  // Used to lock takeoff position once armed
  bool locked;

  // Pose to be published to setpoint_position/local
  geometry_msgs::PoseStamped dest_pose;
};

// Sets the quad at a pose: (px, py, pz), (ox, oy, oz, ow)
class SetPose : public QuadScript {
public:
  SetPose(double px, double py, double pz,
          double ox, double oy, double oz, double ow);

  virtual void init();
  virtual bool completed() const;
  virtual void publish_topic();

protected:
  // Pose to set position to
  geometry_msgs::PoseStamped dest_pose;
};

// Follow the specified quad (real or fake) with the offset specified, or with
// a special mode that denotes a nonconstant offset
class FollowOffset : public QuadScript {
public:
  // Follows quad with offset x,y,z
  FollowOffset(double x, double y, double z, int quad_num);

  // Follows quad with special mode
  FollowOffset(int mode_in, int quad_num);

  virtual void init();
  virtual bool completed() const;
  virtual void publish_topic();

protected:
  int quad_to_follow;
  int mode;

  geometry_msgs::PoseStamped dest_pose;
  geometry_msgs::Point offset;
};

// Hovers at a position until a flying ball is detected. Catches (tries to) then
// returns to position
class CatchBall : public QuadScript {
public:
  CatchBall();

  virtual void init();
  virtual bool completed() const;
  virtual void publish_topic();

protected:
  bool catching;
  geometry_msgs::PoseStamped dest_pose;

  // Times after the initial throw is detected
  int timer;

  // Whether dipping should activate: quad will drop as it attempts a catch for
  // softer catch.
  bool dipping;
  int dip_timer;

  double last_z;
  double vz;
  bool hitPeak;
};

// Locks wherever the quad is left, but if pulled from that position it is
// freely movable and will drift (simulates zero g) until held still, then locks
// position. Also modifiable to simulate other physics conditions, e.g. low
// gravity.
class Drift : public QuadScript {
public:
  Drift();

  virtual void init();
  virtual bool completed() const;
  virtual void publish_topic();

protected:
  geometry_msgs::PoseStamped locked_pose;
  geometry_msgs::PoseStamped dest_pose;

  bool init_locked;
  bool locked;
};

// Tracks a moving platform and descends when above it. Compensates for velocity
// so it doesn't lag behind
class MovingLand : public QuadScript {
public:
  MovingLand();

  virtual void init();
  virtual bool completed() const;
  virtual void publish_topic();
protected:
  geometry_msgs::PoseStamped dest_pose;

  // How high above the platform the quad will set position to. This value
  // decreases when above the platform
  double set_z_above;

  bool disarmed;

  // Whether or not the quad is above the platform
  bool abovePlatform() const;
};

// Helper functions - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Returns whether or not the two poses are within distance max_dist AND
// orientation "distance" max_rot
bool pose_dist_check(geometry_msgs::Pose pose1,
                     geometry_msgs::Pose pose2,
                     double max_dist, double max_rot);

// Returns whether or not the two poses are within xy distance max_dist
bool pose_xy_check(geometry_msgs::Pose pose1,
                   geometry_msgs::Pose pose2,
                   double max_dist);

#endif
