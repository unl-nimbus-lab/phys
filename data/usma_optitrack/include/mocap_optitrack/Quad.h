#ifndef __QUAD_H__
#define __QUAD_H__

// #include "mocap_optitrack/setpoint_scripts.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>

#include "mocap_optitrack/constants_config.h"
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <memory>

namespace {
  const int REAL_QUAD = 0;
  const int FABR_QUAD = 1;
  const int WAND_PROJECTION = 5;
  const int WAND_MOVABLE = 6;
  const int CLOCKWISE_CIRCLE = 10;
}

// Forward declare to prevent circular dependencies
class QuadScript;
bool pose_dist_check(geometry_msgs::Pose pose1,
                     geometry_msgs::Pose pose2,
                     double max_dist, double max_rot);
// #include "mocap_optitrack/QuadScripts.h"

struct QuadData;
class Quad;

// ========
// QuadData
// ========
// A data struct belonging to a Quad that holds all data that needs to be sent
// to QuadScripts, as well as the node to publish with.
struct QuadData {
  // node packaged with data for use of publishers in
  // script functions
  ros::NodeHandle node;

  // Quad's state (header, connected, armed, guided, mode)
  mavros_msgs::State state;

  // Position data
  geometry_msgs::PoseStamped local_pose;
  geometry_msgs::PoseStamped wand_pose;
  geometry_msgs::PoseStamped plat_pose;
  geometry_msgs::PoseStamped ball_pose;

  // Velocity data
  // Angular velocity for wand and plat is not calculated and should not
  // be used!!!
  geometry_msgs::TwistStamped local_vel;
  geometry_msgs::TwistStamped wand_vel;
  geometry_msgs::TwistStamped plat_vel;
  geometry_msgs::TwistStamped ball_vel;

  // Vector of other quads in parent Quad's Formation. Only has public access to
  // other quads.
  std::vector<const Quad*> other_quads;

  // Pointer to this quad with public access
  Quad* this_quad;
};

// ====
// QUAD
// ====
class Quad {
public:
  Quad() {}

  // Initializes node with ns_in and sets up all fields to be filled
  Quad(std::string ns_in);

  // script_queue is all dynamically allocated and must be deleted
  ~Quad();

  // Runs one frame of scripts in script_queue, and controls script selection,
  // deleting and removing scripts when they are completed
  virtual void run();

  // Getters for access to data by other quads, internal
  // script publishers should be passed data and use that
  const geometry_msgs::PoseStamped& get_local_pose() const;
  const geometry_msgs::TwistStamped& get_local_vel() const;
  const mavros_msgs::State& get_state() const;

  // Returns a pointer to the queue at the front of the queue
  QuadScript* front();

  // Returns a pointer to the last script on the queue
  QuadScript* back();

  bool isReal();

  // Adds other_quad to the list of quads this quad can track.
  void add_quad(Quad* other_quad);

  // Adds script to this quad's script queue and passes it the pointer to data.
  // QUAD HAS OWNERSHIP OF QUADSCRIPTS IN SCRIPT_QUEUE
  // USAGE: quad_obj.add_script(new QuadScriptDerived(args));
  void add_script(QuadScript* script_in);

  // Disarms this quad
  void disarm();

protected:

  // The queue of scripts for this quad to run.
  std::queue<QuadScript*> script_queue;

  // Holds all necessary quad data for script publishers
  QuadData data;

  // 0 if this quad corresponds to an actual quadcopter
  int quad_type;

  // ServiceClient to disarm drone
  ros::ServiceClient disarm_client;

  // Subscribers to fill out quad data fields
  ros::Subscriber state_sub;
  ros::Subscriber local_pose_sub;
  ros::Subscriber wand_pose_sub;
  ros::Subscriber plat_pose_sub;
  ros::Subscriber ball_pose_sub;
  ros::Subscriber local_vel_sub;

  // Callbacks for subscribers
  void state_callback(const mavros_msgs::State::ConstPtr& msg);
  void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  virtual void wand_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  virtual void plat_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  virtual void ball_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void local_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  // Velocity functions to calculate and fill out quad data fields
  // Called in their respective position callback function
  void set_wand_vel();
  void set_plat_vel();
  void set_ball_vel();

  // Queues used to store past poses and calculate velocity
  std::queue<geometry_msgs::PoseStamped> past_wand_pose;
  std::queue<geometry_msgs::PoseStamped> past_plat_pose;
  std::queue<geometry_msgs::PoseStamped> past_ball_pose;

  // Checks for wand downward swipe to trigger disarm
  void check_for_disarm_cmd();
};

// ========
// FAKEQUAD
// ========
class FakeQuad : public Quad {
public:
  FakeQuad(int quad_type_in);

  void set_position(double x, double y, double z);

protected:
  bool activated;
  double d_angle;

  virtual void wand_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  virtual void plat_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  virtual void ball_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  virtual void run();
};

// =========
// FORMATION
// =========

// Formations hold Quads and make group movements/control easier.
class Formation {
public:
  Formation();

  // Runs one frame of each quad in this formation's script
  void run();

  // Adds quad to quad_list and links quad to all in formation and vice versa
  void add_quad(Quad& quad);

  // Adds script of type T to all quads in this formation.
  // Constructs a new script for each quad. Ownership is given to the quad.
  template <class T>
  void add_script(T* script) {
    for (int i = 0; i < quad_list.size(); ++i) {
      if (quad_list.at(i)->isReal()) {
        quad_list.at(i)->add_script(new T(*script));
      }
    }
    delete script;
  }

  // Prints the number of quads in this formation
  void print_count();

private:
  // Checks if any quads in this formation are near each other or near a wall,
  // and disarms any that are
  void check_for_collisions();

  // Makes sure quads aren't disarmed on startup before positions are set
  bool initialized;

  // List of all the quads in this formation
  std::vector<Quad*> quad_list;
};

// ================
// HELPER FUNCTIONS
// ================

bool insideBoundaries(geometry_msgs::Pose pose);

#endif
