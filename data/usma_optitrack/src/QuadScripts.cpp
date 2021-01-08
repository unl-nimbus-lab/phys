#include "mocap_optitrack/Quad.h"
#include "mocap_optitrack/QuadScripts.h"

// Base class - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Publisher initialized in specific script class
QuadScript::QuadScript()
 : data(NULL), needsWandCheck(false) {}

void QuadScript::give_data(QuadData* data_in) {
  assert(data_in != NULL);
  data = data_in;
  init();
}

bool QuadScript::standardWandCheckPasses() const {
  bool rot_check = std::abs(data->wand_pose.pose.orientation.y) < 0.1;
  bool z_check = data->wand_pose.pose.position.z > 0.8;
  return rot_check && z_check;
}

void QuadScript::set_needsWandCheck(bool input) {
  needsWandCheck = input;
}

// TAKEOFF - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Takeoff::Takeoff(double height_in) : tkoff_height(height_in), locked(false) {}

void Takeoff::init() {
  ROS_INFO("Takeoff initialized");
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
}

bool Takeoff::completed() const {
  return data->local_pose.pose.position.z > tkoff_height;
}

void Takeoff::publish_topic() {
  ROS_INFO_ONCE("Starting Takeoff...");
  if (!locked) {
    dest_pose = data->local_pose;
    locked = data->state.armed && data->state.mode == "OFFBOARD";
    if (locked) {
      ROS_INFO("takeoff position locked!");
    }
  }
  else {
    dest_pose.pose.position.z += (0.15) / FRAMES_PER_SEC;
  }
  pub.publish(dest_pose);
}


// SETPOSE - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
SetPose::SetPose(double px, double py, double pz,
                 double ox, double oy, double oz, double ow) {
  dest_pose.pose.position.x = px;
  dest_pose.pose.position.y = py;
  dest_pose.pose.position.z = pz;

  dest_pose.pose.orientation.x = ox;
  dest_pose.pose.orientation.y = oy;
  dest_pose.pose.orientation.z = oz;
  dest_pose.pose.orientation.w = ow;
}

void SetPose::init() {
  dest_pose.header = data->local_pose.header;

  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
  ROS_INFO("SetPose initialized");
}

bool SetPose::completed() const {
  double dist = 0.4;
  double rot = 0.1;

  bool wand_check = needsWandCheck ? standardWandCheckPasses() : 1;
  return wand_check && pose_dist_check(dest_pose.pose,
                                       data->local_pose.pose,
                                       dist, rot);
}

void SetPose::publish_topic() {
  ROS_INFO_ONCE("Starting SetPose");
  dest_pose.header.stamp = ros::Time::now();
  pub.publish(dest_pose);
}

// FOLLOWOFFSET - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
FollowOffset::FollowOffset(double x, double y, double z, int quad_num)
 : mode(0), quad_to_follow(quad_num) {
  offset.x = x;
  offset.y = y;
  offset.z = z;
}

FollowOffset::FollowOffset(int mode_in, int quad_num)
 : mode(mode_in), quad_to_follow(quad_num) {}

void FollowOffset::init() {
  dest_pose.header = data->local_pose.header;
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
  ROS_INFO("FollowOffset initialized");
}

bool FollowOffset::completed() const {
  double dist = 0.2;
  double rot = 0.2;
  bool wand_check = needsWandCheck ? standardWandCheckPasses() : 1;
  bool dist_check = pose_dist_check(dest_pose.pose,
                                    data->local_pose.pose,
                                    dist, rot);
  return wand_check && dist_check;
}

void FollowOffset::publish_topic() {
  ROS_INFO_ONCE("Starting FollowOffset");
  dest_pose = data->other_quads.at(quad_to_follow)->get_local_pose();
  dest_pose.header.stamp = ros::Time::now();
  if (mode == 0) {
    dest_pose.pose.position.x += offset.x;
    dest_pose.pose.position.y += offset.y;
    dest_pose.pose.position.z += offset.z;
  }
  else {
    tf::Quaternion q;
    tf::quaternionMsgToTF(data->wand_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (mode == WAND_ROTATE_TOP) {
      dest_pose.pose.position.y -= sin(roll) * 0.5;
      dest_pose.pose.position.x += cos(roll) * 0.5;
    }
    else if (mode == WAND_ROTATE_BOTTOM) {
      dest_pose.pose.position.y += sin(roll) * 0.5;
      dest_pose.pose.position.x -= cos(roll) * 0.5;
    }
  }
  pub.publish(dest_pose);
}

// CatchBall - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// TODO this is placeholder
CatchBall::CatchBall() : catching(false), last_z(0), hitPeak(false), timer(0),
  dip_timer(0), dipping(false) {}

void CatchBall::init() {
  // TODO This is placeholder
  dest_pose.header = data->local_pose.header;
  dest_pose.pose = data->local_pose.pose;
  dest_pose.pose.position.z = 0.5;
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
}

bool CatchBall::completed() const {
  // TODO placeholder
  return false;
}

void CatchBall::publish_topic() {
  ROS_INFO_ONCE("CatchBall started");
  double catch_z = 0.5;

  // Calculate remaining t in ball flight
  double x = data->ball_pose.pose.position.x;
  double y = data->ball_pose.pose.position.y;
  double z = data->ball_pose.pose.position.z;

  double a = -9.8;
  double vx = data->ball_vel.twist.linear.x;
  double vy = data->ball_vel.twist.linear.y;
  if (!catching) {
    vz = data->ball_vel.twist.linear.z;
  }
  else {
    // Much faster instantaneous vz calculation needed
    vz = (z - last_z) * FRAMES_PER_SEC;
  }

  // Calculate t until ball reaches catch_z
  double t_plus = (-vz + sqrt(vz*vz - 2*a*(z - catch_z))) / a;
  double t_minus = (-vz - sqrt(vz*vz - 2*a*(z - catch_z))) / a;
  double t = t_plus > t_minus ? t_plus : t_minus;

  // Project to predict ending coordinates
  // TODO figure out why the 4* is needed
  double x_proj = x + 4 * (vx * t);
  double y_proj = y + 4 * (vy * t);

  // See if ball has been thrown yet
  if (!catching) {
    dest_pose.header = data->local_pose.header;
    dest_pose.pose.position.x = -0;
    dest_pose.pose.position.y = 0;
    dest_pose.pose.position.z = 0.5;
    dest_pose.pose.orientation.w = -1;

    if (catching = vz > 0.3 && z > 1) {
      ROS_INFO("catching!");
    }
  }

  // See if ball has reached peak of trajectory
  if (!hitPeak) {
    hitPeak = last_z > z;
  }
  last_z = z;

  // Start catch timer
  if (catching) {
    ++timer;
  }

  // Begin catching movement
  geometry_msgs::PoseStamped comp_pose = dest_pose;
  if (timer > FRAMES_PER_SEC / 8) {

    // Move to projected ball coordinates at catch height
    if (!dipping) {
      dest_pose.pose.position.x = x_proj;
      dest_pose.pose.position.y = y_proj;
      dest_pose.pose.position.z = catch_z;
      ROS_INFO("Predicted coords: %.2f, %.2f in %.2fs", x_proj, y_proj, t);

      // Overcompensate for more aggressive maneuvering
      double dx = dest_pose.pose.position.x - data->local_pose.pose.position.x;
      double dy = dest_pose.pose.position.y - data->local_pose.pose.position.y;
      comp_pose.pose.position.x += 6 * dx;
      comp_pose.pose.position.y += 6 * dy;

      // See if about to catch
      if (dipping = z < catch_z + 0.2) {
        ROS_INFO("timer ref: %d", timer);
        ROS_INFO("Actual coords: %.2f, %.2f", x, y);
        ROS_INFO("STARTING DIP");
      }
    }

    // Dip to softly catch ball
    else {
      ++dip_timer;

      // Move along trajectory toward a predicted spot underground
      double dip_z = -1.0;
      double t_plus_dip = (-vz + sqrt(vz*vz - 2*a*(z - dip_z))) / a;
      double t_minus_dip = (-vz - sqrt(vz*vz - 2*a*(z - dip_z))) / a;
      double t_dip = t_plus_dip > t_minus_dip ? t_plus_dip : t_minus_dip;

      comp_pose.pose.position.x = x + 4 * (vx * t_dip);
      comp_pose.pose.position.y = y + 4 * (vy * t_dip);
      comp_pose.pose.position.z = dip_z;

      ROS_INFO("Dip coords: %.2f, %.2f, -1.0", comp_pose.pose.position.x, comp_pose.pose.position.y);

      // Reset everything when dip is completed
      if (dip_timer >= FRAMES_PER_SEC / 4) {
        ROS_INFO("timer ref: %d", timer);
        ROS_INFO("ended dip");
        ROS_INFO("RESETTING...");

        dipping = hitPeak = catching = false;
        dip_timer = timer = 0;
      }
    }
  }

  pub.publish(comp_pose);
}

// Place ATTEMPT 2 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Drift::Drift() : locked(false), init_locked(false) {}

void Drift::init() {
  // dest_pose.pose.position.x = -1.0;
  dest_pose.header = data->local_pose.header;
  dest_pose.pose.position.z = 0.5;
  dest_pose.pose.orientation.w = -1;
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
}

bool Drift::completed() const {
  // TODO this is placeholder
  return false;
}

void Drift::publish_topic() {
  ROS_INFO_ONCE("Drift started");

  // Get some params
  double z_vel_offset = 0.01;
  double vx = data->local_vel.twist.linear.x;
  double vy = data->local_vel.twist.linear.y;
  double vz = data->local_vel.twist.linear.z - z_vel_offset;


  if (!locked) {
    locked_pose.pose = data->local_pose.pose;

    // Determine if should lock
    if (locked = (std::abs(vx) < 0.04 &&
                  std::abs(vy) < 0.04 &&
                  std::abs(vz) < 0.04) || !init_locked) {
      ROS_INFO("Locking new position at %.2f, %.2f, %.2f \n", locked_pose.pose.position.x,
                                                              locked_pose.pose.position.y,
                                                              locked_pose.pose.position.z);
      if (!init_locked) {
        ROS_INFO("making initial position lock");
        init_locked = true;
      }
    }
    else {
      // Otherwise set drift conditions
      locked_pose.pose.position.x += vx;
      locked_pose.pose.position.y += vy;
      locked_pose.pose.position.z += vz - (1.0 / FRAMES_PER_SEC);
    }
  }

  // Unlock if pulled far enough
  if (locked && !pose_dist_check(locked_pose.pose, data->local_pose.pose, 0.17, 1)) {
    ROS_INFO("UNLOCKED");
    locked = false;
  }

  locked_pose.header.stamp = ros::Time::now();
  pub.publish(locked_pose);
}


// MOVINGLAND - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
MovingLand::MovingLand() : set_z_above(0.5), disarmed(false) {}

void MovingLand::init() {
  dest_pose = data->plat_pose;
  pub = data->node.advertise<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local",
                             FRAMES_PER_SEC);
}

bool MovingLand::completed() const {
  return !data->state.armed;
}

void MovingLand::publish_topic() {
  ROS_INFO_ONCE("Starting MovingLand");
  // Aim for pose above platform
  dest_pose = data->plat_pose;
  dest_pose.pose.position.z = data->plat_pose.pose.position.z + set_z_above;

  // Align orientation of quad and platform (depends on arbitrary defn)
  dest_pose.pose.orientation.z = -data->plat_pose.pose.orientation.w;
  dest_pose.pose.orientation.w = -data->plat_pose.pose.orientation.z;

  // Set to move at same vel as plat
  double vx = data->plat_vel.twist.linear.x;
  double vy = data->plat_vel.twist.linear.y;
  dest_pose.pose.position.x += 2.2 * vx;
  dest_pose.pose.position.y += 2.2 * vy;

  // Overcompensate to catch up to plat
  // if (std::sqrt(vx*vx + vy*vy) > 0.10) {
  if (true) {
    ROS_INFO_ONCE("Overcompensating");
    double dx = data->plat_pose.pose.position.x - data->local_pose.pose.position.x;
    double dy = data->plat_pose.pose.position.y - data->local_pose.pose.position.y;
    dest_pose.pose.position.x += 1.3 * dx;
    dest_pose.pose.position.y += 1.3 * dy;
  }

  if (abovePlatform()) {
    // Disarm if close enough
    double dz = data->local_pose.pose.position.z -
                data->plat_pose.pose.position.z;
    if (!disarmed && dz < 0.12) {
      data->this_quad->disarm();
      ROS_INFO("Quad disarmed.");
      disarmed = true;
    }

    // Descend if above platform
    double fall_dist = -0.10;
    if ((dest_pose.pose.position.z -
         data->plat_pose.pose.position.z) > fall_dist) {

      // TODO THIS CHECK FOR DEMO ONLY
      // Only descend if platform moving
      if (std::sqrt(vx*vx + vy*vy) > 0.10) {
        set_z_above -= (0.5) / FRAMES_PER_SEC;
      }
    }
    else {
      set_z_above = data->plat_pose.pose.position.z + fall_dist;
    }
  }

  // Publish
  dest_pose.header.stamp = ros::Time::now();
  pub.publish(dest_pose);
}

bool MovingLand::abovePlatform() const {
  return pose_xy_check(data->plat_pose.pose, data->local_pose.pose, 0.12);
}

// HELPER FUNCTIONS - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool pose_dist_check(geometry_msgs::Pose pose1,
                     geometry_msgs::Pose pose2,
                     double max_dist, double max_rot) {
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  // ROS_INFO("dist: %.2f", std::sqrt(dx*dx + dy*dy + dz*dz));
  bool dist_check = std::sqrt(dx*dx + dy*dy + dz*dz) < max_dist;

  double dox = pose1.orientation.x -
               pose2.orientation.x;
  double doy = pose1.orientation.y -
               pose2.orientation.y;
  double doz = pose1.orientation.z -
               pose2.orientation.z;
  double dow = pose1.orientation.w -
               pose2.orientation.w;
  bool rot_check = std::sqrt(dox*dox + doy*doy + doz*doz + dow*dow) < max_rot;

  return dist_check;
}

bool pose_xy_check(geometry_msgs::Pose pose1,
                   geometry_msgs::Pose pose2,
                   double max_dist) {
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  return std::sqrt(dx*dx + dy*dy) < max_dist;
}
