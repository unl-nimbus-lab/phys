#include "mocap_optitrack/setpoint_scripts.h"

// SetPointScript - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
SetPointScript::SetPointScript()
 : completed(false), isStartingScript(true),
   isEndingScript(true), prevScript(NULL) {}

void SetPointScript::startAfter(SetPointScript* prevScript_in) {
  prevScript = prevScript_in;
  prevScript->isEndingScript = false;
  isStartingScript = false;
}

void SetPointScript::makeEndingScript() {
  // isStartingScript = false;
  isEndingScript = true;
}

bool SetPointScript::canStart() {
  // Avoid checking prevScript as it will not exist if isStartingScript
  if (isStartingScript) {
    return true;
  }
  return prevScript->completed;
}

// Takeoff - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Takeoff::Takeoff(double height_in)
 : takeoff_height(height_in), initialized(false), in_air(false), armed(false) {
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  vel_pub = node.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", FRAMES_PER_SEC);
  sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &Takeoff::callback, this);
  state_sub = node.subscribe("mavros/state", 1, &Takeoff::state_call, this);
}

void Takeoff::state_call(const mavros_msgs::State::ConstPtr& state) {
  armed = state->armed;
}

void Takeoff::callback(const POSESTAMPED_CONSTPTR msg) {
  if (canStart()) {
    if (!takeoffCompleted(msg->pose) || isEndingScript) {
      if (!initialized) {
        start_z = msg->pose.position.z;
        initialized = true;
      }
      if (TKOFF_METHOD == TAKEOFF_POSITION_MODE) {
        if (!in_air && msg->pose.position.z > start_z + .05) {  // + (threshold)
          savedPose.header = msg->header;
          savedPose.pose = msg->pose;
          savedPose.pose.position.z = 1.1 * takeoff_height;
          savedPose.pose.orientation.x = 0;
          savedPose.pose.orientation.y = 0;
          in_air = true;
        }
        else if (!in_air){
          savedPose.header = msg->header;
          savedPose.pose = msg->pose;
          savedPose.pose.position.z = 1.1 * takeoff_height;
        }

        if (takeoffCompleted(msg->pose) && isEndingScript) {
          savedPose.pose.position.z = takeoff_height;
        }
        ROS_INFO_ONCE("Taking off (pos mode)...");
        if (!armed) {
          ROS_WARN_ONCE("Drone must be armed and set to offboard mode.");
        }
        pub.publish(savedPose);
      }
      else {
        // TAKEOFF_VELOCITY_MODE only uses this
        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header = msg->header;
        if (!isEndingScript || !takeoffCompleted(msg->pose)) {
          cmd_vel.twist.linear.z = TKOFF_ASCENT_VEL + Z_VEL_OFFSET;
        }
        ROS_INFO_ONCE("Taking off (vel mode)...");
        vel_pub.publish(cmd_vel);
      }
    }
  }
}

bool Takeoff::takeoffCompleted(geometry_msgs::Pose currentPose) {
  if (!completed) {
    completed = (currentPose.position.z > takeoff_height) && armed;
  }
  return completed;
}

// Land - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Land::Land()
 : currentZVel(0), disarmed(false), locked(false), z_drop(0.4) {
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(SET_POSE_TOPIC, FRAMES_PER_SEC, &Land::callback, this);
  vel_sub = node.subscribe("mavros/local_position/velocity", FRAMES_PER_SEC, &Land::vel_callback, this);
  disarm = node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
}

void Land::vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  currentZVel = msg->twist.linear.z;
}

void Land::callback(const POSESTAMPED_CONSTPTR msg) {
  if (canStart()) {
    if (landCompleted(msg->pose) && isEndingScript) {
      if (!disarmed) {
        mavros_msgs::CommandBool disarm_cmd;
        disarm_cmd.request.value = false;
        disarmed = disarm.call(disarm_cmd);
        ROS_INFO("Drone disarmed.");
      }
    }
    else {
      ROS_INFO_ONCE("Landing...");
      if (!locked) {
        lockedPose.header = msg->header;
        lockedPose.pose = msg->pose;
        lockedPose.pose.position.z = 0.45;
        locked = true;
      }
      if (msg->pose.position.z < 0.5) {
        lockedPose.pose.position.z -= 0.1 / FRAMES_PER_SEC;
      }
      pub.publish(lockedPose);
    }
  }
}

bool Land::landCompleted(geometry_msgs::Pose currentPose) {
  if (!completed) {
    completed = (currentPose.position.z < 0.02);
  }
  ROS_INFO(completed ? "completed" : "not completed");
  return completed;
}

// SetPose - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
SetPose::SetPose(geometry_msgs::Pose pose_in) {
  destPose.header.stamp = ros::Time::now();
  destPose.header.frame_id = "fcu";
  destPose.pose = pose_in;
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &SetPose::callback, this);
}

SetPose::SetPose(geometry_msgs::PoseStamped posestamped_in)
 : destPose(posestamped_in) {
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &SetPose::callback, this);
}

SetPose::SetPose(double px, double py, double pz,
        double ox, double oy, double oz, double ow) {
  destPose.header.stamp = ros::Time::now();
  destPose.header.frame_id = "fcu";
  destPose.pose.position.x = px;
  destPose.pose.position.y = py;
  destPose.pose.position.z = pz;
  destPose.pose.orientation.x = ox;
  destPose.pose.orientation.y = oy;
  destPose.pose.orientation.z = oz;
  destPose.pose.orientation.w = ow;
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &SetPose::callback, this);
}

void SetPose::callback(const POSESTAMPED_CONSTPTR msg) {
  if (canStart()) {
    if (!moveToPoseCompleted(msg->pose) || isEndingScript) {
      ROS_INFO_ONCE("Moving to [%.1f][%.1f][%.1f], [%.1f][%.1f][%.1f][%.1f]...",
                    destPose.pose.position.x,
                    destPose.pose.position.y,
                    destPose.pose.position.z,
                    destPose.pose.orientation.x,
                    destPose.pose.orientation.y,
                    destPose.pose.orientation.z,
                    destPose.pose.orientation.w);
      pub.publish(destPose);
    }
  }
}

bool SetPose::moveToPoseCompleted(geometry_msgs::Pose currentPose) {
  if (!completed) {
    double check_distance = 0.4;
    double check_rotation = 0.1;

    double dx = destPose.pose.position.x - currentPose.position.x;
    double dy = destPose.pose.position.y - currentPose.position.y;
    double dz = destPose.pose.position.z - currentPose.position.z;
    double distance_sqd = dx * dx + dy * dy + dz * dz;

    double dox = destPose.pose.orientation.x - currentPose.orientation.x;
    double doy = destPose.pose.orientation.y - currentPose.orientation.y;
    double doz = destPose.pose.orientation.z - currentPose.orientation.z;
    double dow = destPose.pose.orientation.w - currentPose.orientation.w;
    double o_dist_sqd = dox * dox + doy * doy + doz * doz + dow * dow;

    completed = (distance_sqd < check_distance * check_distance) &&
                (o_dist_sqd < check_rotation * check_rotation);
  }
  return completed;
}


// SETPOSE CONTROLLED
SetPoseControlled::SetPoseControlled(geometry_msgs::Pose pose_in)
 : touched_dest(false) {
  destPose.header.stamp = ros::Time::now();
  destPose.header.frame_id = "fcu";
  destPose.pose = pose_in;
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &SetPoseControlled::callback, this);
  wand_sub = node.subscribe("/wand_pose", FRAMES_PER_SEC, &SetPoseControlled::wand_callback, this);
}

SetPoseControlled::SetPoseControlled(geometry_msgs::PoseStamped posestamped_in)
 : destPose(posestamped_in), touched_dest(false) {
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &SetPoseControlled::callback, this);
  wand_sub = node.subscribe("/wand_pose", FRAMES_PER_SEC, &SetPoseControlled::wand_callback, this);
}

SetPoseControlled::SetPoseControlled(double px, double py, double pz,
        double ox, double oy, double oz, double ow)
 : touched_dest(false) {
  destPose.header.stamp = ros::Time::now();
  destPose.header.frame_id = "fcu";
  destPose.pose.position.x = px;
  destPose.pose.position.y = py;
  destPose.pose.position.z = pz;
  destPose.pose.orientation.x = ox;
  destPose.pose.orientation.y = oy;
  destPose.pose.orientation.z = oz;
  destPose.pose.orientation.w = ow;
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &SetPoseControlled::callback, this);
  wand_sub = node.subscribe("/wand_pose", FRAMES_PER_SEC, &SetPoseControlled::wand_callback, this);
}

void SetPoseControlled::callback(const POSESTAMPED_CONSTPTR msg) {
  if (canStart()) {
    if (!moveToPoseCompleted(msg->pose) || isEndingScript) {
      ROS_INFO_ONCE("Moving to [%.1f][%.1f][%.1f], [%.1f][%.1f][%.1f][%.1f]...",
                    destPose.pose.position.x,
                    destPose.pose.position.y,
                    destPose.pose.position.z,
                    destPose.pose.orientation.x, destPose.pose.orientation.y,
                    destPose.pose.orientation.z, destPose.pose.orientation.w);
      ROS_INFO_ONCE("Use wand to control or continue.");


      // Follow wand if wand in correct orientation
      tf::Quaternion q;
      tf::quaternionMsgToTF(wandPose.orientation, q);
      double roll, pitch, yaw;
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // Project position out from wand
      bool wand_check = (std::abs(roll) > 2.8) && (wandPose.position.z > 0.7);
      if (touched_dest && wand_check) {
        double proj_dist = 0.8;
        destPose.pose.position.x = wandPose.position.x - proj_dist * cos(yaw);
        destPose.pose.position.y = wandPose.position.y - proj_dist * sin(yaw);
        destPose.pose.position.z = wandPose.position.z + proj_dist * sin(pitch);

        geometry_msgs::PoseStamped compPose(destPose);
        compPose.pose.position.x +=
          0.0 * (destPose.pose.position.x - msg->pose.position.x);
        compPose.pose.position.y +=
          0.0 * (destPose.pose.position.y - msg->pose.position.y);
        compPose.pose.position.z +=
          0.0 * (destPose.pose.position.z - msg->pose.position.z);
        pub.publish(compPose);
      }
      else {
        pub.publish(destPose);
      }
    }
  }
}

void SetPoseControlled::wand_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  wandPose = msg->pose;
}

bool SetPoseControlled::moveToPoseCompleted(geometry_msgs::Pose currentPose) {
  if (!completed) {
    double check_distance = 0.4;
    double check_rotation = 0.1;

    if (!touched_dest) {
      double dx = destPose.pose.position.x - currentPose.position.x;
      double dy = destPose.pose.position.y - currentPose.position.y;
      double dz = destPose.pose.position.z - currentPose.position.z;
      double distance_sqd = dx * dx + dy * dy + dz * dz;

      double dox = destPose.pose.orientation.x - currentPose.orientation.x;
      double doy = destPose.pose.orientation.y - currentPose.orientation.y;
      double doz = destPose.pose.orientation.z - currentPose.orientation.z;
      double dow = destPose.pose.orientation.w - currentPose.orientation.w;
      double o_dist_sqd = dox * dox + doy * doy + doz * doz + dow * dow;

      touched_dest = (distance_sqd < check_distance * check_distance) &&
                        (o_dist_sqd < check_rotation * check_rotation);
    }

    // Quaternion to RPY
    tf::Quaternion q;
    tf::quaternionMsgToTF(wandPose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // ROS_INFO(" ");
    // ROS_INFO("roll:  %.3f", roll);
    // ROS_INFO("pitch: %.3f", pitch);
    // ROS_INFO("yaw:   %.3f", yaw);

    bool wand_check = (std::abs(roll) < 0.2) &&
                      (wandPose.position.z > 0.8);
    completed = touched_dest && wand_check;
  }
  return completed;
}


// FollowAbove - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
FollowAbove::FollowAbove(std::string follow_topic)
 : z_fol(-1.0), wand_ok(false) {
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(follow_topic, FRAMES_PER_SEC, &FollowAbove::callback, this);
  wand_sub = node.subscribe("/wand_pose", FRAMES_PER_SEC, &FollowAbove::wand_callback, this);
  quad_sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &FollowAbove::quad_callback, this);
}

FollowAbove::FollowAbove(std::string follow_topic, double z_in)
 : z_fol(z_in), wand_ok(false) {
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(follow_topic, FRAMES_PER_SEC, &FollowAbove::callback, this);
  wand_sub = node.subscribe("/wand_pose", FRAMES_PER_SEC, &FollowAbove::wand_callback, this);
  quad_sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &FollowAbove::quad_callback, this);
  z_fol = z_in;
  wand_ok = false;
}

void FollowAbove::callback(const POSESTAMPED_CONSTPTR msg) {
  if (canStart()) {
      if (!abovePlatform(msg->pose) || isEndingScript) {
      geometry_msgs::PoseStamped folPose;
      folPose.header = msg->header;
      folPose.pose = msg->pose;
      folPose.pose.orientation.z = msg->pose.orientation.w;
      folPose.pose.orientation.w = msg->pose.orientation.z;


      if (z_fol < 0) {
        folPose.pose.position.z = msg->pose.position.z + 0.75;
      }
      else {
        folPose.pose.position.z = z_fol;
      }
      ROS_INFO_ONCE("Following object...");
      pub.publish(folPose);
    }
  }
}

void FollowAbove::wand_callback(const POSESTAMPED_CONSTPTR msg) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  wand_ok = (std::abs(roll) < 0.2) &&
            (msg->pose.position.z > 0.8);
}

void FollowAbove::quad_callback(const POSESTAMPED_CONSTPTR msg) {
  quadPose = msg->pose;
}

bool FollowAbove::abovePlatform(const geometry_msgs::Pose platPose) {
  if (!completed) {
    double check_distance = 0.1;
    double check_orient = 0.1;
    double dx = platPose.position.x - quadPose.position.x;
    double dy = platPose.position.y - quadPose.position.y;

    double dz = std::abs(quadPose.orientation.z) - std::abs(platPose.orientation.w);
    double dw = std::abs(quadPose.orientation.w) - std::abs(platPose.orientation.z);

    bool dist_check = ((dx * dx + dy * dy) < (check_distance * check_distance)) &&
                      ((dz * dz + dw * dw) < (check_orient * check_orient));
    completed = dist_check && wand_ok;
  }
  return completed;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
FollowLand::FollowLand(std::string follow_topic)
 : set_z_above(0.75), disarmed(false), queue_initialized(false), dist_trigger(false) {
  pub = node.advertise<geometry_msgs::PoseStamped>(SET_POSE_TOPIC, FRAMES_PER_SEC);
  sub = node.subscribe(follow_topic, FRAMES_PER_SEC, &FollowLand::callback, this);
  quad_sub = node.subscribe(QUAD_POSE_TOPIC, FRAMES_PER_SEC, &FollowLand::quad_callback, this);
  disarm = node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  per_sec = 4.0;
  secs_ahead = 2.0;
}

void FollowLand::quad_callback(const POSESTAMPED_CONSTPTR msg) {
  quadPose = msg->pose;
}

void FollowLand::callback(const POSESTAMPED_CONSTPTR msg) {
  if (canStart()) {
    // Disarm if last script and landed
    if (landCompleted(msg->pose) && isEndingScript && !disarmed) {
      mavros_msgs::CommandBool disarm_cmd;
      disarm_cmd.request.value = false;
      disarmed = disarm.call(disarm_cmd);
      ROS_INFO_ONCE("Drone disarmed.");
    }
    else {
      if (!queue_initialized) {
        for (int i = 0; i < FRAMES_PER_SEC / per_sec; ++i) {
          platform_queue.push(msg->pose);
          quad_queue.push(quadPose);
        }
        queue_initialized = true;
      }

      // Set destination as platform
      geometry_msgs::PoseStamped destPose;
      destPose.header = msg->header;
      destPose.pose = msg->pose;
      destPose.pose.orientation.z = -msg->pose.orientation.w;
      destPose.pose.orientation.w = -msg->pose.orientation.z;


      // Project ahead according to platform velocity
      geometry_msgs::Pose oldPlatPose = platform_queue.front();
      platform_queue.pop();
      platform_queue.push(msg->pose);
      double dx = msg->pose.position.x - oldPlatPose.position.x;
      double dy = msg->pose.position.y - oldPlatPose.position.y;

      destPose.pose.position.x += dx * per_sec * secs_ahead;
      destPose.pose.position.y += dy * per_sec * secs_ahead;

      // Overcompensate by projecting setpose proportionally past destination
      if (std::sqrt(dx * dx + dy * dy) * per_sec > 0.2) {
        destPose.pose.position.x +=
          1 * (msg->pose.position.x - quadPose.position.x);
        destPose.pose.position.y +=
          1 * (msg->pose.position.y - quadPose.position.y);
      }

      if (abovePlatform(msg->pose)) {
        ROS_INFO_ONCE("Descending onto platform...");
        double stop_desc_z = 0.07;
        if (quadPose.position.z - msg->pose.position.z > stop_desc_z) {
          set_z_above -= (0.4) / FRAMES_PER_SEC;
        }
        else {
          set_z_above = stop_desc_z;
        }
      }
      destPose.pose.position.z = set_z_above;
      ROS_INFO_ONCE("Moving above platform...");
      pub.publish(destPose);
    }
  }
}

bool FollowLand::landCompleted(const geometry_msgs::Pose platPose) {
  if (!completed) {
    completed = (quadPose.position.z < platPose.position.z + Z_POS_OFFSET + 0.12) &&
                (abovePlatform(platPose));
  }
  return completed;
}

bool FollowLand::abovePlatform(const geometry_msgs::Pose platPose) {
  double check_distance = 0.12;
  double check_orient = 0.1;
  double dx = platPose.position.x - quadPose.position.x;
  double dy = platPose.position.y - quadPose.position.y;

  double dz = std::abs(quadPose.orientation.z) - std::abs(platPose.orientation.w);
  double dw = std::abs(quadPose.orientation.w) - std::abs(platPose.orientation.z);

  return ((dx * dx + dy * dy) < (check_distance * check_distance)) &&
         ((dz * dz + dw * dw) < (check_orient * check_orient));
}
