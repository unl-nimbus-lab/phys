#include "mocap_optitrack/Quad.h"
#include "mocap_optitrack/QuadScripts.h"

// PUBLIC - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Constructor and Destructor
Quad::Quad(std::string ns_in) {
  quad_type = REAL_QUAD;
  data.node = ros::NodeHandle(ns_in);
  data.this_quad = this;
  // Initialize vel queues to correct size, data cycled through in pose subs
  geometry_msgs::PoseStamped init_pose;
  for (int i = 0; i < FRAMES_PER_SEC / VEL_T_DENOM; ++i) {
    past_wand_pose.push(init_pose);
    past_plat_pose.push(init_pose);
    past_ball_pose.push(init_pose);
  }

  // Initialize subs
  state_sub = data.node.subscribe("mavros/state",
                                  FRAMES_PER_SEC,
                                  &Quad::state_callback,
                                  this);
  local_pose_sub = data.node.subscribe("mavros/local_position/pose",
                                       FRAMES_PER_SEC,
                                       &Quad::local_pose_callback,
                                       this);
  wand_pose_sub = data.node.subscribe("/wand_pose",
                                      FRAMES_PER_SEC,
                                      &Quad::wand_pose_callback,
                                      this);
  plat_pose_sub = data.node.subscribe("/object_pose",
                                      FRAMES_PER_SEC,
                                      &Quad::plat_pose_callback,
                                      this);
  ball_pose_sub = data.node.subscribe("/ball_pose",
                                      FRAMES_PER_SEC,
                                      &Quad::ball_pose_callback,
                                      this);
  local_vel_sub = data.node.subscribe("mavros/local_position/velocity",
                                      FRAMES_PER_SEC,
                                      &Quad::local_vel_callback,
                                      this);

  // Initialize disarming client
  disarm_client = data.node.serviceClient<mavros_msgs::CommandBool>
                                         ("mavros/cmd/arming");
}

// Mostly not called, termination done with ^C...
Quad::~Quad() {
  ROS_INFO_ONCE("Cleaning up remaining quadscripts memory");
  while (!script_queue.empty()) {
    delete script_queue.front();
    script_queue.pop();
  }
}

void Quad::run() {
  check_for_disarm_cmd();

  // Run this quad's active script
  if (!script_queue.empty()) {
    int size = script_queue.size();

    // Delete script and move to next one if completed
    if (script_queue.front()->completed() && script_queue.size() > 1) {
      ROS_INFO("Script completed, starting next one.");
      delete script_queue.front();
      script_queue.pop();
    }

    script_queue.front()->publish_topic();
    ros::spinOnce();
  }
}

// Getters
const geometry_msgs::PoseStamped& Quad::get_local_pose() const {
  return data.local_pose;
}

const geometry_msgs::TwistStamped& Quad::get_local_vel() const {
  return data.local_vel;
}

QuadScript* Quad::front() {
  return script_queue.front();
}

QuadScript* Quad::back() {
  return script_queue.back();
}

bool Quad::isReal() {
  return quad_type == REAL_QUAD;
}

// Adders
void Quad::add_quad(Quad* other_quad) {
  data.other_quads.push_back(other_quad);
}

void Quad::add_script(QuadScript* script_in) {
  script_in->give_data(&data);
  script_queue.push(script_in);
}

// Disarm
void Quad::disarm() {
  if (data.state.armed) {
    mavros_msgs::CommandBool cmd;
    cmd.request.value = false;
    disarm_client.call(cmd);
    // ROS_INFO("Quad disarmed.");
  }
}


// PRIVATE - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Callbacks
void Quad::state_callback(const mavros_msgs::State::ConstPtr& msg) {
  data.state.header = msg->header;
  data.state.connected = msg->connected;
  data.state.armed = msg->armed;
  data.state.guided = msg->guided;
  data.state.mode = msg->mode;
}

void Quad::local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.local_pose.header = msg->header;
  data.local_pose.pose = msg->pose;
}

void Quad::wand_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.wand_pose.header = msg->header;
  data.wand_pose.pose = msg->pose;

  past_wand_pose.push(data.wand_pose);
  past_wand_pose.pop();
  set_wand_vel();
}

void Quad::plat_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.plat_pose.header = msg->header;
  data.plat_pose.pose = msg->pose;

  past_plat_pose.push(data.plat_pose);
  past_plat_pose.pop();
  set_plat_vel();
}

void Quad::ball_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.ball_pose.header = msg->header;
  data.ball_pose.pose = msg->pose;

  past_ball_pose.push(data.ball_pose);
  past_ball_pose.pop();
  set_ball_vel();
}

void Quad::local_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  data.local_vel.header = msg->header;
  data.local_vel.twist = msg->twist;
}

// Vel setters
void Quad::set_wand_vel() {
  int size = past_wand_pose.size();
  // Angular velocity fields are all zero and should not be used
  data.wand_vel.header = past_wand_pose.back().header;
  double dx = past_wand_pose.back().pose.position.x -
              past_wand_pose.front().pose.position.x;
  double dy = past_wand_pose.back().pose.position.y -
              past_wand_pose.front().pose.position.y;
  double dz = past_wand_pose.back().pose.position.z -
              past_wand_pose.front().pose.position.z;
  data.wand_vel.twist.linear.x = dx * VEL_T_DENOM;
  data.wand_vel.twist.linear.y = dy * VEL_T_DENOM;
  data.wand_vel.twist.linear.z = dz * VEL_T_DENOM;
}

void Quad::set_plat_vel() {
  data.plat_vel.header = past_plat_pose.back().header;
  double dx = past_plat_pose.back().pose.position.x -
              past_plat_pose.front().pose.position.x;
  double dy = past_plat_pose.back().pose.position.y -
              past_plat_pose.front().pose.position.y;
  double dz = past_plat_pose.back().pose.position.z -
              past_plat_pose.front().pose.position.z;
  data.plat_vel.twist.linear.x = dx * VEL_T_DENOM;
  data.plat_vel.twist.linear.y = dy * VEL_T_DENOM;
  data.plat_vel.twist.linear.z = dz * VEL_T_DENOM;
}

void Quad::set_ball_vel() {
  data.ball_vel.header = past_ball_pose.back().header;
  double dx = past_ball_pose.back().pose.position.x -
              past_ball_pose.front().pose.position.x;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          past_ball_pose.front().pose.position.x;
  double dy = past_ball_pose.back().pose.position.y -
              past_ball_pose.front().pose.position.y;
  double dz = past_ball_pose.back().pose.position.z -
              past_ball_pose.front().pose.position.z;
  data.ball_vel.twist.linear.x = dx * VEL_T_DENOM;
  data.ball_vel.twist.linear.y = dy * VEL_T_DENOM;
  data.ball_vel.twist.linear.z = dz * VEL_T_DENOM;
}

void Quad::check_for_disarm_cmd() {
  if (data.wand_vel.twist.linear.z < -2.0 &&
      std::abs(data.wand_pose.pose.orientation.y) > 0.9 &&
      quad_type == REAL_QUAD) {
    ROS_INFO("disarmed by wand");
    disarm();
  }
}

// FAKE QUAD - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
FakeQuad::FakeQuad(int quad_type_in) : d_angle(0), activated(false) {
  data.node = ros::NodeHandle("fake");
  quad_type = quad_type_in;
  if (quad_type == WAND_PROJECTION) {
    data.local_pose.pose.position.x = -1;
    data.local_pose.pose.position.y = 0;
    data.local_pose.pose.position.z = 0.5;
  }
  if (quad_type == WAND_MOVABLE) {
    data.local_pose.pose.position.x = -1;
    data.local_pose.pose.position.y = 0;
    data.local_pose.pose.position.z = 0.5;
  }
  if (quad_type == CLOCKWISE_CIRCLE) {
    d_angle = 270;
    data.local_pose.pose.position.x = -1;
    data.local_pose.pose.position.y = 0;
    data.local_pose.pose.position.z = 0.5;
  }
  ROS_INFO("FakeQuad created");
  wand_pose_sub = data.node.subscribe("/wand_pose",
                                      FRAMES_PER_SEC,
                                      &FakeQuad::wand_pose_callback,
                                      this);
  plat_pose_sub = data.node.subscribe("/object_pose",
                                      FRAMES_PER_SEC,
                                      &FakeQuad::plat_pose_callback,
                                      this);
  ball_pose_sub = data.node.subscribe("/ball_pose",
                                      FRAMES_PER_SEC,
                                      &FakeQuad::ball_pose_callback,
                                      this);
}

void FakeQuad::set_position(double x, double y, double z) {
  data.local_pose.pose.position.x = x;
  data.local_pose.pose.position.y = y;
  data.local_pose.pose.position.z = z;
}


void FakeQuad::wand_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.wand_pose.header = msg->header;
  data.wand_pose.pose = msg->pose;

  past_wand_pose.push(data.wand_pose);
  past_wand_pose.pop();
  set_wand_vel();
}

void FakeQuad::plat_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.plat_pose.header = msg->header;
  data.plat_pose.pose = msg->pose;

  past_plat_pose.push(data.plat_pose);
  past_plat_pose.pop();
  set_plat_vel();
}

void FakeQuad::ball_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  data.ball_pose.header = msg->header;
  data.ball_pose.pose = msg->pose;

  past_ball_pose.push(data.ball_pose);
  past_ball_pose.pop();
  set_ball_vel();
}


void FakeQuad::run() {
  switch (quad_type) {

    // Project position out past where the wand is pointing
    case WAND_PROJECTION: {
      tf::Quaternion q;
      tf::quaternionMsgToTF(data.wand_pose.pose.orientation, q);
      double roll, pitch, yaw;
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      if (std::abs(roll) > 2.8 && data.wand_pose.pose.position.z > 0.5) {
        double proj_dist = 0.8;
        data.local_pose.pose.position.x =
          (data.wand_pose.pose.position.x - proj_dist * cos(yaw));
        data.local_pose.pose.position.y =
          (data.wand_pose.pose.position.y - proj_dist * sin(yaw));
        data.local_pose.pose.position.z =
          (data.wand_pose.pose.position.z + proj_dist * sin(pitch));
      }
      break;
    }

    // Move according to wand orientation
    case WAND_MOVABLE: {
      if (data.wand_pose.pose.position.z > 0.8) {
        if (std::abs(data.wand_pose.pose.orientation.y) < 0.1 &&
            data.local_pose.pose.position.x <= 1.3) {
          // Increase x
          data.local_pose.pose.position.x += 0.5 / FRAMES_PER_SEC;
        }
        else if (std::abs(data.wand_pose.pose.orientation.y) > 0.9 &&
                 data.local_pose.pose.position.x >= -1.5) {
          // Decrease x
          data.local_pose.pose.position.x -= 0.5 / FRAMES_PER_SEC;
        }
      }
      break;
    }

    // Move in a constant clockwise circle
    // (TODO this currently goes counterclockwise...)
    case CLOCKWISE_CIRCLE: {
      if (!activated) {
        bool rot_check = std::abs(data.wand_pose.pose.orientation.y) < 0.1;
        bool z_check = data.wand_pose.pose.position.z > 0.8;
        activated = rot_check && z_check;
      }
      else {
        double r = 1.0;
        double r_angle = M_PI * d_angle / 180;
        data.local_pose.pose.position.x = r * cos(r_angle);
        data.local_pose.pose.position.y = r * sin(r_angle);

        // Degrees per second
        d_angle += 45 / FRAMES_PER_SEC;
      }
      break;
    }

    default: {
      ROS_WARN_ONCE("FakeQuad type %d not found...", quad_type);
      break;
    }
  }
  ros::spinOnce();
}


// FORMATION - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Formation::Formation() : initialized(false) {}

void Formation::run() {
  for (int i = 0; i < quad_list.size(); ++i) {
    quad_list.at(i)->run();
  }

  if (!initialized) {
    sleep(1.0);
    initialized = true;
  }
  else {
    check_for_collisions();
  }
}

void Formation::add_quad(Quad& quad) {
  for (int i = 0; i < quad_list.size(); ++i) {
    quad_list.at(i)->add_quad(&quad);
    quad.add_quad(quad_list.at(i));
  }
  quad.add_quad(&quad);
  quad_list.push_back(&quad);
}

void Formation::print_count() {
  int size = quad_list.size();
  ROS_INFO("num of quads: %d", size);
}

void Formation::check_for_collisions() {
  for (int i = 0; i < quad_list.size(); ++i) {
    if (quad_list.at(i)->isReal()) {

      // Check if i is near another quad
      for (int j = i + 1; j < quad_list.size(); ++j) {
        if (pose_dist_check(quad_list.at(i)->get_local_pose().pose,
                            quad_list.at(j)->get_local_pose().pose,
                            0.6, 1.0)) {
          quad_list.at(i)->disarm();
          quad_list.at(j)->disarm();
          ROS_INFO("Too close");
        }
      }

      // Check if i is near the boundaries
      if (!insideBoundaries(quad_list.at(i)->get_local_pose().pose)) {
        quad_list.at(i)->disarm();
        // ROS_INFO("Outside boundaries");
      }

    }
  }
}

bool insideBoundaries(geometry_msgs::Pose pose) {
  return pose.position.x > -2.05 && pose.position.x < 2.00 &&
         pose.position.y > -1.60 && pose.position.y < 1.60 &&
         pose.position.z < 2.00;
}
