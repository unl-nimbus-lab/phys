#ifndef __CONSTANTS_CONFIG_H__
#define __CONSTANTS_CONFIG_H__

const double FRAMES_PER_SEC = 40;

// ~calcs per sec but continuous
// 1/duration of dt for velocity calculations
const double VEL_T_DENOM = 10;

const std::string QUAD_POSE_TOPIC = "mavros/local_position/pose";
const std::string SET_POSE_TOPIC = "mavros/setpoint_position/local";

// Offsets for calibration of landings
const double Z_VEL_OFFSET = -0.03;  // in m/s
const double Z_POS_OFFSET = 0.00;    // in meters

#endif
