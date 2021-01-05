#include "mocap_optitrack/latlon_conversions.h"

// Constants for lat/long calculations
// baud 115200 for odroid
namespace {
  const int radius = 6378137;
  const double pi = 3.141592653589793;
  const double baseLat = 41.3918;
  const double baseLon = -73.9625;
}

sensor_msgs::NavSatFix xyz_to_fix(double dX, double dY, double dZ) {

  // Calculate lat/lon
  // North defined as away from door and alt in meters above reference
  double dNorth = dZ;
  double dEast = dX;
  double dAlt = -dY;

  double dLat = dNorth / radius;
  double dLon = dEast / (radius * cos(pi * baseLat / 180));

  // Create and fill out fix data
  sensor_msgs::NavSatFix fix;
  // fix.header.seq = seq++; // set by publisher
  fix.header.stamp = ros::Time::now();
  fix.header.frame_id = "fcu";    // TODO publish a custom transform; default = fcu

  // status: -1 = nofix; 0 = fix; 1 = satbased aug; 2 = groundbased aug
  // service: 1 = gps; 2 = glonass; 4 = compass; 8 = galileo
  fix.status.status = 2;
  fix.status.service = 1;

  fix.latitude = baseLat + dLat;
  fix.longitude = baseLon + dLon;
  fix.altitude = dAlt;

  // TODO ???? unsure if this estimates 0 DOP, ask someone who knows
  for (int i = 0; i < 9; ++i) {
    fix.position_covariance[i] = 0;
  }

  // UNKNOWN = 0; APPROXIMATED = 1; DIAGONAL KNOWN = 2; KNOWN = 3;
  fix.position_covariance_type = 1;

  return fix;
}

geometry_msgs::TwistStamped get_vel(double dX, double dY, double dZ,
                                    double& prevN, double& prevE, double& prevA,
                                    int rate) {
  double dNorth = dZ;
  double dEast = dX;
  double dAlt = -dY;

  geometry_msgs::TwistStamped vel;
  vel.header.stamp = ros::Time::now();
  vel.header.frame_id = "fcu";
  vel.twist.linear.x = (dEast - prevE) / rate;
  vel.twist.linear.y = (dNorth - prevN) / rate;
  vel.twist.linear.z = (dAlt - prevA) / rate;

  prevN = dNorth;
  prevE = dEast;
  prevA = dAlt;

  return vel;
}

// Below this line: currently UNUSED/DEPRECATED/UNFINISHED
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
std::string to_string(double n) {
  std::stringstream ss;
  ss << n;
  return ss.str();
}

std::string dec_to_dms(double lat) {
  std::string dms;
  dms += to_string(floor(lat));

  double minutes = 60 * (lat - floor(lat));
  dms += to_string(floor(minutes));

  double seconds = 60 * (minutes - floor(minutes));
  dms += to_string(seconds);

  return dms;
}

std::string xyz_to_nmea(double dX, double dY, double dZ) {
  double dNorth = dZ;
  double dEast = dX;
  double dAlt = -dY;

  double dLat = dNorth / radius;
  double dLon = dEast / (radius * cos(pi * baseLat / 180));

  return latlon_to_nmea(baseLat + dLat, baseLon + dLon, dAlt);
}

std::string latlon_to_nmea(double lat, double lon, double dAlt) {
  // Using HIL_GPS fields
  ros::Time now = ros::Time::now();
  uint64_t time_usec = now.sec * 1000000 + now.nsec * 1000;
  uint8_t fix_type = 3;
  uint32_t lat_int = (uint32_t) lat * 10000000;
  uint32_t lon_int = (uint32_t) lon * 10000000;
  uint32_t alt = dAlt * 1000;
  uint16_t eph = 1;
  uint16_t epv = 1;
  uint16_t vel = 65535;
  int16_t vn = 0;
  int16_t ve = 0;
  int16_t vd = 0;
  uint16_t cog = 65535;
  uint8_t satellites_visible = 8;


  std::string lat_str = dec_to_dms(lat);
  std::string lon_str = dec_to_dms(lon);

  ROS_INFO("lat_str: %s", lat_str.c_str());
  ROS_INFO("lon_str: %s", lon_str.c_str());

  std::string nmea;
}
