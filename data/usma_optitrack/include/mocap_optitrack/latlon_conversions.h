#ifndef __LATLON_CONVERSIONS_H__
#define __LATLON_CONVERSIONS_H__

#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <string>
#include <sstream>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>

// Converts dx, dy, dz into a NavSatFix for global_position/global or
// global_position/raw/fix
sensor_msgs::NavSatFix xyz_to_fix(double dX, double dY, double dZ);

geometry_msgs::TwistStamped get_vel(double dX, double dY, double dZ,
                                    double& prevN, double& prevE, double& prevA, 
                                    int rate);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Currently UNUSED/DEPRECATED/UNFINISHED

std::string to_string(double n);

std::string dec_to_dms(double lat);

std::string xyz_to_nmea(double dX, double dY, double dZ);

std::string latlon_to_nmea(double lat, double lon, double dAlt);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#endif
