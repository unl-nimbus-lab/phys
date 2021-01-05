#ifndef MakeMarker_h
#define MakeMarker_h

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <math.h>

void makeMarker(ros::Publisher markerViz, geometry_msgs::PoseStamped pos, float r, float g, float b);
void makeMarker(ros::Publisher markerViz, int id, float x, float y, float r, float g, float b);
void makeMarker(ros::Publisher markerViz, int id, float x, float y, float r, float g, float b, float a);
void makeMarkerGlobal(ros::Publisher markerViz, int id, float x, float y, float r, float g, float b, float a);
void makeMarker(ros::Publisher markerViz, int id, float x, float y, float r, float g, float b, int shape);
#endif
