/**
 *  This header file defines the SwarmControllerNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 30/10/2016
 *  Modified on: 30/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _SWARM_CONTROLLER_NODE_H_
#define _SWARM_CONTROLLER_NODE_H_

#include <string>

#include "Node.h"
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <swarm_control/OdometryWithUavId.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define PI 3.14159265

#define MAXVEL   5.0

#define VISION_DISTANCE 100.0

class SwarmControllerNode : public Node
{
public:
  SwarmControllerNode(ros::NodeHandle *nh);
  virtual ~SwarmControllerNode();

private:
  virtual void controlLoop();

  // UAV id
  int id_;

  // Differences from current position to origin
  double dx_;
  double dy_;
  bool initialDeltasCalculated_;

  // Enable control
  bool enableControl_;

  // Rules weights
  double r1_;
  double r2_;
  double r3_;
  double r4_;

  // Position of elements in the system
  nav_msgs::Odometry odom_;
  geometry_msgs::Point migrationPoint_;
  std::vector<swarm_control::OdometryWithUavId> neighbors_;

  // Flight mode
  std::string mode_;
  bool guided_;
  bool armed_;

  // ROS objects
  ros::Subscriber mavros_state_sub_;    // Subscriber to flight mode
  ros::Subscriber migration_point_sub_; // Subscriber to migration point
  ros::Subscriber global_position_sub_; // Subscriber to global position
  ros::Subscriber odom_sub_;            // Subscriber to odometry
  ros::Subscriber enable_control_sub_;  // Subscriber to enable or disable control
  ros::Subscriber uavs_odom_sub_;       // Subscriber to all UAVs odometry
  ros::Publisher uav_odom_pub_;         // UAV odom publisher
  ros::Publisher cmd_vel_pub_;          // Velocity publisher
  tf::TransformBroadcaster pose_br_;

  ros::Publisher v1_pub_;
  ros::Publisher v2_pub_;
  ros::Publisher v3_pub_;
  ros::Publisher v4_pub_;
  ros::Publisher vRes_pub_;

  // Member functions
  void mavrosStateCb( const mavros_msgs::StateConstPtr &msg );
  void migrationPointCb( const geometry_msgs::PointConstPtr &msg );
  void globalPositionCb( const sensor_msgs::NavSatFix &msg );
  void odomCb( const nav_msgs::OdometryConstPtr &msg );
  void enableControlCb( const std_msgs::BoolConstPtr &msg );
  void uavsOdomCb( const swarm_control::OdometryWithUavIdConstPtr &msg );
  void publishUavOdom ();
  void publishVelocity( double velX, double velY );

  // Rules
  geometry_msgs::Point rule1();
  geometry_msgs::Point rule2();
  geometry_msgs::Point rule3();
  geometry_msgs::Point rule4();


  // Utility functions
  double haversines( double lat1, double lon1, double lat2, double lon2 );

};

#endif // _SWARM_CONTROLLER_NODE_H_
