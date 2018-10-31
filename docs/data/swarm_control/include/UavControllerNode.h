/**
 *  This header file defines the UavControllerNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 30/10/2016
 *  Modified on: 30/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _UAV_CONTROLLER_NODE_H_
#define _UAV_CONTROLLER_NODE_H_

#include <string>
#include "Node.h"
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>

#define FACTOR  60.0

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

class UavControllerNode : public Node
{
public:
  UavControllerNode(ros::NodeHandle *nh);
  virtual ~UavControllerNode();

private:
  virtual void controlLoop();

  // Position of elements in the system
  geometry_msgs::Point32 position_;
  geometry_msgs::Point32 migrationPoint_;
  geometry_msgs::Point32 lastMP_;
  double lastMPVelX_, lastMPVelY_;

  // Drone commands
  double roll_;
  double pitch_;

  // Flight mode
  std::string mode_;
  bool guided_;
  bool armed_;

  // ROS objects
  ros::Subscriber mavros_state_sub_;    // Subscriber to flight mode
  ros::Subscriber migration_point_sub_; // Subscriber to migration point
  ros::Subscriber odom_sub_;            // Subscriber to odometry
  ros::Publisher rc_override_pub_;      // RC publisher
  ros::Time lastTime_;                  // Timer

  // Member functions
  void mavrosStateCb( const mavros_msgs::StateConstPtr &msg );
  void migrationPointCb( const geometry_msgs::Point32ConstPtr &msg );
  void odomCb( const nav_msgs::OdometryConstPtr &msg );
  void publishRCOverride();

};

#endif // _UAV_CONTROLLER_NODE_H_
