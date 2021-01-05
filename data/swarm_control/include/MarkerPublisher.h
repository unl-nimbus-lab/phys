/**
 *  This header file defines the MarkerPublisher class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 01/03/2017
 *  Modified on: 01/03/2017
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _MARKER_PUBLISHER_H_
#define _MARKER_PUBLISHER_H_

#include <sstream>
#include "Node.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

class MarkerPublisher : public Node
{
public:
  MarkerPublisher(ros::NodeHandle *nh);
  virtual ~MarkerPublisher();

private:
  virtual void controlLoop();

  // UAV id
  int id_;

  // UAV pose and each vector will be saved as data members
  geometry_msgs::Pose pose_;
  geometry_msgs::Point v1_;
  geometry_msgs::Point v2_;
  geometry_msgs::Point v3_;
  geometry_msgs::Point v4_;
  geometry_msgs::Point vRes_;

  // ROS objects
  ros::Subscriber v1_sub_;      // Subscriber to v1
  ros::Subscriber v2_sub_;      // Subscriber to v2
  ros::Subscriber v3_sub_;      // Subscriber to v3
  ros::Subscriber v4_sub_;      // Subscriber to v4
  ros::Subscriber vRes_sub_;    // Subscriber to vRes
  ros::Publisher marker_pub_;   // Publish markers representing the vectors to RViz

  // TF transform listener to get UAV's pose published in tf
  tf::TransformListener pose_lst_;

  // Member functions
  void v1Cb( const geometry_msgs::PointConstPtr &msg );
  void v2Cb( const geometry_msgs::PointConstPtr &msg );
  void v3Cb( const geometry_msgs::PointConstPtr &msg );
  void v4Cb( const geometry_msgs::PointConstPtr &msg );
  void vResCb( const geometry_msgs::PointConstPtr &msg );

};

#endif // _MARKER_PUBLISHER_H_
