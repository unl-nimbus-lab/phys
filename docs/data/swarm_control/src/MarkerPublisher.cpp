/**
 *  This source file implements the MarkerPublisher class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 01/03/2017
 *  Modified on: 01/03/2017
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "MarkerPublisher.h"


MarkerPublisher::MarkerPublisher(ros::NodeHandle *nh)
  : Node(nh, 10)
{
    // Try to get UAV ID from the parameter server
    // TODO: Handle the case where the parameter doesn't exist
    if ( !ros::param::get("swarm_controller_node/uav_id", id_) )
    {
        id_ = 0;
    }

    v1_sub_ = nh->subscribe("v1", 1, &MarkerPublisher::v1Cb, this);
    v2_sub_ = nh->subscribe("v2", 1, &MarkerPublisher::v2Cb, this);
    v3_sub_ = nh->subscribe("v3", 1, &MarkerPublisher::v3Cb, this);
    v4_sub_ = nh->subscribe("v4", 1, &MarkerPublisher::v4Cb, this);
    vRes_sub_ = nh->subscribe("vRes", 1, &MarkerPublisher::vResCb, this);

    marker_pub_ = nh->advertise<visualization_msgs::Marker>("/vector_markers", 10);
}

MarkerPublisher::~MarkerPublisher()
{
    v1_sub_.shutdown();
    v2_sub_.shutdown();
    v3_sub_.shutdown();
    v4_sub_.shutdown();
    vRes_sub_.shutdown();
    marker_pub_.shutdown();
}

void MarkerPublisher::controlLoop()
{
    // Create some strings that will be useful later
    std::string base_link_frame, markers_ns;
    std::stringstream ss1, ss2;
    ss1 << "/uav" << id_ << "/base_link";
    ss2 << "vectors_" << id_;
    base_link_frame = ss1.str();
    markers_ns = ss2.str();

    // Get the most recent transform available from world frame to UAV body frame
    tf::StampedTransform transform;
    try{
      pose_lst_.lookupTransform("/world", base_link_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Create 1 Marker msg for each vector
    visualization_msgs::Marker v1Marker, v2Marker, v3Marker, v4Marker, vResMarker;

    // Initialize the Markers with shared data:
    // frame_id = the arrows will be positioned relative to the world frame
    // stamp = timestamp in which the marker is published
    // action = ADD means create new marker, past markers will be replaced
    // pose.orientation.w = initialize orientation quaternion to ( 0, 0, 0, 1 )
    // type = these Markers will be arrows
    // ns = namespace, differentiate between each UAV markers
    v1Marker.header.frame_id = v2Marker.header.frame_id = v3Marker.header.frame_id =
            v4Marker.header.frame_id = vResMarker.header.frame_id = "world";
    v1Marker.header.stamp = v2Marker.header.stamp = v3Marker.header.stamp =
            v4Marker.header.stamp = vResMarker.header.stamp = ros::Time::now();
    v1Marker.action = v2Marker.action = v3Marker.action =
            v4Marker.action = vResMarker.action = visualization_msgs::Marker::ADD;
    v1Marker.pose.orientation.w = v2Marker.pose.orientation.w = v3Marker.pose.orientation.w =
            v4Marker.pose.orientation.w = vResMarker.pose.orientation.w = 1;
    v1Marker.type = v2Marker.type = v3Marker.type =
            v4Marker.type = vResMarker.type = visualization_msgs::Marker::ARROW;
    v1Marker.ns = v2Marker.ns = v3Marker.ns = v4Marker.ns = vResMarker.ns = markers_ns;

    // IDs differentiate between each marker under the same namespace
    v1Marker.id = 1;
    v2Marker.id = 2;
    v3Marker.id = 3;
    v4Marker.id = 4;
    vResMarker.id = 0;

    // Pose = where RViz shall render the marker.
    // For the position, translate the markers to the UAV body frame
    v1Marker.pose.position.x = v2Marker.pose.position.x = v3Marker.pose.position.x =
            v4Marker.pose.position.x = vResMarker.pose.position.x = transform.getOrigin().x();
    v1Marker.pose.position.y = v2Marker.pose.position.y = v3Marker.pose.position.y =
            v4Marker.pose.position.y = vResMarker.pose.position.y = transform.getOrigin().y();
    v1Marker.pose.position.z = v2Marker.pose.position.z = v3Marker.pose.position.z =
            v4Marker.pose.position.z = vResMarker.pose.position.z = transform.getOrigin().z();
    // For the orientation, only the yaw axis rotation is important and is calculated using the x and y of the vectors
    tf::Quaternion q;
    q.setRPY( 0.0, 0.0, atan2( v1_.y, v1_.x ) );
    v1Marker.pose.orientation.x = q.getX();
    v1Marker.pose.orientation.y = q.getY();
    v1Marker.pose.orientation.z = q.getZ();
    v1Marker.pose.orientation.w = q.getW();
   q.setRPY( 0.0, 0.0, atan2( v2_.y, v2_.x ) );
    v2Marker.pose.orientation.x = q.getX();
    v2Marker.pose.orientation.y = q.getY();
    v2Marker.pose.orientation.z = q.getZ();
    v2Marker.pose.orientation.w = q.getW();
    q.setRPY( 0.0, 0.0, atan2( v3_.y, v3_.x ) );
    v3Marker.pose.orientation.x = q.getX();
    v3Marker.pose.orientation.y = q.getY();
    v3Marker.pose.orientation.z = q.getZ();
    v3Marker.pose.orientation.w = q.getW();
    q.setRPY( 0.0, 0.0, atan2( v4_.y, v4_.x ) );
    v4Marker.pose.orientation.x = q.getX();
    v4Marker.pose.orientation.y = q.getY();
    v4Marker.pose.orientation.z = q.getZ();
    v4Marker.pose.orientation.w = q.getW();
    q.setRPY( 0.0, 0.0, atan2( vRes_.y, vRes_.x ) );
    vResMarker.pose.orientation.x = q.getX();
    vResMarker.pose.orientation.y = q.getY();
    vResMarker.pose.orientation.z = q.getZ();
    vResMarker.pose.orientation.w = q.getW();

    // Scale = the size of the marker. X is the size of the arrow. Y and Z are the radius of the arrow's body
    v1Marker.scale.x = sqrt( v1_.x * v1_.x + v1_.y * v1_.y + v1_.z * v1_.z );
    v2Marker.scale.x = sqrt( v2_.x * v2_.x + v2_.y * v2_.y + v2_.z * v2_.z );
    v3Marker.scale.x = sqrt( v3_.x * v3_.x + v3_.y * v3_.y + v3_.z * v3_.z );
    v4Marker.scale.x = sqrt( v4_.x * v4_.x + v4_.y * v4_.y + v4_.z * v4_.z );
    vResMarker.scale.x = sqrt( vRes_.x * vRes_.x + vRes_.y * vRes_.y + vRes_.z * vRes_.z );
    v1Marker.scale.y = v2Marker.scale.y = v3Marker.scale.y = v4Marker.scale.y = vResMarker.scale.y = 0.02;
    v1Marker.scale.z = v2Marker.scale.z = v3Marker.scale.z = v4Marker.scale.z = vResMarker.scale.z = 0.02;

    // color = the color of each marker
    v1Marker.color.r = 0.84;
    v1Marker.color.g = 0.0;
    v1Marker.color.b = 0.2;
    v2Marker.color.r = 0.16;
    v2Marker.color.g = 0.69;
    v2Marker.color.b = 0.79;
    v3Marker.color.r = 0.69;
    v3Marker.color.g = 0.31;
    v3Marker.color.b = 0.71;
    v4Marker.color.r = 1.0;
    v4Marker.color.g = 0.9;
    v4Marker.color.b = 0.0;
    vResMarker.color.r = 0.57;
    vResMarker.color.g = 0.71;
    vResMarker.color.b = 0.35;
    v1Marker.color.a = v2Marker.color.a = v3Marker.color.a = v4Marker.color.a = vResMarker.color.a = 1.0;

    // Publish the markers
    marker_pub_.publish(v1Marker);
    marker_pub_.publish(v2Marker);
    marker_pub_.publish(v3Marker);
    marker_pub_.publish(v4Marker);
    marker_pub_.publish(vResMarker);
}

void MarkerPublisher::v1Cb(const geometry_msgs::PointConstPtr &msg)
{
    v1_.x = msg->x;
    v1_.y = msg->y;
    v1_.z = msg->z;
}

void MarkerPublisher::v2Cb(const geometry_msgs::PointConstPtr &msg)
{
    v2_.x = msg->x;
    v2_.y = msg->y;
    v2_.z = msg->z;
}

void MarkerPublisher::v3Cb(const geometry_msgs::PointConstPtr &msg)
{
    v3_.x = msg->x;
    v3_.y = msg->y;
    v3_.z = msg->z;
}

void MarkerPublisher::v4Cb(const geometry_msgs::PointConstPtr &msg)
{
    v4_.x = msg->x;
    v4_.y = msg->y;
    v4_.z = msg->z;
}

void MarkerPublisher::vResCb(const geometry_msgs::PointConstPtr &msg)
{
    vRes_.x = msg->x;
    vRes_.y = msg->y;
    vRes_.z = msg->z;
}
