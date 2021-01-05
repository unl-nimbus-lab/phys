#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

// #include <mavlink/common/mavlink_msg_hil_gps.h>

#include "mocap_optitrack/latlon_conversions.h"

#include <cmath>
#include <string>

namespace {
  double prevN = 0;
  double prevE = 0;
  double prevA = 0;
}

int main(int argc, char** argv){

  std::string rigid1_id = argv[1];
  std::string rigid2_id = argv[2];

  ros::init(argc, argv, "transform_to_coords");
  ros::NodeHandle node;

  // Get transform between rigids
  tf::TransformListener listener;
  ros::Rate rate(20.0);

  ros::Publisher gps_global_pub =
    node.advertise<sensor_msgs::NavSatFix>("/mavros/global_position/global", 5);
  ros::Publisher gps_rawfix_pub =
    node.advertise<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 5);
  ros::Publisher gp_vel =
    node.advertise<geometry_msgs::TwistStamped>("/mavros/global_position/gp_vel", 5);
  ros::Publisher gp_rel_alt =
    node.advertise<std_msgs::Float64>("/mavros/global_position/rel_alt", 5);
  ros::Publisher gp_compass_hdg =
    node.advertise<std_msgs::Float64>("/mavros/global_position/compass_hdg", 5);
  ros::Publisher gp_rawvel =
    node.advertise<geometry_msgs::TwistStamped>("/mavros/global_position/raw/gps_vel", 5);

  sleep(1.0);
  while(node.ok()) {

    // ROS_INFO("check");

    tf::StampedTransform transform;
    try{
      listener.lookupTransform(rigid1_id + "/tf", rigid2_id + "/tf",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException& exc) {
      ROS_ERROR("%s", exc.what());
      ros::Duration(1).sleep();
      continue;
    }

    double dX = transform.getOrigin().x();
    double dY = transform.getOrigin().y();
    double dZ = transform.getOrigin().z();
    sensor_msgs::NavSatFix fakeFix = xyz_to_fix(dX, dY, dZ);

    geometry_msgs::TwistStamped fakeVel = get_vel(dX, dY, dZ,
                                                  prevN, prevE, prevA, 20);

    // Publish data
    // TODO calibrate compass to align with fake coords
    gps_global_pub.publish(fakeFix);
    gps_rawfix_pub.publish(fakeFix);
    gp_rawvel.publish(fakeVel);

    ros::spinOnce();

    rate.sleep();

  }
  return 0;
}
