#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#define LAT_T 1.000090417370
#define LAT_B 0.999909582630

#define LON_L 0.999910174320
#define LON_R 1.000089826270

#define MAX_XY 10.0
#define MIN_XY -10.0


void cb_gps(const sensor_msgs::NavSatFixConstPtr& msg);
ros::Publisher pub_gps_trans;

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_to_zero_translate");
    ros::NodeHandle nh;
    ROS_INFO("Starting node to translate gps coordinates to zero...");

    ros::Subscriber sub_gps = nh.subscribe("irobot/gps_data", 1, cb_gps);
    pub_gps_trans = nh.advertise<nav_msgs::Odometry>("irobot/gps_odom", 1);

    ros::spin();
}

void cb_gps(const sensor_msgs::NavSatFixConstPtr& msg) {
    double x_percent = (msg->longitude - LAT_T) / (LAT_B - LAT_T);
    double x = -1 * (x_percent * (MAX_XY - MIN_XY) + MIN_XY);
    double y_percent = (msg->latitude - LON_L) / (LON_R - LON_L);
    double y = 1 * (y_percent * (MAX_XY - MIN_XY) + MIN_XY);
    //ROS_INFO("Lat: %f", msg->latitude);
    //ROS_INFO("Lon: %f", msg->longitude);
    //ROS_INFO("GPS LP: %f Y: %f", y_percent, y);
    //ROS_INFO("GPS LP: %f X: %f", x_percent, x);
    //

    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = msg->header.stamp;
    msg_odom.header.frame_id = "odom";
    msg_odom.child_frame_id = "base_link";

    msg_odom.pose.pose.position.x = x;
    msg_odom.pose.pose.position.y = y;
    msg_odom.pose.pose.position.z = msg->altitude;
    msg_odom.pose.pose.orientation.w = 0;
    msg_odom.pose.pose.orientation.x = 1;
    msg_odom.pose.pose.orientation.y = 0;
    msg_odom.pose.pose.orientation.z = 0;

    double cov[] = {0.000075, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.000075, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 500, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 99999, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 99999, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 99999};
    for (int i = 0; i < 36; i++) {
        msg_odom.pose.covariance[i] = cov[i];
    }


    pub_gps_trans.publish(msg_odom);
}