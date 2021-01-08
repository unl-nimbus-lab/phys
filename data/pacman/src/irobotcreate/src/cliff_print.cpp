#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

//Node for debugging cliff sensors

void print_sensors();
void cb_l(const sensor_msgs::LaserScan::ConstPtr &msg);
void cb_fl(const sensor_msgs::LaserScan::ConstPtr &msg);
void cb_fr(const sensor_msgs::LaserScan::ConstPtr &msg);
void cb_r(const sensor_msgs::LaserScan::ConstPtr &msg);

sensor_msgs::LaserScan msg_l;
sensor_msgs::LaserScan msg_fl;
sensor_msgs::LaserScan msg_fr;
sensor_msgs::LaserScan msg_r;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cliff_print");
    ros::NodeHandle nh;
    ROS_INFO("Cliff sensor printing node started...");

    ros::Subscriber sub_l = nh.subscribe("/irobot/left_cliff_scan", 1, cb_l);
    ros::Subscriber sub_fl = nh.subscribe("/irobot/left_front_cliff_scan", 1, cb_fl);
    ros::Subscriber sub_fr = nh.subscribe("/irobot/right_front_cliff_scan", 1, cb_fr);
    ros::Subscriber sub_r = nh.subscribe("/irobot/right_cliff_scan", 1, cb_r);

    while(sub_l.getNumPublishers() == 0 ||
            sub_fl.getNumPublishers() == 0 ||
            sub_fr.getNumPublishers() == 0 ||
            sub_r.getNumPublishers() == 0) {
        //wait on publishers to connect
        ROS_INFO("Waiting on publishers...");
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        print_sensors();
    }
}

void print_sensors() {
    ROS_INFO("-");
    ROS_INFO("Cliff Sensors:");
    if (msg_l.ranges.size() > 0) {
        ROS_INFO(" L : %f\n", msg_l.ranges[0]);
    }
    if (msg_fl.ranges.size() > 0) {
        ROS_INFO(" FL: %f\n", msg_fl.ranges[0]);
    }
    if (msg_fr.ranges.size() > 0) {
        ROS_INFO(" R : %f\n", msg_fr.ranges[0]);
    }
    if (msg_r.ranges.size() > 0) {
        ROS_INFO(" FR: %f\n", msg_r.ranges[0]);
    }
}

void cb_l(const sensor_msgs::LaserScan::ConstPtr &msg) {
    msg_l = *msg;
}

void cb_fl(const sensor_msgs::LaserScan::ConstPtr &msg) {
    msg_fl = *msg;
}

void cb_fr(const sensor_msgs::LaserScan::ConstPtr &msg) {
    msg_fr = *msg;
}

void cb_r(const sensor_msgs::LaserScan::ConstPtr &msg) {
    msg_r = *msg;
}
