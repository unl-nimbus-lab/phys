#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cliff_print");
    ros::NodeHandle nh;
    ROS_INFO("Cliff sensor printing node started...");

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("irobot/cmd_vel", 1);
    std::mt19937 generator;
    generator.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> distribution;
    std::uniform_int_distribution<int>::param_type range {0, 2};
    double vel_primary = 0.5;
    double vel_secondary = 0.25;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        int action = distribution(generator, range);
        geometry_msgs::Twist msg;
        switch (action) {
            case 0:
                //forward
                msg.linear.x = vel_primary;
                msg.angular.z = vel_secondary;
                break;
            case 1:
                //left
                msg.angular.z = vel_primary;
                msg.linear.x = vel_secondary;
                break;
            case 2:
                //right
                msg.angular.z = -vel_primary;
                msg.linear.x = vel_secondary;
                break;
            default:
                //oops
                msg.linear.x = vel_primary;
                msg.angular.z = vel_secondary;
                break;
        }
        pub_vel.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
