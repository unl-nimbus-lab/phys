//
// Created by carlh on 12/6/16.
//

#include <sparki_set_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <PID.h>

using namespace SparkiControl;

static PID pid(0.01, 0.005, 0.00005);

void SparkiSetPose::poseCallback(const geometry_msgs::PoseStampedConstPtr& poseStamped) {
    ROS_INFO("Updating Pose\n");
    geometry_msgs::Quaternion orient = poseStamped->pose.orientation;
    tf2::Quaternion orientation(orient.x, orient.y, orient.z, orient.w);
    geometry_msgs::TransformStamped transform;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::string sparkiName = m_node->getNamespace().substr(2); // Need to strip leading //
    ros::Rate rate(100);
    while (m_node->ok()) {
        try {
            transform = tfBuffer.lookupTransform(sparkiName, "table_map", ros::Time(0));
            ROS_INFO("Found transformation from %s to table_map", sparkiName.c_str());
            break;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            rate.sleep();
            continue;
        }
    }


    // TODO FUTURE WORK: Create a twist message that can be converted to send an arcadeDrive signal to the Sparki

    geometry_msgs::TwistStamped twist;
    twist.twist.angular.x = orientation.getX();
    twist.twist.angular.y = orientation.getY();
    twist.twist.angular.z = orientation.getZ();
    twist.twist.linear.x = poseStamped->pose.position.x;
    twist.twist.linear.y = poseStamped->pose.position.y;
    twist.twist.linear.z = poseStamped->pose.position.z;
    twist.header.stamp = ros::Time::now();
    geometry_msgs::TwistStampedConstPtr ptr(new geometry_msgs::TwistStamped(twist));
    publishMessage(ptr);
}

SparkiSetPose::SparkiSetPose() {
    ROS_INFO("Initializing SparkiSetPose");
    m_counter = 0;
    m_node = new NodeHandle();
    ConnectSubscribers();
    AdvertisePublishers();
    ROS_INFO("SparkiSetPose is ready");
}

SparkiSetPose::~SparkiSetPose() {
    ROS_INFO("Destroying the PID");
    DisconnectSubscribers(); // Probably don't need this?  I think it'll automatically unsubscribe when this goes out of scope.
    UnAdvertisePublishers();
    m_node->shutdown();
}

void SparkiSetPose::ConnectSubscribers() {
    m_poseSubscriber = m_node->subscribe<geometry_msgs::PoseStamped>("set_pose", 10, &SparkiSetPose::poseCallback, this);
}

void SparkiSetPose::DisconnectSubscribers() {
    m_poseSubscriber.shutdown();
}

void SparkiSetPose::AdvertisePublishers() {
    m_publisher = m_node->advertise<geometry_msgs::TwistStamped>("cmd_vel", 10);
}

void SparkiSetPose::UnAdvertisePublishers() {
    m_publisher.shutdown();
}

void SparkiSetPose::publishMessage(geometry_msgs::TwistStampedConstPtr twist) {
    if (m_node->ok()) {
        ROS_INFO("Publishing twist: %i", m_counter++);
        m_publisher.publish(twist);
    }
}