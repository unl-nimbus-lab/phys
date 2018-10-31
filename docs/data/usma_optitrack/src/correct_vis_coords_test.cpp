#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <std_msgs/String.h>

#include "mocap_optitrack/constants_config.h"

std::string ns = "/testing";

class CorrectCoords {
public:
  CorrectCoords(std::string subtopic, std::string pubtopic)
 : rate(FRAMES_PER_SEC * 1.1) {
    pub = node.advertise<geometry_msgs::PoseStamped>(pubtopic, 120);
    sub = node.subscribe(subtopic, 120, &CorrectCoords::callback, this);
  }

  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    fixedPose.header = msg->header;
    fixedPose.pose = msg->pose;

    // CALIBRATIONS HERE - - - - - - - - - - - - - - - - -
    // - - - - - - - - - - - - - - - - - - - - - - - - - -
    fixedPose.pose.position.x = msg->pose.position.y;
    fixedPose.pose.position.y = -msg->pose.position.x;

    // Most likely don't have to touch these
    fixedPose.pose.orientation.z = msg->pose.orientation.w;
    fixedPose.pose.orientation.w = msg->pose.orientation.z;
  }

  void publish_topic() {
    pub.publish(fixedPose);
    ros::spinOnce();
    rate.sleep();
  }

private:
  ros::NodeHandle node;
  ros::Rate rate;
  ros::Publisher pub;
  ros::Subscriber sub;

  geometry_msgs::PoseStamped fixedPose;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_and_publish");

  CorrectCoords correct(argv[1], argv[2]);

  while(ros::ok()) {
    correct.publish_topic();
  }

  return 0;
}
