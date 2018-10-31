#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <std_msgs/String.h>

class CorrectCoords {
public:
  CorrectCoords(std::string subtopic, std::string pubtopic) {
    pub = node.advertise<geometry_msgs::PoseStamped>(pubtopic, 120);
    sub = node.subscribe(subtopic, 120, &CorrectCoords::callback, this);
  }

  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Don't touch this
    geometry_msgs::PoseStamped fixedPose;
    fixedPose.header = msg->header;
    fixedPose.pose = msg->pose;

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // CALIBRATIONS HERE

    fixedPose.pose.position.x = msg->pose.position.y;
    fixedPose.pose.position.y = -msg->pose.position.x;

    // Most likely don't have to touch these
    fixedPose.pose.orientation.z = msg->pose.orientation.w;
    fixedPose.pose.orientation.w = msg->pose.orientation.z;

    //
    // - - - - - - - - - - - - - - - - - - - - - - - -




    pub.publish(fixedPose);
  }

private:
  ros::NodeHandle node;
  ros::Publisher pub;
  ros::Subscriber sub;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_and_publish");

  CorrectCoords correct(argv[1], argv[2]);

  ros::spin();

  return 0;
}
