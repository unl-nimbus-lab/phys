#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace {
  std::string rigid_body_name;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

int main(int argc, char** argv) {
  ros::init(argc, argv, "broadcast_rigid_transform");
  ros::NodeHandle node;
  rigid_body_name = argv[1];

  ros::Subscriber sp_sub = node.subscribe(rigid_body_name + "/pose",
                                          10,
                                          &poseCallback);
  ros::spin();
  return 0;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->pose.position.x,
                                  msg->pose.position.y,
                                  msg->pose.position.z));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,
                                         ros::Time::now(),
                                         "world",
                                         rigid_body_name + "/tf"));
}
