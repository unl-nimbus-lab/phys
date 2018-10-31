#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "setpoint_node");
  ros::NodeHandle node;

  ros::Rate rate(10.0);
  ros::Publisher pub =
    node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 5);

  geometry_msgs::PoseStamped setpt;
  setpt.header.stamp = ros::Time::now();
  setpt.header.frame_id = "fcu";
  setpt.pose.position.x = std::atof(argv[1]);
  setpt.pose.position.y = std::atof(argv[2]);
  setpt.pose.position.z = std::atof(argv[3]);

  setpt.pose.orientation.x = std::atof(argv[4]);
  setpt.pose.orientation.y = std::atof(argv[5]);
  setpt.pose.orientation.z = std::atof(argv[6]);
  setpt.pose.orientation.w = std::atof(argv[7]);

  sleep(1.0);
  while (node.ok()) {
    pub.publish(setpt);
    ros::spinOnce();
    rate.sleep();
  }

}
