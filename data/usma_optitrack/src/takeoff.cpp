#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class Takeoff {
public:
  Takeoff(double height_in) : takeoff_height(height_in), completed(false) {
    pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 5);
    sub = node.subscribe("/mavros/local_position/pose", 5, &Takeoff::callback, this);
  }

  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped destPose;
    destPose.header = msg->header;

    destPose.pose = msg->pose;
    if (!takeoffCompleted(msg->pose)) {
      destPose.pose.position.z = 2 * takeoff_height;
    }
    else {
      destPose.pose.position.x = 0;
      destPose.pose.position.y = 0;
      destPose.pose.position.z = takeoff_height;

      destPose.pose.orientation.x = 0;
      destPose.pose.orientation.y = 0;
      destPose.pose.orientation.z = 0;
      destPose.pose.orientation.w = -1.0;
    }
    pub.publish(destPose);
  }

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;

  bool completed;
  double takeoff_height;

  bool takeoffCompleted(geometry_msgs::Pose currentPose) {
    if (currentPose.position.z > takeoff_height / 2) {
      completed = true;
    }
    return completed;
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "takeoff");

  Takeoff tkoff(std::atof(argv[1]));

  ros::spin();
  return 0;
}
