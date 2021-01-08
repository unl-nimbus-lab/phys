//#include <vehicle_perception/surf_detector.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

tf::TransformBroadcaster *broadcaster;

void imuCallback( const sensor_msgs::Imu::ConstPtr& msg ){
  tf::Vector3 translation(0, 0, 0);
  tf::Quaternion rotation( msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Transform pose(rotation, translation);
  tf::StampedTransform t(pose, ros::Time::now(), "/head_unit", "/imu");
  broadcaster->sendTransform(t);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_to_tf");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub;
  imu_sub = nh.subscribe("/android/imu", 1, imuCallback);

  broadcaster = new tf::TransformBroadcaster();

  ros::spin();
  return 0;
}

