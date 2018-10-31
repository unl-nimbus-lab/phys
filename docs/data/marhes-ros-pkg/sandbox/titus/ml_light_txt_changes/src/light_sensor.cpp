#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "phidgets_ros/Float64Stamped.h"
#include "tf/tf.h"

class LightSensor
{
public:
  LightSensor(ros::NodeHandle nh);
private:
  ros::NodeHandle n_;
  ros::Publisher flls_pub_, frls_pub_, rlls_pub_, rrls_pub_;
  ros::Subscriber odom_sub_;
  ros::Timer timer_;
  
  double x_, y_, theta_, angle_, sensor_hypotenuse_;
  
  void cb_tmr(const ros::TimerEvent& event);
  void cb_odom(nav_msgs::Odometry msg);
  double distance(double x, double y);
};

LightSensor::LightSensor(ros::NodeHandle nh)
{
  double xdist, ydist;
  
  n_ = nh;
  ros::NodeHandle n_private("~");
  n_private.param("xdist", xdist, 8 / 0.0254);
  n_private.param("ydist", ydist, 6 / 0.0254);
  
  angle_ = atan2(ydist, xdist);
  sensor_hypotenuse_ = sqrt(xdist * xdist + ydist * ydist);
  
  flls_pub_ = n_.advertise<phidgets_ros::Float64Stamped>("flls", 1);
  frls_pub_ = n_.advertise<phidgets_ros::Float64Stamped>("frls", 1);
  rlls_pub_ = n_.advertise<phidgets_ros::Float64Stamped>("rlls", 1);
  rrls_pub_ = n_.advertise<phidgets_ros::Float64Stamped>("rrls", 1);
  odom_sub_ = n_.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 10, &LightSensor::cb_odom, this);
  timer_ = n_.createTimer(ros::Duration(0.02), &LightSensor::cb_tmr, this);
}

void LightSensor::cb_tmr(const ros::TimerEvent& event)
{
  double flls_x, frls_x, rlls_x, rrls_x;
  double flls_y, frls_y, rlls_y, rrls_y;
  phidgets_ros::Float64Stamped msg;
  
  flls_x = x_ + cos(theta_ + angle_) * sensor_hypotenuse_;
  frls_x = x_ + cos(theta_ - angle_) * sensor_hypotenuse_;
  rlls_x = x_ + cos(theta_ + M_PI - angle_) * sensor_hypotenuse_;
  rrls_x = x_ + cos(theta_ - M_PI + angle_) * sensor_hypotenuse_;
  flls_y = y_ + sin(theta_ + angle_) * sensor_hypotenuse_;
  frls_y = y_ + sin(theta_ - angle_) * sensor_hypotenuse_;
  rlls_y = y_ + sin(theta_ + M_PI - angle_) * sensor_hypotenuse_;
  rrls_y = y_ + sin(theta_ - M_PI + angle_) * sensor_hypotenuse_;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "light_sensor";
  msg.data = 1 / pow(distance(flls_x, flls_y), 2);
  flls_pub_.publish(msg);
  msg.data = 1 / pow(distance(frls_x, frls_y), 2);
  frls_pub_.publish(msg);
  msg.data = 1 / pow(distance(rlls_x, rlls_y), 2);
  rlls_pub_.publish(msg);
  msg.data = 1 / pow(distance(rrls_x, rrls_y), 2);
  rrls_pub_.publish(msg);
}

void LightSensor::cb_odom(nav_msgs::Odometry msg)
{
  x_ = msg.pose.pose.position.x;
  y_ = msg.pose.pose.position.y;
  theta_ = tf::getYaw(msg.pose.pose.orientation);
}

double LightSensor::distance(double x, double y)
{
  return sqrt(x * x + y * y);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "light_sensors_fake");
	ros::NodeHandle n;

  LightSensor* s = new LightSensor(n);
  ros::spin();

  return 0;
}
