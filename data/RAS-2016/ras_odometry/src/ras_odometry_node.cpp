#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "phidgets/motor_encoder.h"

#define CPR 900
#define PI 3.14159265359
#define B 0.228
#define R 0.072/2

int count_change_left;
int count_change_right;

void leftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg) {
	count_change_left = msg->count_change;
}

void rightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg) {
	count_change_right = -msg->count_change;
}

int main(int argc, char **argv) {

	geometry_msgs::Twist current_pos;
	geometry_msgs::Twist pose_diff;

  ros::init(argc, argv, "odometry");
  ros::NodeHandle n("~");
  double x_origin;
  double y_origin;
  double start_angle;
  n.getParam("x", x_origin);
  n.getParam("y", y_origin);
  n.getParam("angle", start_angle);

	current_pos.angular.x = 0;
	current_pos.angular.y = 0;
  current_pos.angular.z = start_angle;
  current_pos.linear.x = x_origin;
  current_pos.linear.y = y_origin;
	current_pos.linear.z = 0;

	pose_diff.angular.x = 0;
	pose_diff.angular.y = 0;
	pose_diff.angular.z = 0;
	pose_diff.linear.x = 0;
	pose_diff.linear.y = 0;
	pose_diff.linear.z = 0;

	ros::Publisher pos_pub = n.advertise<geometry_msgs::Twist>("/robot8/pose", 1000);
	ros::Publisher posdiff_pub = n.advertise<geometry_msgs::Twist> ("/robot8/pose_diff", 1000);
	ros::Subscriber left_encoder_sub = n.subscribe("/left/encoder", 1000, &leftEncoderCallback);
	ros::Subscriber right_encoder_sub = n.subscribe("/right/encoder", 1000, &rightEncoderCallback);
	ros::Rate loop_rate(125);

	while(ros::ok()) {

		double d_theta_right = (double) count_change_right / CPR * 2 * PI;
		double d_theta_left = (double) count_change_left / CPR * 2 * PI;

		pose_diff.angular.z = (d_theta_right - d_theta_left) * R / B;
		pose_diff.linear.x = (d_theta_left + d_theta_right) / 2 * R;

		current_pos.angular.z += R / B * (d_theta_right - d_theta_left);
		current_pos.linear.x += R * cos(current_pos.angular.z) / 2 * (d_theta_left + d_theta_right);
		current_pos.linear.y += R * sin(current_pos.angular.z) / 2 * (d_theta_left + d_theta_right);

		pos_pub.publish(current_pos);
		posdiff_pub.publish(pose_diff);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
