#include "pioneer_odometry.h"
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

using namespace robmovil;

#define WHEEL_BASELINE 0.331
#define WHEEL_RADIUS 0.0975
#define ENCODER_TICKS 500.0

inline double wrapAngle( double angle )
{
  double twoPi = 2.0 * M_PI;
  return angle - twoPi * floor( angle / twoPi );
}

PioneerOdometry::PioneerOdometry(ros::NodeHandle& nh)
  : nh_(nh), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = nh.subscribe("/robot/cmd_vel", 1, &PioneerOdometry::on_velocity_cmd, this);

  vel_pub_left_ = nh.advertise<std_msgs::Float64>("/robot/left_wheel/cmd_vel", 1);
  vel_pub_right_ = nh.advertise<std_msgs::Float64>("/robot/right_wheel/cmd_vel", 1);

  encoder_sub_ = nh.subscribe("/robot/encoders", 1, &PioneerOdometry::on_encoder_ticks, this);

  pub_odometry_ = nh.advertise<nav_msgs::Odometry>("/robot/odometry", 1);

  tf_broadcaster = boost::make_shared<tf::TransformBroadcaster>();
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::Twist& twist)
{
  double linearVel = twist.linear.x;
  double angularVel = twist.angular.z;
  
  double vLeft = (linearVel - angularVel * (WHEEL_BASELINE/2)) / WHEEL_RADIUS;
  double vRight = (linearVel + angularVel * (WHEEL_BASELINE/2)) / WHEEL_RADIUS;

  // publish left velocity
  {
    std_msgs::Float64 msg;
    msg.data = vLeft;

    vel_pub_left_.publish( msg );
  }

  // publish right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vRight;

    vel_pub_right_.publish( msg );
  }
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::EncoderTicks& encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (not ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_left_ = encoder.ticks_left.data;
    last_ticks_right_ = encoder.ticks_right.data;
    last_ticks_time = encoder.header.stamp;
    return;
  }

  int32_t delta_ticks_left = encoder.ticks_left.data - last_ticks_left_;
  int32_t delta_ticks_right = encoder.ticks_right.data - last_ticks_right_;

  // calculo el desplazamiento relativo

  double delta_left = M_PI * 2 * WHEEL_RADIUS * delta_ticks_left / ENCODER_TICKS;
  double delta_right = M_PI * 2 * WHEEL_RADIUS * delta_ticks_right / ENCODER_TICKS;

  double delta_theta = (delta_right - delta_left) / WHEEL_BASELINE;
  double delta_distance = (delta_left + delta_right) / 2;

  double delta_x = delta_distance * cos( theta_ );
  double delta_y = delta_distance * sin( theta_ );

  double delta_t = (encoder.header.stamp - last_ticks_time).toSec();

  // actualizo el estado local

  ROS_DEBUG_STREAM("theta " << theta_);
  ROS_DEBUG_STREAM("delta theta " << delta_theta);

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // normalizo el angulo
  //theta_ = wrapAngle( theta_ );

  ROS_DEBUG_STREAM("theta " << theta_ << std::endl);

  // Armo el mensaje de odometría

  nav_msgs::Odometry msg;

  msg.header.stamp = encoder.header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
  
  //msg.pose.covariance = ...

  msg.twist.twist.linear.x = delta_distance / delta_t;
 
  msg.twist.twist.linear.y = 0;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = delta_theta / delta_t;

  //msg.twist.covariance = ...
  pub_odometry_.publish( msg );

  // Actualizo las variables de estado

  last_ticks_left_ = encoder.ticks_left.data;
  last_ticks_right_ = encoder.ticks_right.data;
  last_ticks_time = encoder.header.stamp;

  /* Mando tambien un transform usando TF */
  tf::Transform t;
  tf::poseMsgToTF(msg.pose.pose, t);
  tf_broadcaster->sendTransform(tf::StampedTransform(t, encoder.header.stamp, "odom", "base_link"));

}
