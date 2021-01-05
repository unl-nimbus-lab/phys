#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"

double LIN_TH = 0.3; // This is in rad.
double ANG_TH = 2.3; // This is in rad.
double CMD_VEL = 1.0;
bool INIT = true;
ros::Publisher g_cmd_pub;

#if 1
void judge_cmd_vel(double roll, double pitch, double yaw)
{
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0.0;
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = 0.0;

  // forward, see the pitch
  if(pitch > LIN_TH)
    vel_msg.linear.x = CMD_VEL;

  // backward, see the pitch
  if(pitch < -LIN_TH)
    vel_msg.linear.x = -CMD_VEL;

  // turn right, see the roll
  if(yaw > 0 && yaw < ANG_TH)
    vel_msg.angular.z = -CMD_VEL;

  // turn left, see the roll
  if(yaw < 0 && yaw > -ANG_TH)
    vel_msg.angular.z = CMD_VEL;

  g_cmd_pub.publish(vel_msg);

     
}
#endif

void imuCallback(const sensor_msgs::Imu::ConstPtr& inImu)
{
  double qx, qy, qz, qw;
  double roll, pitch, yaw;
  static double h_roll, h_pitch, h_yaw;
  //tf::Quaternion qh;

  qx = inImu -> orientation.x;
  qy = inImu -> orientation.y;
  qz = inImu -> orientation.z;
  qw = inImu -> orientation.w;

  tf::Quaternion q(qx, qy, qz, qw);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  std::cout << "oRoll: " << roll << ", oPitch: " << pitch << ", oYaw: " << yaw << std::endl;

  judge_cmd_vel(roll, pitch, yaw);
#if 0
  if(INIT)
  {
    qh = tf::Quaternion(qx, qy, qz, qw);
    INIT = false;
  }
  else
  {
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q - qh);
    m.getRPY(roll, pitch, yaw);
    std::cout << "oRoll: " << roll << ", oPitch: " << pitch << ", oYaw: " << yaw << std::endl;
  }
#endif


#if 0
  std::cout << "oRoll: " << roll << ", oPitch: " << pitch << ", oYaw: " << yaw << std::endl;
  if(INIT)
  {
    h_roll = roll; 
    h_pitch = pitch;
    h_yaw = yaw;
    INIT = false;
  }
  else
  {
    roll -= h_roll;
    pitch -= h_pitch;
    yaw -= h_yaw;
    std::cout << "hRoll: " << h_roll << ", hPitch: " << h_pitch << ", hYaw: " << h_yaw << std::endl;
    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
  }
#endif

#if 0
  sensor_msgs::Imu outIMU;
  outIMU.angular_velocity.x = inImu->angular_velocity.x;
  outIMU.angular_velocity.y = inImu->angular_velocity.y;
  outIMU.angular_velocity.z = inImu->angular_velocity.z;
  outIMU.angular_velocity_covariance = inImu->angular_velocity_covariance;
  outIMU.linear_acceleration.x = inImu->linear_acceleration.x;
  outIMU.linear_acceleration.y = inImu->linear_acceleration.y;
  outIMU.linear_acceleration.z = inImu->linear_acceleration.z;
  outIMU.linear_acceleration_covariance = inImu->linear_acceleration_covariance;
  outIMU.orientation.x = inImu->orientation.x;
  outIMU.orientation.y = inImu->orientation.y;
  outIMU.orientation.z = inImu->orientation.z;
  outIMU.orientation.w = inImu->orientation.w;
  outIMU.orientation_covariance = inImu->orientation_covariance;
  outIMU.header.stamp = inImu->header.stamp;
  outIMU.header.frame_id = "imu_link";
  outIMU.header.seq = 0;
  pub_imu.publish(outIMU);
#endif
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_utility");
  ros::NodeHandle nh;
  ros::Rate r(100);
  ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &imuCallback);
  g_cmd_pub = nh.advertise<geometry_msgs::Twist>("/diff_drive_controller/cmd_vel", 1);
  
  while(ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
