#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <alpha_msgs/FilteredState.h>
#include <alpha_msgs/IMU.h>
#include <alpha_msgs/AirPressure.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
ros::Publisher pose_pub;
ros::Publisher baro_raw_pub;
geometry_msgs::Quaternion q_raw;
float vz,z=0;
bool calibrate = false;
bool calibrated = false;
int baro_count = 0;
float baro_sum = 0;
float k_vz = -0.001;
float k_z = -0.035;
float baro_altitude;
float zero_altitude = 20;
float temp = 30;
float p0 = 0;
float world_az;
float az_offset=9.8;
float az_sum;
int az_count;
ros::Time calibrate_start_time;
void ahrs_cb(const geometry_msgs::Quaternion::ConstPtr msg){
  alpha_msgs::FilteredState pose;
  float q0 = msg->w;
  float q1 = msg->x;
  float q2 = msg->y;
  float q3 = msg->z;
  q_raw = (*msg);
  pose.x = 0;
  pose.y = 0;
  pose.z = z;
  pose.roll = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  pose.pitch = asin(2*(q0*q2-q3*q1));
  pose.yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

  pose_pub.publish(pose);
}

void imu_cb(const alpha_msgs::IMU::ConstPtr msg){
  float q_norm = sqrt(q_raw.w*q_raw.w+q_raw.x*q_raw.x+q_raw.y*q_raw.y+q_raw.z*q_raw.z);
  //normalize q
  q_raw.w /= q_norm;
  q_raw.x /= q_norm;
  q_raw.y /= q_norm;
  q_raw.z /= q_norm;
  geometry_msgs::Quaternion q_inv;
  q_inv.w = q_raw.w;
  q_inv.x = -q_raw.x;
  q_inv.y = -q_raw.y;
  q_inv.z = -q_raw.z;
  geometry_msgs::Quaternion &q = q_inv;
  float ax = msg->linear_acceleration.x;
  float ay = msg->linear_acceleration.y;
  float az = msg->linear_acceleration.z;
  float raw_az = 
    2*(q.x*q.z+q.w*q.y)*ax +
    2*(q.y*q.z-q.w*q.x)*ay +
    (1-2*q.x*q.x-2*q.y*q.y)*az;
  world_az = raw_az - az_offset;
  //  std::cout<<"az "<<world_az<<std::endl;
  if(calibrate){
    az_count += 1;
    az_sum += raw_az;
  }
}
void baro_cb(const alpha_msgs::AirPressure::ConstPtr msg){

  float baro_raw = msg->pressure;

  if(calibrate){
    baro_count += 1;
    baro_sum += baro_raw;
  }
  else if(calibrated){
    baro_altitude = ((pow(p0/baro_raw,1/5.257)-1)*(temp+273.15))/0.0065-zero_altitude;
    //    std::cout<<"baro altitude "<<baro_altitude<<std::endl;
    std_msgs::Float32 msg;
    msg.data = baro_altitude;
    baro_raw_pub.publish(msg);
  }
}
void calib_cb(const std_msgs::Empty::ConstPtr msg){
  calibrate = true;
  calibrate_start_time = ros::Time::now();
  baro_count = 0;
  baro_sum = 0;
  az_sum =  0;
  az_count = 0;
}


int main(int argc, char* argv[]){
  ros::init(argc,argv,"simple_pose_node");

  ros::NodeHandle nh;
  ros::Rate rate(100);
  ros::Subscriber ahrs_sub = nh.subscribe("/ahrs",10,ahrs_cb);
  ros::Subscriber imu_sub = nh.subscribe("/imu",1,imu_cb);
  ros::Subscriber baro_sub = nh.subscribe("/pressure",1,baro_cb);
  ros::Subscriber calib_sub = nh.subscribe("/calibrate",1,calib_cb);
  pose_pub = nh.advertise<alpha_msgs::FilteredState>("/pose",10);
  baro_raw_pub = nh.advertise<std_msgs::Float32>("/baro_altitude",10);
  ros::Duration calibration_time(3);
  ros::Time last_update;
  while(ros::ok()){
    if(calibrate){
      if(calibrate && (ros::Time::now() > calibrate_start_time + calibration_time)){
	float baro_mean = baro_sum/baro_count;
	std::cout<<"baro mean "<<baro_mean<<std::endl;
	p0 = baro_mean/pow((1-(0.0065*zero_altitude)/(temp+0.0065*zero_altitude+273.15)),5.257);
	std::cout<< "p0 "<<p0<<std::endl;
	az_offset = az_sum/az_count;
	calibrate=false;
	z = 0;
	vz = 0;
	calibrated = true;
      }
    }
    else if(calibrated){
      float dt = (ros::Time::now()-last_update).toSec();
      vz += world_az*dt;
      z +=vz*dt;
      vz +=  k_vz*(z - baro_altitude);
      z += k_z*(z - baro_altitude);
      last_update = ros::Time::now();
    }
    last_update = ros::Time::now();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;

}
