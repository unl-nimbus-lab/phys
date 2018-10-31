#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

//test-----------interactive control
//#include <interactive_markers/interactive_marker_server.h>

 //test-----tf transform------------
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

//function convert arduino to ros
//------------ROS version-------
// #include <iostream>
inline long millis() {
  return (1e3*ros::Time::now().sec + 1e-6*ros::Time::now().nsec);
}
inline long micros() {
  return (1e6*ros::Time::now().sec + 1e-3*ros::Time::now().nsec);
}

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define delay(x) ros::Duration(x/1000.0).sleep()
#define say(x) (std::cout << x)
#define sayend(x) (std::cout << x << std::endl)
#define saytab(x) (std::cout << x << "\t")
#define saycomma(x) (std::cout << x << " ,")
#define Min(a,b) ((a)<(b)?(a):(b))
#define Max(a,b) ((a)>(b)?(a):(b))
#define isfinite(x) (!(x!=x))

//----------------------------------
#include "useful_fn.h"
#include "config.h"
// #include "MAF.h"
// #include "mpu6050.h"
// #include "compass.h"
// #include "ahrs.h"
// #include "alt.h"
// #include "controlsystem.h"
// #include "motor.h"


float GPS_hAcc = 10;
float GPS_vAcc = 10;
float GPS_vel_EAST = 0;
float GPS_vel_NORTH = 0;
float R[3][3]={{1,0,0},
               {0,1,0},
               {0,0,1}};
float gyro_bias[3]={0,0,0};
float acc_bias[3]={0,0,0};

bool GPS_ALIVE = false;
bool _fix_ok = false;
bool gps_vel_ned_valid=false;


float         vision_x[2],vision_y[2],vision_z[2];
float         vision_bx[2],vision_by[2],vision_bz[2];
unsigned long vision_time_stamp;
unsigned long last_vision_time;
bool          vision_is_good=false;
int           vision_topic_timeout = 501; //0.5 sec timeout


#include "inertial_nav.h"
//1 ahrs  ok
//2 alt   ok
//3 remote cmd
//4 control



sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField compass_msg;
geometry_msgs::PoseWithCovarianceStamped pose_gps,pose_nav,inertial_gps;
geometry_msgs::PoseStamped RC;
geometry_msgs::TwistWithCovarianceStamped pose_gps_vel;
nav_msgs::Odometry alt_odom;

//test-----tf transform------------

// void poseCallback(const geometry_msgs::Pose& msg, std::string turtle_name){
//   static tf::TransformBroadcaster br;
//   tf::Transform transform;

//   tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
//   //q.setRPY(0, 0, msg->theta);    //set quaternion q from RPY

//   transform.setOrigin( tf::Vector3(msg.position.x, msg.position.y, msg.position.z) );
//   transform.setRotation(q);
//   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
// }




double roll, pitch, yaw;



void imucallback(sensor_msgs::Imu data)
{
  imu_msg=data;

  tf::Quaternion q;
  //q0 in ahrs is w but in oriantation is x
  q[0]=data.orientation.w;
  q[1]=data.orientation.x;
  q[2]=data.orientation.y;
  q[3]=data.orientation.z;
  float q0q1 = q[0]*q[1];
  float q0q2 = q[0]*q[2];
  float q0q3 = q[0]*q[3];
  float q1q1 = q[1]*q[1];
  float q1q2 = q[1]*q[2];
  float q1q3 = q[1]*q[3];
  float q2q2 = q[2]*q[2];   
  float q2q3 = q[2]*q[3];
  float q3q3 = q[3]*q[3];  
  R[0][0] = 2*(0.5 - q2q2 - q3q3);//2*(0.5 - q2q2 - q3q3);//=q0q0 + q1q1 - q2q2 - q3q3
  R[0][1] = 2*(q1q2 - q0q3);//2*(q0q1 + q2q3)
  R[0][2] = 2*(q1q3 + q0q2);//2*(q1q3 - q0q2); 2*(q0q2 - q1q3)
  R[1][0] = 2*(q1q2 + q0q3);
  R[1][1] = 2*(0.5 - q1q1 - q3q3);//2*(0.5 - q1q1 - q3q3);//q0q0 - q1q1 + q2q2 - q3q3
  R[1][2] = 2*(q2q3 - q0q1);
  R[2][0] = 2*(q1q3 - q0q2);//-sin pitch
  R[2][1] = 2*(q2q3 + q0q1);
  R[2][2] = 2*(0.5 - q1q1 - q2q2);//2*(0.5 - q1q1 - q2q2);//=q0q0 - q1q1 - q2q2 + q3q3
  // saytab(q[0]);saytab(q[1]);saytab(q[2]);sayend(q[3]);
  tf::Quaternion q_hmt;
  q_hmt[0]=data.orientation.x;
  q_hmt[1]=data.orientation.y;
  q_hmt[2]=data.orientation.z;
  q_hmt[3]=data.orientation.w;
  tf::Matrix3x3 m(q_hmt);
  m.getRPY(roll, pitch, yaw);
  // ROS_INFO("rpy:[%f] :[%f] : [%f]",roll,pitch, yaw);
}
void compasscallback(sensor_msgs::MagneticField data)
{
  compass_msg=data;
}

void pose_gps_callback(geometry_msgs::PoseWithCovarianceStamped data)
{
  pose_gps=data;
  GPS_hAcc = pose_gps.pose.covariance[0];
  GPS_vAcc = 10;
  hx = pose_gps.pose.pose.position.x;
  hy = pose_gps.pose.pose.position.y;
  // ROS_INFO("pz:[%f]", data.pose.pose.position.z);
  GPS_ALIVE = true;
  _fix_ok = true;
  
}
void pose_gps_vel_callback(geometry_msgs::TwistWithCovarianceStamped data)
{
  pose_gps_vel=data;
  GPS_vel_EAST = pose_gps_vel.twist.twist.linear.x*100;
  GPS_vel_NORTH = pose_gps_vel.twist.twist.linear.y*100;
  gps_vel_ned_valid=true;
  // ROS_INFO("pz:[%f]", data.pose.pose.position.z);
}
void pose_nav_callback(geometry_msgs::PoseWithCovarianceStamped data)
{
  pose_nav=data;
  // ROS_INFO("pose_nav_pz:[%f]", data.pose.pose.position.z);
}
void alt_callback(nav_msgs::Odometry data)
{
  alt_odom = data;
}

static void process_1000HZ();
static void process_100HZ();
static void process_50HZ();
static void process_33HZ();
static void process_15HZ();
static void process_10HZ();
static void pre_test_num_sensor();
static void test_num_readsensor();
uint8_t ADJUSTABLE_TASK();

int main( int argc, char** argv )
{
  ros::init(argc, argv, "thirdorder");
  ros::NodeHandle n;
  ros::Rate r(200);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu_max", 1, imucallback);
  ros::Subscriber pose_gps_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose",1, pose_gps_callback);
  ros::Subscriber pose_gps_vel_sub = n.subscribe<geometry_msgs::TwistWithCovarianceStamped>("/imu_max/Navvel",1,pose_gps_vel_callback);
  ros::Subscriber alt_odom_sub = n.subscribe<nav_msgs::Odometry>("/imu_max/alt_odometry", 1,alt_callback);
  // ros::Subscriber compass_sub = n.subscribe<sensor_msgs::MagneticField>("/imu_max/Mag", 200, compasscallback);
  
  ros::Publisher  inertial_gps_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/Inertial_nav", 1);

  bool init_quad_complete = false;


  if(argc>1) 
  {
    _delay_gps = atof(argv[1]); //convert char to float 
    saytab("_delay_gps  -- is now set to ");sayend(_delay_gps);
  }

  t_prev = micros();
 while (ros::ok())
  {
      //100 hz task

      ros::spinOnce();
      r.sleep();//for 200hz stamped time

      currentTime = micros();


      if (currentTime - previousTime > 10000)
      {
        frameCounter++;

        G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
        if(G_Dt >0.02) G_Dt =0.01;  //Prevent error from time peak to 1.83e13

        hundredHZpreviousTime = currentTime;

        Acc_f[0] = imu_msg.linear_acceleration.x/9.80655;
        Acc_f[1] = imu_msg.linear_acceleration.y/9.80655;
        Acc_f[2] = imu_msg.linear_acceleration.z/9.80655;
        gyro[0] = imu_msg.angular_velocity.x;
        gyro[1] = imu_msg.angular_velocity.y;
        gyro[2] = imu_msg.angular_velocity.z;
        position_estimator();


        inertial_gps.header.stamp = ros::Time::now();
        inertial_gps.header.frame_id = "/odom";
        inertial_gps.pose.pose.position.x = x_est[0];
        inertial_gps.pose.pose.position.y = y_est[0];
        inertial_gps.pose.pose.position.z = alt_odom.pose.pose.position.z;
        inertial_gps.pose.pose.orientation.x = imu_msg.orientation.x;
        inertial_gps.pose.pose.orientation.y = imu_msg.orientation.y;
        inertial_gps.pose.pose.orientation.z = imu_msg.orientation.z;
        inertial_gps.pose.pose.orientation.w = imu_msg.orientation.w;

        inertial_gps.pose.covariance[0]   =inertial_gps.pose.covariance[7] = GPS_hAcc;
        inertial_gps.pose.covariance[14]  =0.01; //z position
        inertial_gps.pose.covariance[21]=  inertial_gps.pose.covariance[28]=inertial_gps.pose.covariance[35]=0.001;




        inertial_gps_pub.publish(inertial_gps);






      if (frameCounter % TASK_50HZ == 0) {
         G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
         fiftyHZpreviousTime = currentTime;


      }
      ////////////////////////////////////////33 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_33HZ == 0) {

      }
      ///////////////////////////////////////25 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_25HZ == 0) {
        // for test
      //  if (!flag_trim) update_NAV(0.04);
      }
      ////////////////////////////////////////15 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_15HZ == 0) {
        //////////////////CHECK IF NOT FLYING THEN CAN DOWNLOAD SOME USEFUL VALUE TO CHECK ON GROUND STATION
        // if (!armed) {
        //   if (Serial3.available()) {
        //     RECIEVE_COMMAND_FROM_GROUND_STATION();
        //   }
        // }
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // FLIGHT_MODE(AUX_1);

      }
      if (frameCounter % ADJUSTABLE_TASK() == 0)     {
        //FOR TEST

            G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
            lowPriorityTenHZpreviousTime =  currentTime;

      }

    // Reset frameCounter back to 0 after reaching 100 (1s)
    if (frameCounter >= 100) {

        frameCounter = 0;
      }

      previousTime = currentTime;
    }
  }
  return 0;
}



uint8_t ADJUSTABLE_TASK() {
  return (10/*+G_Dt_sonar*4000*/);
}

static void process_1000HZ() {
  //        sayend(currentTime - sensorPreviousTime);
  sensorPreviousTime = currentTime;
}
static void process_100HZ() {
}

static void process_50HZ() {
}
static void process_33HZ() {
  //            G_Dt = (currentTime - thitythreeHZpreviousTime) / 1000000.0;
  //            thitythreeHZpreviousTime = currentTime;
  //Get sonar_h variable for instant height in cm.
  // maf_pressure();  //try faster

}
static void process_15HZ() {

}


static void process_10HZ() {
//            G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
//
//             currentTime = micros();
//
//             lowPriorityTenHZpreviousTime =  currentTime;
}
