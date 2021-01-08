#include <ros/ros.h>
#include <ros/time.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Range.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
// #include <geometry_msgs/Vector3Stamped.h>
#include <sensor_fusion_comm/ExtEkf.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>

#include "nav_msgs/Odometry.h"
#include "crossover_nav/Ack.h"
#include "crossover_nav/Navdata.h"
#include "crossover_nav/odom_data.h"
#include "crossover_nav/Status.h"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>



#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <termios.h>

//function convert arduino to ros
//------------ROS version-------
// #include <iostream>
inline long millis() {
  return (1e3 * ros::Time::now().sec + 1e-6 * ros::Time::now().nsec);
}
inline long micros() {
  return (1e6 * ros::Time::now().sec + 1e-3 * ros::Time::now().nsec);
}
//not use millis() it give me online floating point behind sec
//go use ros::Time::now();
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define delay(x) ros::Duration(x/1000.0).sleep()
#define say(x) (std::cout << x)
#define sayend(x) (std::cout << x << std::endl)
#define saytab(x) (std::cout << x << "\t")
#define saycomma(x) (std::cout << x << " ,")
#define Min(a,b) ((a)<(b)?(a):(b))
#define Max(a,b) ((a)>(b)?(a):(b))
#define isfinite(X) std::isfinite(X)
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define wrap_pi(x) (x < -3.14 ? x+6.28 : (x > 3.14 ? x - 6.28: x))
#define HALF_M_PI 1.570796
//----------------------------------
#include "useful_fn.h"
#include "config.h"
float GPS_hAcc = 10;
float GPS_vAcc = 10;
float GPS_vel_EAST = 0;
float GPS_vel_NORTH = 0;
float R[3][3] = {{1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
};
float gyro_bias[3] = {0, 0, 0};
float acc_bias[3] = {0, 0, 0};

bool GPS_ALIVE = false;
bool _fix_ok = false;
bool gps_vel_ned_valid = false;

const float delay_slam  = 0.14;

float         vision_x[2], vision_y[2], vision_z[2];
float         vision_bx[2], vision_by[2], vision_bz[2];
unsigned long vision_time_stamp;
unsigned long last_vision_time;
bool          vision_is_good = false;
int           vision_topic_timeout = 500; //0.5 sec timeout


#define NOT_OK 0
#define READY_TO_INIT 1
#define INITING 2
#define READY_TO_USE 3
uint8_t slam_stage = 0;

float z_slam = 0;
float scale_slam = 1;

#define SAMPLE 6

double DELAY_SLAM = 0;

#include "inertial_nav.h"

//global

sensor_msgs::Imu imuMsg;
sensor_msgs::MagneticField magMsg;
sensor_msgs::Joy remoteMsg;
sensor_msgs::FluidPressure baroMsg;
sensor_msgs::Temperature tempMsg;
sensor_msgs::NavSatFix gpsrawMsg;
sensor_msgs::Range sonarMsg;

geometry_msgs::PoseWithCovarianceStamped poseMsg, inertial_gps, odom_pose_filtered;
geometry_msgs::TwistWithCovarianceStamped navvelMsg;
geometry_msgs::PointStamped baroaltMsg;
// geometry_msgs::Vector3Stamped magMsg;
nav_msgs::Odometry pose_navMsg, msf_data;

nav_msgs::Odometry altMsg, desire_navMSG;

crossover_nav::Ack ackMsg;
crossover_nav::Navdata NavMsg;
crossover_nav::odom_data odom_data;
crossover_nav::Status StatusMsg;

sensor_fusion_comm::DoubleArrayStamped state_out;
sensor_fusion_comm::DoubleArrayStamped ppid_outMsg;
sensor_fusion_comm::DoubleArrayStamped ppid2_outMsg;
sensor_fusion_comm::DoubleArrayStamped motor_outMsg;
sensor_fusion_comm::DoubleArrayStamped control_outMsg;
bool GPS_STATE = false, GPS_LAST_STATE = true;
bool VISION_STATE = false, VISION_LAST_STATE = true;
bool VISION_IS_READY = false;
bool GPS_IS_READY = false;



double ROLL_IMU, PITCH_IMU, YAW_IMU;



ros::Time  time_start;



bool request_offset = false;
float x_msf_offset, y_msf_offset;
float x_gps_offset, y_gps_offset;
float x_slam_offset, y_slam_offset, z_slam_offset;
float msf_scale = 1;

float baro_inertial_z_late;

tf::Vector3 gps_offset(0,0,0);
tf::Vector3 gps_inertial_offset(0,0,0);
tf::Vector3 vision_offset(0,0,0);
tf::Vector3 vision_scaled(0,0,0);
bool status_updated = false;
double YAW_SLAM_CALED=0;
bool GPS_INERTIAL_INITED = false;

inline void check_state();
inline void gps_hacc_analysis();


#include "Ulink_capter_and_inertial_nav.h"





//param variable
double DELAY_NAV;
double HANDLE_OFF_SPD;
double HANDLE_OFF_RADIUS;
double GPS_SWITCH_RADIUS;
double VIS_SWITCH_RADIUS;
double VIS_SWITCH_ANGLE;
double GPS_HACC_CUTOFF;
double ALT_DIFF_SCALE;
// VISION NOISE
double VIS_POSITION_NOISE;
double VIS_ORIENTATION_NOISE;
// GPS analysis
double TAU_GRAD_GPS;
double MARGIN;
double LOWER_LIMIT;
double UPPER_LIMIT;
double MAX_HACC_ALLOW;
double MIN_HACC_ALLOW;

//void msf_callback(const nav_msgs::Odometry& data);
inline float get_pose_nav_z_buffer();
void odom_callback(const crossover_nav::odom_data::ConstPtr& data);
void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data);



int main(int argc, char **argv)
{

  //init ros node
  ros::init(argc, argv, "quad_talker");
  ros::NodeHandle n;
  ros::Rate r(400);


  //variable need 
  std::string s;
  //retreve param
  n.param<std::string>("DEVICE", port, "/dev/ttyACM0");
  n.param("BAUDRATE", baud, 115200);
  n.param("DELAY_NAV", DELAY_NAV, 0.25);
  n.param("HANDLE_OFF_SPD", HANDLE_OFF_SPD, 0.05);
  n.param("HANDLE_OFF_RADIUS", HANDLE_OFF_RADIUS, 0.15);
  n.param("GPS_SWITCH_RADIUS", GPS_SWITCH_RADIUS, 0.5);
  n.param("VIS_SWITCH_RADIUS", VIS_SWITCH_RADIUS, 0.5);
  n.param("VIS_SWITCH_ANGLE", VIS_SWITCH_ANGLE, 0.1);
  n.param("GPS_HACC_CUTOFF", GPS_HACC_CUTOFF, 4.0);
  n.param("ALT_DIFF_SCALE", ALT_DIFF_SCALE, 0.1);

  n.param("VIS_POSITION_NOISE", VIS_POSITION_NOISE, 0.1);
  n.param("VIS_ORIENTATION_NOISE", VIS_ORIENTATION_NOISE, 0.1);


  n.param("TAU_GRAD_GPS", TAU_GRAD_GPS,0.22);
  n.param("MARGIN", MARGIN, 1.0);
  n.param("LOWER_LIMIT", LOWER_LIMIT, -5.0);
  n.param("UPPER_LIMIT", UPPER_LIMIT, 5.0);
  n.param("MAX_HACC_ALLOW", MAX_HACC_ALLOW, 6.0);
  n.param("MIN_HACC_ALLOW", MIN_HACC_ALLOW, 2.8);

  n.param("DEBUG_CONTROL_OUT" , DEBUG_CONTROL_OUT, false);
  n.param("DEBUG_PPID_OUT"    , DEBUG_PPID_OUT, false);
  n.param("DEBUG_PPID2_OUT"   , DEBUG_PPID2_OUT, false);
  n.param("DEBUG_RADIO_OUT"   , DEBUG_RADIO_OUT, false);
  n.param("DEBUG_MOTOR_OUT"   , DEBUG_MOTOR_OUT, false);
  // n.param("SWITCH_RADIUS", baud, 115200);

  saytab("--- DEVICE  \t\t\t\t\t"); sayend(port);
  saytab("--- BAUDRATE  \t\t\t\t\t"); sayend(baud);
  saytab("--- _delay_gps  \t\t\t\t\t"); sayend(DELAY_NAV);
  saytab("--- HANDLE_OFF_SPD  \t\t\t"); sayend(HANDLE_OFF_SPD);
  saytab("--- GPS_SWITCH_RADIUS  \t\t\t\t"); sayend(GPS_SWITCH_RADIUS);
  saytab("--- VIS_SWITCH_RADIUS  \t\t\t\t"); sayend(VIS_SWITCH_RADIUS);
  saytab("--- VIS_SWITCH_ANGLE  \t\t\t\t"); sayend(VIS_SWITCH_ANGLE);
  saytab("--- GPS_HACC_CUTOFF  \t\t\t\t"); sayend(GPS_HACC_CUTOFF);
  saytab("--- ALT_DIFF_SCALE  \t\t\t\t"); sayend(ALT_DIFF_SCALE);

  _delay_gps = DELAY_NAV;



  //publisher
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_max", 10);
  ros::Publisher pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 10);
  ros::Publisher navvel = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/imu_max/Navvel", 10);
  ros::Publisher posenav = n.advertise<nav_msgs::Odometry>("/imu_max/pose_nav", 10);
  ros::Publisher gpsraw = n.advertise<sensor_msgs::NavSatFix>("/imu_max/Gpsraw", 10);
  // ros::Publisher sonar = n.advertise<sensor_msgs::Range>("/sonar", 10);
  // ros::Publisher Nav = n.advertise<crossover_nav::Navdata>("/imu_max/Navdata", 10);
  ros::Publisher alt = n.advertise<nav_msgs::Odometry>("/imu_max/alt_odometry", 10);
  ros::Publisher des_nav = n.advertise<nav_msgs::Odometry>("/imu_max/pose_des", 10);
  ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("/imu_max/mag", 10);

  ros::Publisher control_pub,ppid_pub,ppid2_pub,radio_pub,motor_pub;
  if(DEBUG_CONTROL_OUT)
    control_pub = n.advertise<sensor_fusion_comm::DoubleArrayStamped>("/imu_max/control_out",1);
  if(DEBUG_PPID_OUT)
    ppid_pub = n.advertise<sensor_fusion_comm::DoubleArrayStamped>("/imu_max/ppid_out",1);
  if(DEBUG_PPID2_OUT)
    ppid2_pub = n.advertise<sensor_fusion_comm::DoubleArrayStamped>("/imu_max/ppid2_out",1);
  if(DEBUG_RADIO_OUT)
    radio_pub = n.advertise<sensor_msgs::Joy>("/imu_max/radio_out", 1);
  if(DEBUG_MOTOR_OUT)
    motor_pub = n.advertise<sensor_fusion_comm::DoubleArrayStamped>("/imu_max/motor_out",1);
  
  //inertial nav
  ros::Publisher  inertial_gps_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/Inertial_nav", 10);
  ros::Publisher  baro_alt_pub = n.advertise<geometry_msgs::PointStamped>("/imu_max/baro_alt", 10);


  // offset pub
  ros::Publisher  status_pub = n.advertise<crossover_nav::Status>("/imu_max/status",1);


  //odom filtered
  ros::Publisher odom_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/odom_pose_filtered", 10);

  //state out pub
  //ros::Publisher state_out_pub = n.advertise<sensor_fusion_comm::ExtEkf>("/state_out2",10);

  //subscriber
  // ros::Subscriber msf_sub = n.subscribe<nav_msgs::Odometry>("/msf_core/odometry", 10,msf_callback);
  ros::Subscriber odom_sub = n.subscribe<crossover_nav::odom_data>("/slam", 10, odom_callback);
  ros::Subscriber state_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 10, state_out_callback);


  //initialize imu covariance
  for (int i = 0; i < 9; i++)
  {
    imuMsg.orientation_covariance[i] = 0;
    imuMsg.angular_velocity_covariance[i] = 0;
    imuMsg.linear_acceleration_covariance[i] = 0;
    gpsrawMsg.position_covariance[i] = 0;
    altMsg.pose.covariance[i] = 0;
  }
  // memset(&altMsg.pose.covariance,0,sizeof(altMsg.pose.covariance));
  for (int i = 0; i < 36; i++)
  {
    poseMsg.pose.covariance[i] = 0;
    navvelMsg.twist.covariance[i] = 0;
  }
  imuMsg.orientation_covariance[0] = imuMsg.orientation_covariance[4] = imuMsg.orientation_covariance[8] = 1;
  imuMsg.angular_velocity_covariance[0] = imuMsg.angular_velocity_covariance[4] = imuMsg.angular_velocity_covariance[8] = 1.2184696791468346e-7;
  imuMsg.linear_acceleration_covariance[0] = imuMsg.linear_acceleration_covariance[4] = imuMsg.linear_acceleration_covariance[8] = 8.99999999999e-8;
  state_out.data.resize(36);
  control_outMsg.data.resize(4);
  ppid_outMsg.data.resize(12);
  ppid2_outMsg.data.resize(12);
  motor_outMsg.data.resize(4);




  //initializa frame id
  imuMsg.header.frame_id  = "base_link";
  //inertial nav estimation
  pose_navMsg.header.frame_id = "odom";
  pose_navMsg.child_frame_id  = "base_link";
  //des nav visual
  desire_navMSG.header.frame_id = "base_link";
  desire_navMSG.child_frame_id  = "vel_desire";
  //GPS pose in m or m/s and covaraince calculate from hacc vacc
  poseMsg.header.frame_id = "odom";  //recheck odom or world?
  // poseVal.child_frame_id  = 'base_link'
  //gpsraw to use natsatfix
  gpsrawMsg.header.frame_id  = "base_link";
  navvelMsg.header.frame_id  = "base_link";

  altMsg.header.frame_id = "odom";
  altMsg.child_frame_id  = "base_link";

  odom_pose_filtered.header.frame_id = "odom";

  StatusMsg.header.frame_id="odom";

  // SETUP SERIAL PORT

  // Exit if opening port failed
  // Open the serial port.
  if (!silent)
    printf("Trying to connect to %s.. \n", port.c_str());
  fd = open_port(port);
  if (fd == -1)
  {
    if (!silent)
      fprintf(stderr, "failure, could not open port.\n");
    exit(EXIT_FAILURE);
  }
  else
  {
    if (!silent)
      printf("success.\n");
  }
  if (!silent)
    printf("Trying to configure %s.. ", port.c_str());
  bool setup = setup_port(fd, baud, 8, 1, false, false);
  if (!setup)
  {
    if (!silent)
      fprintf(stderr, "failure, could not configure port.\n");
    exit(EXIT_FAILURE);
  }
  else
  {
    if (!silent)
      printf("success.\n");
  }
  int* fd_ptr = &fd;

  //time stamp for UAV data
  ros::Time imu_ts , Nav_ts , pose_ts, posenav_ts, gpsraw_ts, navvel_ts, sonar_ts, alt_ts, des_nav_ts, rosspin_ts, odom_pub_ts, mag_ts ,control_ts,radio_ts,ppid_ts,ppid2_ts,motor_ts;
  //initialize time
  imu_ts = Nav_ts = pose_ts = posenav_ts = gpsraw_ts = navvel_ts = sonar_ts = alt_ts = des_nav_ts = rosspin_ts = odom_pub_ts = mag_ts = ros::Time::now();

  int rosspin_count = 0;
  // ros::Time tomorrow = ros::Time::now() + ros::Duration(24*60*60);
  // ros::Duration negative_one_day = ros::Time::now() - tomorrow;

  time_start = ros::Time::now();
  ros::Time cur_time = time_start;

  //inertial_nav
  t_prev = micros();

  while (ros::ok())
  {
    serial_handle(fd_ptr);

    // r.sleep();
    cur_time = ros::Time::now();
    if (imu_ts != imuMsg.header.stamp) {
      imu_pub.publish(imuMsg);
      // Nav.publish(NavMsg);
      imu_ts = imuMsg.header.stamp;
      //try
      //state_out_pub.publish(//state_out);
      Acc_f[0] = imuMsg.linear_acceleration.x / 9.80655;
      Acc_f[1] = imuMsg.linear_acceleration.y / 9.80655;
      Acc_f[2] = imuMsg.linear_acceleration.z / 9.80655;

        //   //inertial_nav param
        GPS_hAcc = poseMsg.pose.covariance[0];
        GPS_vAcc = poseMsg.pose.covariance[0];
        hx = poseMsg.pose.pose.position.x;
        hy = poseMsg.pose.pose.position.y;
        GPS_vel_EAST = navvelMsg.twist.twist.linear.x*100;
        GPS_vel_NORTH = navvelMsg.twist.twist.linear.y*100;
        gps_vel_ned_valid = true;
        // ROS_INFO("pz:[%f]", data.pose.pose.position.z);
        GPS_ALIVE = true;
        _fix_ok = true;


      position_estimator();
      //ROS_INFO_STREAM("Inertial \t" << millis() << "\t" << GPS_vel_EAST);
      //prepare to send out inertial data
      inertial_gps.header.stamp = ros::Time::now();
      inertial_gps.header.frame_id = "/odom";
      inertial_gps.pose.pose.position.x = x_est[0]+2*gps_inertial_offset.x();
      inertial_gps.pose.pose.position.y = y_est[0]+2*gps_inertial_offset.y();
      //vel ? ? ?
      inertial_gps.pose.pose.position.z = altMsg.pose.pose.position.z;
      inertial_gps.pose.pose.orientation.x = imuMsg.orientation.x;
      inertial_gps.pose.pose.orientation.y = imuMsg.orientation.y;
      inertial_gps.pose.pose.orientation.z = imuMsg.orientation.z;
      inertial_gps.pose.pose.orientation.w = imuMsg.orientation.w;

      inertial_gps.pose.covariance[0]   = inertial_gps.pose.covariance[7] = GPS_hAcc;
      inertial_gps.pose.covariance[14]  = 0.01; //z position
      inertial_gps.pose.covariance[21] =  inertial_gps.pose.covariance[28] = inertial_gps.pose.covariance[35] = 0.001;
      inertial_gps_pub.publish(inertial_gps);



          //UPDATE DISTORT STATE TO USE IN MSF
          check_state();



    }
    if (pose_ts != poseMsg.header.stamp) {
      pose.publish(poseMsg);
      pose_ts = poseMsg.header.stamp;
    }
    if (posenav_ts != pose_navMsg.header.stamp) {
      posenav.publish(pose_navMsg);
      posenav_ts = pose_navMsg.header.stamp;

      //get data z to use scale slam
      baro_inertial_z_late = get_pose_nav_z_buffer();
    }
    if (gpsraw_ts != gpsrawMsg.header.stamp) {
      gpsraw.publish(gpsrawMsg);
      gpsraw_ts = gpsrawMsg.header.stamp;
    }
    if (navvel_ts != navvelMsg.header.stamp) {
      navvel.publish(navvelMsg);
      navvel_ts = navvelMsg.header.stamp;

      baro_alt_pub.publish(baroaltMsg);
    }
    // if (sonar_ts != sonarMsg.header.stamp) {
    //   sonar.publish(sonarMsg);
    //   sonar_ts = sonarMsg.header.stamp;
    // }
    if (alt_ts != altMsg.header.stamp) {
      alt.publish(altMsg);
      alt_ts = altMsg.header.stamp;
    }
    if (des_nav_ts != desire_navMSG.header.stamp) {
      des_nav.publish(desire_navMSG);
      des_nav_ts = desire_navMSG.header.stamp;
    }
    if (odom_pub_ts != odom_pose_filtered.header.stamp) {
      odom_pub.publish(odom_pose_filtered);
      odom_pub_ts = odom_pose_filtered.header.stamp;
    }
    if (mag_ts != magMsg.header.stamp) {
      mag_pub.publish(magMsg);
      mag_ts = magMsg.header.stamp;
    }
    if (control_ts != control_outMsg.header.stamp && DEBUG_CONTROL_OUT) {
      control_pub.publish(control_outMsg);
      control_ts = control_outMsg.header.stamp;
    }
    if (radio_ts != remoteMsg.header.stamp && DEBUG_RADIO_OUT) {
      radio_pub.publish(remoteMsg);
      radio_ts = remoteMsg.header.stamp;
    }
    if (ppid_ts != ppid_outMsg.header.stamp && DEBUG_PPID_OUT) {
      ppid_pub.publish(ppid_outMsg);
      ppid_ts = ppid_outMsg.header.stamp;
    }
    if (ppid2_ts != ppid2_outMsg.header.stamp && DEBUG_PPID2_OUT) {
      ppid2_pub.publish(ppid2_outMsg);
      ppid2_ts = ppid2_outMsg.header.stamp;
    }
    if (motor_ts != motor_outMsg.header.stamp && DEBUG_MOTOR_OUT) {
      motor_pub.publish(motor_outMsg);
      motor_ts = motor_outMsg.header.stamp;
    }

    //UPDATE STATUS
    static ros::Time low_task_stamp = cur_time;
    if(cur_time - low_task_stamp > ros::Duration(2) || status_updated) { //low priority task 0.5 hz 
        low_task_stamp = cur_time;

        status_updated=false;
        tf::Vector3 gps_off_ = 2*gps_inertial_offset;
        StatusMsg.header.stamp = cur_time;
        StatusMsg.GPS_OFFSET.x = gps_off_.x();
        StatusMsg.GPS_OFFSET.y = gps_off_.y();
        StatusMsg.GPS_OFFSET.z = gps_off_.z();



        tf::Vector3 vis_off_ = 2*vision_offset;
        StatusMsg.VISION_OFFSET.x = vis_off_.x();
        StatusMsg.VISION_OFFSET.y = vis_off_.y();
        StatusMsg.VISION_OFFSET.z = vis_off_.z();

        StatusMsg.GPS_IN_USING = GPS_STATE;
        StatusMsg.VISION_IN_USING = VISION_STATE;

        status_pub.publish(StatusMsg);
      
    }
    rosspin_count++;
    if (rosspin_count > 10) {
      // if(cur_time - rosspin_ts > ros::Duration(0.02))
      //  {
      ros::spinOnce();
      //   rosspin_ts = cur_time;
      // }
      rosspin_count = 0;
    }
    // r.sleep();



    /////////////////////////////////////////////////////////////////////////////////////////////////
    // currentTime = micros();


    // if (currentTime - hundredHZpreviousTime > 10000)
    // {

    //   G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
    //   if (G_Dt > 0.02) G_Dt = 0.01; //Prevent error from time peak to 1.83e13

    //   hundredHZpreviousTime = currentTime;

    //   Acc_f[0] = imuMsg.linear_acceleration.x / 9.80655;
    //   Acc_f[1] = imuMsg.linear_acceleration.y / 9.80655;
    //   Acc_f[2] = imuMsg.linear_acceleration.z / 9.80655;
    //   position_estimator();
    //   //ROS_INFO_STREAM("Inertial \t" << millis() << "\t" << GPS_vel_EAST);
    //   //prepare to send out inertial data
    //   inertial_gps.header.stamp = ros::Time::now();
    //   inertial_gps.header.frame_id = "/odom";
    //   inertial_gps.pose.pose.position.x = x_est[0];
    //   inertial_gps.pose.pose.position.y = y_est[0];
    //   inertial_gps.pose.pose.position.z = altMsg.pose.pose.position.z;
    //   inertial_gps.pose.pose.orientation.x = imuMsg.orientation.x;
    //   inertial_gps.pose.pose.orientation.y = imuMsg.orientation.y;
    //   inertial_gps.pose.pose.orientation.z = imuMsg.orientation.z;
    //   inertial_gps.pose.pose.orientation.w = imuMsg.orientation.w;

    //   inertial_gps.pose.covariance[0]   = inertial_gps.pose.covariance[7] = GPS_hAcc;
    //   inertial_gps.pose.covariance[14]  = 0.01; //z position
    //   inertial_gps.pose.covariance[21] =  inertial_gps.pose.covariance[28] = inertial_gps.pose.covariance[35] = 0.001;




    //   inertial_gps_pub.publish(inertial_gps);
    // }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    static bool flag_init_arduinoGPS = false;
    static bool flag_init_debug = false;
    static ros::Time t_sethome_wd = cur_time;
    static ros::Time t_setdebug_wd = cur_time;
    if (/*argc >= 5 && */!flag_init_arduinoGPS && gpsrawMsg.status.status == 1) //request to reset arduino GPS
    {
      if (ackMsg.ACK_SETHOME)
      {
        flag_init_arduinoGPS = true;
        printf("got ACK !! ALREADY set home-----------------------\n\n\n");
        t_sethome_wd = cur_time;
      }
      if (cur_time - t_sethome_wd > ros::Duration(0.2)) {
        send_reset_latlon(fd, 1);
        printf("\nRESET arduino LAT LON ---------------------------\n-------------------------------------------------\n-------------------------------------------------\n-------------------------------------------------\n-------------------------------------------------\n-------------------------------------------------\n");
        t_sethome_wd = cur_time;
      }
    }

    if(!flag_init_debug) {
      if(!ackMsg.ACK_DEBUG) {
        if (cur_time - t_setdebug_wd > ros::Duration(0.2)) {
          t_setdebug_wd = cur_time;
          send_debug_mode(fd);
          ROS_WARN_THROTTLE(2,"Sending debug mode %s %s %s %s %s ", (DEBUG_CONTROL_OUT ? "CONT":0)
                                                     , (DEBUG_PPID_OUT ? "PPID":0)
                                                     , (DEBUG_PPID2_OUT ? "PPID2":0)
                                                     , (DEBUG_RADIO_OUT ? "RADIO":0)
                                                     , (DEBUG_MOTOR_OUT ? "MOT":0)
                                                     );
        }
      }else{
        flag_init_debug = true;
        t_setdebug_wd = cur_time;
      }
    }
    // int c = 0;//getch();   // call your non-blocking input function
    // if (c == 'r' || c == 'R') {
    //   dynamic_reconfigure::ReconfigureRequest srv_req;
    //   dynamic_reconfigure::ReconfigureResponse srv_resp;
    //   dynamic_reconfigure::BoolParameter bool_param;
    //   dynamic_reconfigure::Config conf;

    //   bool_param.name = "core_init_filter";
    //   bool_param.value = true;
    //   conf.bools.push_back(bool_param);

    //   srv_req.config = conf;

    //   ros::service::call("/pose_from_compass_position_gps/position_pose_sensor/set_parameters", srv_req, srv_resp);
    // }

    //////////////////////////////SCALE SLAM////////////////////////////////////////////////////////
    static float alt_start_collect;
    static float slam_start_collect;
    static uint8_t count_collect = 0;
    static float scale_collect[SAMPLE];


    
    
    if (slam_stage == NOT_OK) count_collect = 0;
    if (slam_stage == READY_TO_INIT) {
      alt_start_collect = baro_inertial_z_late;
      slam_start_collect = z_slam;
      slam_stage = INITING;
      printf("START_COLLECT_DATA alt=%.2f\tslam=%.2f\n", alt_start_collect, slam_start_collect);
    }
    if (slam_stage == INITING && fabs(alt_start_collect - baro_inertial_z_late) > ALT_DIFF_SCALE) {
      //more than 20cm
      scale_slam = (fabs(slam_start_collect - z_slam) / fabs(alt_start_collect - baro_inertial_z_late));

      if (count_collect < SAMPLE) {
        scale_collect[count_collect] = scale_slam;
        if (count_collect == SAMPLE - 1) {
          slam_stage = READY_TO_USE;
          count_collect = 0;
        } else
        {
          slam_stage = READY_TO_INIT;
          printf("collect data #%d alt=%.2f\tslam=%.2f scale=%.2f\n", count_collect, baro_inertial_z_late, z_slam, scale_slam);
          count_collect++;
        }
      }

      if (slam_stage == READY_TO_USE) {
        scale_slam = 0;
        for (int i = 0; i < SAMPLE; i++) {
          scale_slam += scale_collect[i];
        }
        scale_slam /= SAMPLE;
        if (scale_slam == 0) {
          printf("WRONG INIT return ..."); slam_stage = READY_TO_INIT;
        }
        else {
          printf("END collect data alt=%.2f\tslam=%.2f\n", baro_inertial_z_late, z_slam);
          printf("\n Scale inited ----------------%.2f-----------------\n", scale_slam);
        }
      }

    }





  }


  // Run indefinitely while the ROS and serial threads handle the data
  if (!silent)
    printf("\nREADY, waiting for serial/ROS data.\n");

  int noErrors = 0;
  if (fd == -1 || fd == 0)
  {
    if (!silent)
      fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", port.c_str(), baud);
    exit(EXIT_FAILURE);
  }
  else
  {
    if (!silent)
      fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", port.c_str(), baud);
  }

  // FIXME ADD MORE CONNECTION ATTEMPTS

  if (fd == -1 || fd == 0)
  {
    exit(noErrors);
  }

  // Ready to roll
  printf("\nULINK SERIAL TO ROS BRIDGE STARTED ON MAV- RUNNING..\n\n");

  /**
     Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
  */
  ros::spin();

  close_port(fd);

  //g_thread_join(serial_thread);
  //exit(0);

  return 0;

}







inline float get_pose_nav_z_buffer() {
    // Try to pop acc for compensate delay 0.25 ms
  static uint16_t index = 0;
  static bool index_full = false;
  const int size_of_buffer = (delay_slam*50); //delay * navmsg_hz- > 50hz
  static float z_buffer[size_of_buffer];
  float z;
  //push
  z_buffer[index]=pose_navMsg.pose.pose.position.z;

  //check if full then pop
  if(index_full)
    z=z_buffer[(index+1)%size_of_buffer];
    //imuMsg.linear_acceleration.y = acc_buffer[(index+1)%size_of_buffer];
  else if(index==size_of_buffer-1)
    index_full=true;
  else 
    z=0;
  //update index
  index++;
  index=(index)%size_of_buffer;

  return z;
}
void odom_callback(const crossover_nav::odom_data::ConstPtr& a)
{
  // odom_pose_filtered.header = data.header;  //use compass to stamp
  // odom_pose_filtered.pose.pose.position = data.pose.pose.position;
  // odom_pose_filtered.pose.pose.orientation = data.pose.pose.orientation; //use compass instend

  crossover_nav::odom_data data = *a;
  static bool yaw_init = false;
  static float yaw_offset;
  static float yaw_offset_online_calibrate;
  static int sum_good = 0;
  float cov_tran, cov_orient;

  if (data.odom_state == 2) {
    if (sum_good < 40)
      sum_good++;
    if (slam_stage == READY_TO_USE) {
      cov_tran = max(cov_tran*0.95,VIS_POSITION_NOISE);
    } else {
      cov_tran = Min(cov_tran*1.15,10);
    }
  } else {
    if(sum_good<=0) sum_good=0;
    else          --sum_good;
    cov_tran = Min(cov_tran*1.15,10);
  }

  if(cov_tran>10) cov_tran = 10;
  if(cov_tran<VIS_POSITION_NOISE) cov_tran =VIS_POSITION_NOISE;

  //try 
  if(VISION_STATE) {
    cov_tran=VIS_POSITION_NOISE;
  }else{
    cov_tran=10;
  }

  //TODO  - depend it with number of tracked feature
  cov_orient = cov_tran*0.5;
  //Updata covariance
  odom_pose_filtered.pose.covariance[0] = cov_tran;
  odom_pose_filtered.pose.covariance[7] = cov_tran;
  odom_pose_filtered.pose.covariance[14] = cov_tran;
  odom_pose_filtered.pose.covariance[21] = cov_orient;
  odom_pose_filtered.pose.covariance[28] = cov_orient;
  odom_pose_filtered.pose.covariance[35] = cov_orient;




  //Prepare offset
  if (!yaw_init)
  {
    yaw_offset = YAW_IMU;
    request_offset = true;
    yaw_init = true;
    ROS_INFO_THROTTLE(2,"SET yaw init \t\t%f\n",yaw_offset);
  }//then reset when going completely lost
  if (yaw_init && data.odom_state == 1) {
    yaw_init = false;
    ROS_INFO_THROTTLE(2,"RESET yaw init \t\t%f\n",YAW_IMU);
  }
  //TODO check GPS covariace




  // printf("%d",sum_good);
  // static uint8_t bad_count = 0;
  //if data not collect enough .. nothing to do now. for prevent spike data
  if (sum_good < 10 && sum_good!=0) { //1->9
    slam_stage = NOT_OK;
    // printf("VISION NOW NOT OK %d \n",sum_good);
    return;
  }else if(sum_good==0 && slam_stage == READY_TO_USE) {
    slam_stage = READY_TO_INIT;
    // printf("VISION bad 10 time then reinits\n");
    return;
  }else if(sum_good >=10 && slam_stage == NOT_OK ){ //10->
    slam_stage = READY_TO_INIT;
  }
  

  // if(slam_stage==READY_TO_INIT &&  alt_change is)



  //ORIENTATION ROTATE ---------------------
  //note cam_to_vision give us rpy that cam rotated
  tf::Quaternion q_vc(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w);
  q_vc.normalize();


  //rebuild quaternion domain 120 -> -240  to align the same axis (vision axis(front z) to imu axis (front x))
  //construct form q(x,y,z,w)
  tf::Quaternion q_vc_rebuild(-q_vc.z(),
                               q_vc.x(),
                               q_vc.y(),
                              -q_vc.w());

  /* Use method ref http://ros-users.122217.n3.nabble.com/tf-and-quaternions-td2206128.html
  tf::Quaternion gyroscope_offset(tf::Vector3(0,0,1), M_PI/4.0);  // Axis/angle 
  tf::Quaternion robot_rotation = stransform * gyroscope_offset.inverse(); 
  */
  tf::Quaternion q_yaw_offset;
  q_yaw_offset.setRPY(0,0,wrap_pi(yaw_offset + yaw_offset_online_calibrate));
  tf::Quaternion q_vis = q_yaw_offset*q_vc_rebuild;

  //just for get raw yaw that cam is -- to calibrate yaw offset online -- not the same place at 
  tf::Matrix3x3 cam_to_vision_m(q_vc_rebuild);
  double r_,p_,y_;
  cam_to_vision_m.getRPY(r_,p_,y_);
  double _y_ = wrap_pi(y_ + yaw_offset);
  // ROS_INFO_STREAM(_y_ << "\t" << y_);
  y_=_y_;

  //Public to global -- > for switching conditioning
  YAW_SLAM_CALED = wrap_pi(y_+ yaw_offset_online_calibrate);
  // ROS_INFO_STREAM(YAW_SLAM_CALED << "\t" << YAW_IMU);


  //VECTOR ROTATE ------------------------
  tf::Quaternion q_pos(data.pose.pose.position.z, -data.pose.pose.position.x, -data.pose.pose.position.y,0);
  q_pos = q_yaw_offset*q_pos*q_yaw_offset.inverse();

  tf::Vector3 Translation_com_to_odom = q_pos.getAxis();






  //Send out slam z position to pre-scale
  z_slam = Translation_com_to_odom.z(); //save for scale

  if (slam_stage != READY_TO_USE) return;
  else {
    static tf::Vector3 slam_offset(0,0,0);
    if (request_offset /*&& gpsrawMsg.status.status==1*/) { // if gps available -> send offset;

      slam_offset = Translation_com_to_odom;
      x_msf_offset = state_out.data[0];
      y_msf_offset = state_out.data[1];
      msf_scale    = 1.0;//state_out.data[16];
      if (msf_scale == 0) msf_scale = 1;
      request_offset = false;

      ROS_INFO_STREAM("\n\n\n\n\nSLAM READY TO USE\t" << msf_scale << "\n\n\n\n\n");
    }

    vision_scaled = (Translation_com_to_odom - slam_offset)/scale_slam;
    // ROS_INFO_STREAM(vision_scaled.x() << "\t" << vision_scaled.y() <<"\t" << vision_scaled.z()<<"\t"  << vision_offset.x() <<  "\t"  << vision_offset.y() << "\n");


    odom_pose_filtered.header.stamp = ros::Time::now();

    odom_pose_filtered.pose.pose.position.x = vision_scaled.x() + 2*vision_offset.x();
    odom_pose_filtered.pose.pose.position.y = vision_scaled.y() + 2*vision_offset.y();
    odom_pose_filtered.pose.pose.position.z = vision_scaled.z() + 2*vision_offset.z();

    odom_pose_filtered.pose.pose.orientation.w = q_vis.w();
    odom_pose_filtered.pose.pose.orientation.x = q_vis.x();
    odom_pose_filtered.pose.pose.orientation.y = q_vis.y();
    odom_pose_filtered.pose.pose.orientation.z = q_vis.z();



    //ROS_INFO_STREAM(-p_ << "\t" << cyaw << "\t" << YAW_IMU << "\t" << yaw_offset_online_calibrate << "\n");
    //calculate yaw offset between yaw-cam to yaw-imu
    yaw_offset_online_calibrate = wrap_pi(yaw_offset_online_calibrate*0.99 + 0.01*(wrap_pi(YAW_IMU-y_)));



  }



}











void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data) {
  state_out = *data;
  //ROS_INFO_STREAM("state\t"  << state_out.data[4] << "\t"  << state_out.data[5] << "\n" );
  float x_ef = state_out.data[0];
  float y_ef = state_out.data[1];
  float z_ef = state_out.data[2];
  float vx_ef = state_out.data[3];
  float vy_ef = state_out.data[4];
  float vz_ef = state_out.data[5];

  
  send_msf_data(fd, x_ef, y_ef, z_ef, vx_ef, vy_ef, vz_ef);

}













inline float handle_offset(tf::Vector3& offset, const tf::Vector3& state_,const tf::Vector3& source_,bool flag) {
  tf::Vector3 source = source_;
  tf::Vector3 state = state_;

  source = source + offset;
  tf::Vector3 different = state-source;
  tf::Vector3 different2= different-offset;
  float distance = 1/invSqrt(different2.x()*different2.x()+different2.y()*different2.y());

  if(distance > HANDLE_OFF_RADIUS && flag) {
    offset =offset*(1-HANDLE_OFF_SPD) + HANDLE_OFF_SPD*(different);
    ROS_INFO_THROTTLE(0.2,"%f\t%f\t%f\n",different2.x(),different2.y(),distance);  //print every 200ms =5hz
    // gps_inertial_offset=offset;
  }

  
  return distance;
  // ROS_INFO_STREAM(gps_inertial_offset.x() << "\t" << gps_inertial_offset.y() << "\n");
  // ROS_INFO_STREAM(state.x() << "\t"  << state.y() << "\t"  << gps_filted.x() << "\t" << gps_filted.y() << "\t"  << gps_offset.x() << "\t"  << gps_offset.y() << "\n" );
}

inline bool MSF_IS_RUNNING() {
  static const float TIMEOUT_MSF = 1; //approx 0.05 -> 20 Hz from 100 hz mean drop for 4 msg 
  ros::Time cur = ros::Time::now();
  if(cur - state_out.header.stamp > ros::Duration(TIMEOUT_MSF)) return false;
  else                                                          return true;

}


inline void check_vision() {
  if(slam_stage==READY_TO_USE) 
    VISION_IS_READY=true;
  else 
    VISION_IS_READY=false;
}


inline void check_gps() {

  ROS_INFO_THROTTLE(0.5,"GPShacc = %f\n",GPS_hAcc);

  if(GPS_hAcc < GPS_HACC_CUTOFF) 
    GPS_IS_READY=true;
  else 
    GPS_IS_READY=false;
}


inline void check_state() {
  check_vision();
  check_gps();

  static bool msf_is_running = false;
  bool temp = MSF_IS_RUNNING();
  if(temp!=msf_is_running) { msf_is_running=temp; ROS_WARN("MSF_IS_NOW : %d\n", temp);}

  static tf::Vector3 state_msf(0,0,0);
  if(msf_is_running) {
    //if msf running get state value
    state_msf[0]=state_out.data[0];
    state_msf[1]=state_out.data[1];
    state_msf[2]=state_out.data[2];
  }
  //SELEECTOR------------------------------------------
  static bool v_flag=true;
  static bool gps_flag =true;
  if(VISION_IS_READY) {
    //WHEN VISION SCALED
    if(msf_is_running && !VISION_STATE) { //if state not init yet handle before use it
      //UPDATE OFFSET BEFOR USE IT----------------------
      //GET VALUE
      tf::Vector3 vision = vision_scaled;
      

        if(handle_offset(vision_offset, state_msf,  vision, v_flag) <= VIS_SWITCH_RADIUS && 
           fabs(wrap_pi(YAW_SLAM_CALED-YAW_IMU)) < VIS_SWITCH_ANGLE) { //RADIUS and ANGLE must be aligned 
          //reset flag
          v_flag = false;
          //ready to use state
          VISION_STATE=true;
          ROS_INFO("VISION OFFSET IS OK\n");
          //ready to publish new vis offset
          status_updated = true; 
        }




    }else{
      //WHEN VISION OK BUT MSF NOT RUNNING , PREPARE TO USE
      VISION_STATE=true;
    }
  }else{
    //WHEN VISION NOT READY TO USE
    VISION_STATE=false; 
    v_flag=true; //ready to offset if it comming again
  }
  //END FOR VISION

  if(GPS_IS_READY) {
    //WHEN GPS HDOP OK
    if(msf_is_running && !GPS_STATE) { //if state not init yet handle it first
    //UPDATE OFFSET -----------------------------------
    //GET VALUE
    tf::Vector3 gps(x_est[0],y_est[0],state_msf[2]);


      if(handle_offset(gps_inertial_offset,state_msf,gps,gps_flag) <= GPS_SWITCH_RADIUS) {
        //if gps is not init -> add gps_offset to force it know where's home (vision home)
        if(GPS_INERTIAL_INITED == false) { 
          gps_offset = gps_inertial_offset;
          gps_inertial_offset.setZero();
          GPS_INERTIAL_INITED = true;
        }
        
        //reset flag
        gps_flag=false;
        //ready to use state
        GPS_STATE = true;
        ROS_INFO("GPS OFFSET IS OK\n");
        //ready to publish new gps offset
        status_updated = true;  
      }

    }else{
      //WHEN GPS OK BUT MSF NOT RUNNING , PREPARE TO USE
      GPS_STATE=true;
    }
  }else{
    //WHEN GPS NOT READY TO USE
    GPS_STATE=false;
    gps_flag=true; //ready to offset if it comming again
  }
  //END FOR GPS





  //Selector distort topic services//////////////////////////////////
  if (ros::service::exists("/msf_distort/set_parameters", false)) //wait for service exist and not report on screen
    if (VISION_LAST_STATE != VISION_STATE || GPS_LAST_STATE != GPS_STATE) {
      //only when state changed this condition will meet
      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::BoolParameter bool_param;
      dynamic_reconfigure::Config conf;

      bool_param.name = "publish_pose";
      if (VISION_STATE)
      {
        ROS_WARN("ON slam");
        bool_param.value = true;
      }
      else
      {
        ROS_WARN("OFF slam");
        bool_param.value = false;
      }
      conf.bools.push_back(bool_param);


      bool_param.name = "publish_position";
      if (!GPS_STATE)
      {
        ROS_WARN("OFF GPS");
        bool_param.value = false;
      }
      else
      {
        ROS_WARN("ON GPS");
        bool_param.value = true;
      }

      conf.bools.push_back(bool_param);

      srv_req.config = conf;

      ros::service::call("/msf_distort/set_parameters", srv_req, srv_resp);

      //update last state
      VISION_LAST_STATE = VISION_STATE;
      GPS_LAST_STATE    = GPS_STATE;
      status_updated = true; //prepare to publish changed state
    }

  /////////////////////////////////////////////////////////////////////
}

// inline void gps_hacc_analysis() {
//   static const float freq = 5; //5hz use instead dt = 1/f (sec)
//   static const float dt = 1/freq;
//   static const float max_hacc=MAX_HACC_ALLOW;
//   static const float min_hacc=MIN_HACC_ALLOW;
//   static float score=0;
//   static float prev_hacc=GPS_hAcc;
//   static float grad_hacc=0;
  
//   //receive current hacc
//   float cur_hacc = GPS_hAcc;

//   //calculate gradient of hacc (+is incress uncertainty)  - apply smoothy 
//   grad_hacc = /*grad_hacc + (dt/(dt+TAU_GRAD_GPS)) * */(cur_hacc-prev_hacc);  
//   prev_hacc = cur_hacc;

//   //find slope way
//   //choice -- 1.use constant grad 2.use direct grad 
//   float justice = (grad_hacc==0 ? 0 : (grad_hacc > 0 ? MARGIN : -MARGIN));
//   score+=justice/cur_hacc;

//   //bad hacc score will max alway ~ min for high accuracy gps
//   if(cur_hacc > max_hacc) 
//     score=UPPER_LIMIT;
//   else if(cur_hacc < min_hacc)
//     score=LOWER_LIMIT;
//   else
//     score=min(Max(score,LOWER_LIMIT),UPPER_LIMIT);

//   ROS_WARN_THROTTLE(0.25,"grad=%f  GPS score = %f\tstatus  = %s",grad_hacc,score,(score>0 ? "BAD" : "GOOD"));
// }


inline void gps_hacc_analysis() {
  static const float freq = 5; //5hz use instead dt = 1/f (sec)
  static const float dt = 1/freq;
  static const float alpha = (dt/(dt+TAU_GRAD_GPS));
  static const float max_hacc=MAX_HACC_ALLOW;
  static const float min_hacc=MIN_HACC_ALLOW;
  static const float M = 1/(max_hacc-min_hacc);
  static float score=0;
  static float prev_hacc=GPS_hAcc;
  static float grad_hacc=0;
  
  //receive current hacc
  float cur_hacc = GPS_hAcc;

  //calculate gradient of hacc (+is incress uncertainty)  - apply smoothy 
  grad_hacc = /*(1-alpha)*grad_hacc + alpha**/(cur_hacc-prev_hacc);  
  prev_hacc = cur_hacc;

  //rate of score depend on hacc
  float D = 0;
  float hacc_constrained = min(Max(cur_hacc,min_hacc),max_hacc);
  if(grad_hacc>0) {
    D = M*(hacc_constrained-min_hacc);
  }else if (grad_hacc<0) {
    D = -M*(hacc_constrained-min_hacc) + 1;
  }

  //find slope way
  //choice -- 1.use constant grad 2.use direct grad 
  float justice = (grad_hacc==0 ? 0 : (grad_hacc > 0 ? MARGIN : -MARGIN));
  score+=(justice*(0.75*D + 0.25));

  //bad hacc score will max alway ~ min for high accuracy gps
  if(cur_hacc > max_hacc) 
    score=UPPER_LIMIT;
  else if(cur_hacc < min_hacc)
    score=LOWER_LIMIT;
  else
    score=min(Max(score,LOWER_LIMIT),UPPER_LIMIT);

  ROS_WARN_THROTTLE(0.25,"grad=%f  GPS score = %f\tstatus  = %s",grad_hacc,score,(score>0 ? "BAD" : "GOOD"));
}
// /*

// Just for note

// */
// Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx
// innovationSubset.setZero();

//  // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
//     Eigen::MatrixXd pht = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
//     Eigen::MatrixXd hphrInv  = (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();
//     kalmanGainSubset.noalias() = pht * hphrInv;


// /*it is 
// input--------------------
//   z-Hx
//   (HPH'+R)^-1
//   distance accpeted
// output--------------------
//   true/false"*/
//  if (checkMahalanobisThreshold(innovationSubset, hphrInv, measurement.mahalanobisThresh_))

//   // bla bla


//   bool checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
//                                              const Eigen::MatrixXd &invCovariance,
//                                              const double nsigmas)
//   {
//     double sqMahalanobis = innovation.dot(invCovariance * innovation);
//     double threshold = nsigmas * nsigmas;

//     if (sqMahalanobis >= threshold)
//     {
//       FB_DEBUG("Innovation mahalanobis distance test failed. Squared Mahalanobis is: " << sqMahalanobis << "\n" <<
//                "Threshold is: " << threshold << "\n" <<
//                "Innovation is: " << innovation << "\n" <<
//                "Innovation covariance is:\n" << invCovariance << "\n");

//       return false;
//     }

//     return true;
//   }