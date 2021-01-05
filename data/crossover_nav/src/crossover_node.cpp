#include <ros/ros.h>
#include <ros/time.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Range.h"


#include "mavros_msgs/Altitude.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/RCIn.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
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

#include <rovio/SrvResetToPose.h>

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
float GPS_velAcc = 10;
float GPS_vel_EAST = 0;
float GPS_vel_NORTH = 0;

tf::Quaternion Qwi_;

float R[3][3] = {{1, 0, 0},
{0, 1, 0},
{0, 0, 1}
};
float gyro_bias[3] = {0, 0, 0};
float acc_bias[3] = {0, 0, 0};

bool GPS_ALIVE = false;
bool _fix_ok = false;
bool gps_vel_ned_valid = false;

const float delay_slam  = 0.05;

float         vision_x[2], vision_y[2], vision_z[2];
float         vision_bx[2], vision_by[2], vision_bz[2];
unsigned long vision_time_stamp;
unsigned long last_vision_time;
bool          vision_is_good = false;
int           vision_topic_timeout = 500; //0.5 sec timeout

enum {
 NOT_OK = 0,
 READY_TO_INIT ,
 INITING,
 READY_TO_USE
};
uint8_t slam_stage = 0;

tf::Vector3 slam_before_scale(0,0,0);
float scale_slam = 1;


double DELAY_SLAM = 0;

#include "inertial_nav.h"

//global

sensor_msgs::Imu imuMsg;
sensor_msgs::MagneticField magMsg;
sensor_msgs::Joy remoteMsg;
sensor_msgs::FluidPressure baroMsg;
sensor_msgs::Temperature tempMsg;
sensor_msgs::NavSatFix gpsrawMsg, 
                       state_fixMsg;
sensor_msgs::Range sonarMsg;

geometry_msgs::PoseWithCovarianceStamped poseMsg, 
                                         inertial_gps, 
                                         odom_pose_filtered;
geometry_msgs::TwistWithCovarianceStamped navvelOdomMsg;
geometry_msgs::TwistStamped navvelMsg, 
                            inertial_gps_vel,
                            v_avoid_msgs;
geometry_msgs::PointStamped baroaltABSMsg;
double baroaltABSMsg_init;
// geometry_msgs::Vector3Stamped magMsg;
nav_msgs::Odometry pose_navMsg, msf_data, rovioMsg;

nav_msgs::Odometry altMsg, 
                   desire_navMSG, 
                   lpeMsg,
                   msf_errorMsg, 
                   ukf_errorMsg, 
                   inav_errorMsg,
                   ukfMsg;

mavros_msgs::Altitude baro_alt_msg;
mavros_msgs::State quad_stateMsg;
mavros_msgs::RCIn RCInMsg;

crossover_nav::Ack ackMsg;
crossover_nav::Navdata NavMsg;
crossover_nav::odom_data odom_data;
crossover_nav::Status StatusMsg;

sensor_fusion_comm::DoubleArrayStamped state_out;


bool GPS_STATE = false, GPS_LAST_STATE = true;
bool VISION_STATE = false, VISION_LAST_STATE = true;
bool VISION_IS_READY = false;
bool GPS_IS_READY = false;
bool FILTER_INIT_REQUEST = false;


double ROLL_IMU, PITCH_IMU, YAW_IMU;

Location GPS,GPS_HOME;


ros::Time  time_start;


bool msf_is_running = false;
bool request_offset = false;
float x_msf_offset, y_msf_offset;


float baro_inertial_z_late;
float baro_maf;

tf::Vector3 gps_offset(0,0,0);
tf::Vector3 gps_inertial_offset(0,0,0);
tf::Vector3 vision_offset(0,0,0);
tf::Vector3 vision_scaled(0,0,0);
bool status_updated = false;
double YAW_SLAM_CALED=0;
bool GPS_INERTIAL_INITED = false;

inline void check_state();
inline bool gps_hacc_analysis();


// #include "Ulink_capter_and_inertial_nav.h"





//param variable
double DELAY_NAV;
double HANDLE_OFF_SPD;
double HANDLE_OFF_RADIUS;
double GPS_SWITCH_RADIUS;
double VIS_SWITCH_RADIUS;
double VIS_SWITCH_ANGLE;
double GPS_HACC_CUTOFF;
double ALT_DIFF_SCALE;
int NUM_SAMPLE_PRESCALE=6;
// VISION NOISE
double VIS_POSITION_NOISE;
double VIS_ORIENTATION_NOISE;
double VIS_SCALE_ACCEPTION;
// GPS analysis
double TAU_GRAD_GPS;
double MARGIN;
double LOWER_LIMIT;
double UPPER_LIMIT;
double MAX_HACC_ALLOW;
double MIN_HACC_ALLOW;
// Other
double CAMERA_PITCH;
double HOME_OFF_x;
double HOME_OFF_y;
bool REALSENSE_ENABLE=false;
double MIN_AGL_AVOID=0.5;

enum {
  px=0,
  py,
  pz,
  vx,
  vy,
  vz,
  qw,
  qx,
  qy,
  qz,
  b_wx,
  b_wy,
  b_wz,
  b_ax,
  b_ay,
  b_az,
  L,
  pwvx,
  pwvy,
  pwvz,
  bp,
  qifw,
  qifx,
  qify,
  qifz,
  tz,
  sizestate
};

//////////////////////////////////////////////////////////
tf::Vector3 PwiBuff(const tf::Vector3& Pwi_in);
tf::Quaternion QwiBuff(const tf::Quaternion& Qwi_in);
bool PwiIsReady = false;
bool QwiIsReady = false;
tf::Vector3 Pwi_late;
tf::Quaternion Qwi_late;
tf::Quaternion Qwi;
bool Twv_Inited = false;
//////////////////////////////////////////////////////////

//void msf_callback(const nav_msgs::Odometry& data);
inline float get_pose_nav_z_buffer(float z_estimate);
void DoSetHome();

//ROS topic callback
void odom_callback(const crossover_nav::odom_data::ConstPtr& data);
void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data);
void imu_callback(const sensor_msgs::Imu::ConstPtr& data);
void baro_alt_callback(const sensor_msgs::FluidPressure::ConstPtr& data);
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& data);
void gps_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& data);
void quad_state_callback(const mavros_msgs::State::ConstPtr& data);
void lpe_callback(const nav_msgs::Odometry::ConstPtr& data);
void rcin_callback(const mavros_msgs::RCIn::ConstPtr& data);
void rovio_callback(const nav_msgs::Odometry::ConstPtr& data);
void v_avoid_callback(const geometry_msgs::TwistStamped::ConstPtr& data);
// void ukf_callback(const nav_msgs::Odometry::ConstPtr& data) {
//   ukfMsg = *data;
// }

//////////////////////////////////////////////////////////
enum {
  NONE = 0,
  INDOOR = 1,
  OUTDOOR = 2
};
int ORIGIN=NONE;
static bool Privilege_INDOOR = false;
inline bool IS_ORIGIN_SET();
inline void CHECK_ORIGIN() ;
///////////////////////////////////////////////////////////

inline bool MSF_IS_RUNNING();
void map_projector(Location _GPS, Location _GPS_HOME, double *x, double *y);
void map_reveseprojector(Location *_GPS, Location _GPS_HOME, double x, double y);
inline void slam_prescale(ros::Time cur_time);
inline void Init_filter();
inline void Distort_Service();

ros::Publisher imu_pub ;//
ros::Publisher pose ;//e<g
ros::Publisher navvel ;//e
ros::Publisher posenav ;//
ros::Publisher gpsraw ;//e
ros::Publisher sonar ;//e<
ros::Publisher Nav ;//e<be
ros::Publisher alt ;//e<na
ros::Publisher des_nav ;//
ros::Publisher mag_pub ;//
ros::Publisher inertial_gps_pub ;// 
ros::Publisher inertial_gps_vel_pub;
ros::Publisher baro_alt_abs_pub ;// 
ros::Publisher status_pub ;// n.adve
ros::Publisher odom_pub ;// n.adverti
ros::Publisher state_fix_pub ;// n.a
ros::Publisher state_out_pub ;// n.ad
ros::Publisher msf_error_pub ;// n.a
ros::Publisher ukf_error_pub ;// n.a
ros::Publisher inav_error_pub ;// n.
ros::Publisher vis_status_pub ;// n.
ros::Publisher slam_reset_pub ;// n.a
ros::Publisher local_pos_pub;
ros::ServiceClient clientrovio;


/////////////////////////////////////
///////////OFFBOARD EIGHT PATH///////
bool RESET_POSE_INIT_REQUEST = true;
void DRIVE_PATH(bool READY );
enum {
  DRIVE_EIGHT_EQUATION=0
};

bool OBV_IS_ENABLED = false;
bool OBV_IS_READY = false;
inline bool IS_AGL_NOT_TOO_LOW() {
  return (state_out.data[pz]-state_out.data[tz] > MIN_AGL_AVOID ? true:false);
}
inline void FIND_OBSTACLE();
tf::Vector3 v_avoid;


int main(int argc, char **argv)
{

  //init ros node
  ros::init(argc, argv, "quad_talker");
  ros::NodeHandle n;
  ros::Rate r(400);


  //variable need 
  std::string s;
  //retreve param
  // n.param<std::string>("DEVICE", port, "/dev/ttyACM0");
  // n.param("BAUDRATE", baud, 115200);
  n.param("DELAY_NAV", DELAY_NAV, 0.25);
  n.param("HANDLE_OFF_SPD", HANDLE_OFF_SPD, 0.05);
  n.param("HANDLE_OFF_RADIUS", HANDLE_OFF_RADIUS, 0.15);
  n.param("GPS_SWITCH_RADIUS", GPS_SWITCH_RADIUS, 0.5);
  n.param("VIS_SWITCH_RADIUS", VIS_SWITCH_RADIUS, 0.5);
  n.param("VIS_SWITCH_ANGLE", VIS_SWITCH_ANGLE, 0.1);
  n.param("GPS_HACC_CUTOFF", GPS_HACC_CUTOFF, 4.0);
  n.param("ALT_DIFF_SCALE", ALT_DIFF_SCALE, 0.1);
  n.param("NUM_SAMPLE_PRESCALE", NUM_SAMPLE_PRESCALE, 6);

  n.param("VIS_POSITION_NOISE", VIS_POSITION_NOISE, 0.1);
  n.param("VIS_ORIENTATION_NOISE", VIS_ORIENTATION_NOISE, 0.1);
  n.param("VIS_SCALE_ACCEPTION", VIS_SCALE_ACCEPTION, 0.05);

  n.param("TAU_GRAD_GPS", TAU_GRAD_GPS,0.22);
  n.param("MARGIN", MARGIN, 1.0);
  n.param("LOWER_LIMIT", LOWER_LIMIT, -5.0);
  n.param("UPPER_LIMIT", UPPER_LIMIT, 5.0);
  n.param("MAX_HACC_ALLOW", MAX_HACC_ALLOW, 6.0);
  n.param("MIN_HACC_ALLOW", MIN_HACC_ALLOW, 2.8);

  n.param("CAMERA_PITCH", CAMERA_PITCH, 0.0);
  n.param("HOME_OFF_x", HOME_OFF_x, 0.0);
  n.param("HOME_OFF_y", HOME_OFF_y, 0.0);
  n.param("HOME_LAT_NO_GPS", GPS_HOME.lat, 7.0064177);
  n.param("HOME_LNG_NO_GPS", GPS_HOME.lng, 100.5025051);

  n.param("REALSENSE_ENABLE", REALSENSE_ENABLE, false);
  n.param("MIN_AGL_AVOID", MIN_AGL_AVOID, 0.5);

  // n.param("DEBUG_CONTROL_OUT" , DEBUG_CONTROL_OUT, false);
  // n.param("DEBUG_PPID_OUT"    , DEBUG_PPID_OUT, false);
  // n.param("DEBUG_PPID2_OUT"   , DEBUG_PPID2_OUT, false);
  // n.param("DEBUG_RADIO_OUT"   , DEBUG_RADIO_OUT, false);
  // n.param("DEBUG_MOTOR_OUT"   , DEBUG_MOTOR_OUT, false);
  // n.param("SWITCH_RADIUS", baud, 115200);

  // saytab("--- DEVICE  \t\t\t\t\t"); sayend(port);
  // saytab("--- BAUDRATE  \t\t\t\t\t"); sayend(baud);
  saytab("--- _delay_gps  \t\t\t\t\t"); sayend(DELAY_NAV);
  saytab("--- HANDLE_OFF_SPD  \t\t\t"); sayend(HANDLE_OFF_SPD);
  saytab("--- GPS_SWITCH_RADIUS  \t\t\t\t"); sayend(GPS_SWITCH_RADIUS);
  saytab("--- VIS_SWITCH_RADIUS  \t\t\t\t"); sayend(VIS_SWITCH_RADIUS);
  saytab("--- VIS_SWITCH_ANGLE  \t\t\t\t"); sayend(VIS_SWITCH_ANGLE);
  saytab("--- GPS_HACC_CUTOFF  \t\t\t\t"); sayend(GPS_HACC_CUTOFF);
  saytab("--- ALT_DIFF_SCALE  \t\t\t\t"); sayend(ALT_DIFF_SCALE);

  _delay_gps = DELAY_NAV;

  //publisher
  // imu_pub = n.advertise<sensor_msgs::Imu>("/imu_max", 10);
  pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 10);      ///GPS already offset
  navvel = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/imu_max/Navvel", 10); ///GPS body vel for ukf
  // posenav = n.advertise<nav_msgs::Odometry>("/imu_max/pose_nav", 10);
  // gpsraw = n.advertise<sensor_msgs::NavSatFix>("/imu_max/Gpsraw", 10);
  // sonar = n.advertise<sensor_msgs::Range>("/sonar", 10);
  // Nav = n.advertise<crossover_nav::Navdata>("/imu_max/Navdata", 10);
  // alt = n.advertise<nav_msgs::Odometry>("/imu_max/alt_odometry", 10);
  // des_nav = n.advertise<nav_msgs::Odometry>("/imu_max/pose_des", 10);
  // mag_pub = n.advertise<sensor_msgs::MagneticField>("/imu_max/mag", 10);


  
  //inertial nav
   inertial_gps_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/Inertial_nav", 10);
   inertial_gps_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/imu_max/Inertial_nav_vel", 10);
   baro_alt_abs_pub = n.advertise<geometry_msgs::PointStamped>("/imu_max/baro_alt", 10);


  // offset pub
   status_pub = n.advertise<crossover_nav::Status>("/imu_max/status",1);


  //odom filtered
  odom_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/odom_pose_filtered", 10);

  //filter to NavSatFix
   state_fix_pub = n.advertise<sensor_msgs::NavSatFix>("/imu_max/state_fix", 10);
  //state out pub
  //state_out_pub = n.advertise<sensor_fusion_comm::ExtEkf>("/state_out2",10);

   local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_position/local", 10);

  //  msf_error_pub = n.advertise<nav_msgs::Odometry>("/error_odom/msf", 10);
  //  ukf_error_pub = n.advertise<nav_msgs::Odometry>("/error_odom/ukf", 10);
  //  inav_error_pub = n.advertise<nav_msgs::Odometry>("/error_odom/inav", 10);

  //Publish status as velocity/twist to mavros to indicate companion computer status eg. vision, ekf, etc...
   vis_status_pub = n.advertise<geometry_msgs::TwistStamped>
    ("/mavros/vision_speed/speed_twist", 10);

  slam_reset_pub = n.advertise<std_msgs::Bool>
    ("/slam_reset", 10);

  //subscriber
  // ros::Subscriber msf_sub = n.subscribe<nav_msgs::Odometry>("/msf_core/odometry", 10,msf_callback);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_callback);
  ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 10, gps_callback);
  ros::Subscriber alt_sub = n.subscribe<sensor_msgs::FluidPressure>("/mavros/imu/atm_pressure", 30, baro_alt_callback);
  ros::Subscriber gps_vel_sub = n.subscribe<geometry_msgs::TwistStamped>("/mavros/global_position/raw/gps_vel", 10, gps_vel_callback);
  ros::Subscriber quad_state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, quad_state_callback);
  ros::Subscriber lpe_sub = n.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, lpe_callback);
  ros::Subscriber odom_sub = n.subscribe<crossover_nav::odom_data>("/slam", 10, odom_callback);
  ros::Subscriber state_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 10, state_out_callback);
  ros::Subscriber rcin_sub = n.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, rcin_callback);
  ros::Subscriber rovio_sub = n.subscribe<nav_msgs::Odometry>("/rovio/odometry", 10, rovio_callback);
  ros::Subscriber v_avoid_sub = n.subscribe<geometry_msgs::TwistStamped>("/v_avoid", 10, v_avoid_callback);

  clientrovio = n.serviceClient<rovio::SrvResetToPose>("rovio/reset_to_pose");
  // ros::Subscriber ukf_sub = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, ukf_callback);


  




  for (int i = 0; i < 36; i++)
  {
    poseMsg.pose.covariance[i] = 0;
  }
  state_out.data.resize(sizestate);

  RCInMsg.channels.resize(8);


  odom_pose_filtered.header.frame_id = "odom";

  StatusMsg.header.frame_id="odom";


  //time stamp for UAV data
  ros::Time imu_ts, 
            Nav_ts, 
            pose_ts, 
            posenav_ts, 
            gpsraw_ts, 
            navvel_ts, 
            sonar_ts, 
            alt_ts, 
            des_nav_ts, 
            rosspin_ts, 
            odom_pub_ts, 
            mag_ts,
            baroaltABS_ts,
            state_fix_ts,
            v_avoid_ts;
  //initialize time
  imu_ts = 
  Nav_ts = 
  pose_ts = 
  posenav_ts = 
  gpsraw_ts = 
  navvel_ts = 
  sonar_ts = 
  alt_ts = 
  des_nav_ts = 
  rosspin_ts = 
  odom_pub_ts = 
  mag_ts = 
  state_fix_ts = 
  baroaltABS_ts =
  v_avoid_ts =
  ros::Time::now();

  int rosspin_count = 0;
  // ros::Time tomorrow = ros::Time::now() + ros::Duration(24*60*60);
  // ros::Duration negative_one_day = ros::Time::now() - tomorrow;

  time_start = ros::Time::now();
  ros::Time cur_time = time_start;

  //inertial_nav
  // t_prev = micros();

  sayend("Initilizing..");
  while (ros::ok())
  {
    // serial_handle(fd_ptr);
    r.sleep();
    cur_time = ros::Time::now();
    if (cur_time - imu_ts > ros::Duration(0.01)) { //fix at 100hz
      // imu_pub.publish(imuMsg);
      // Nav.publish(NavMsg);
      imu_ts = cur_time;

      GPS_hAcc = (gpsrawMsg.position_covariance[0] > 0 ? gpsrawMsg.position_covariance[0]:999);
      GPS_velAcc = (gpsrawMsg.position_covariance[8] > 0 ? gpsrawMsg.position_covariance[8]:999);



      inertial_gps.header.stamp = imuMsg.header.stamp;
      inertial_gps.header.frame_id = "/odom";
      inertial_gps.pose.pose.position.x = 0;//x_est[0];//+2*gps_inertial_offset.x();
      inertial_gps.pose.pose.position.y = 0;//y_est[0];//+2*gps_inertial_offset.y();
      inertial_gps.pose.pose.position.z =  estimated_altitude;
      inertial_gps.pose.pose.orientation = imuMsg.orientation;

      inertial_gps.pose.covariance[0]   = inertial_gps.pose.covariance[7] = GPS_hAcc;
      inertial_gps.pose.covariance[14]  = 0.01; //z position
      inertial_gps.pose.covariance[21] =  inertial_gps.pose.covariance[28] = inertial_gps.pose.covariance[35] = 0.001;
      inertial_gps_pub.publish(inertial_gps);

      // inertial_gps_vel.header.stamp = ros::Time::now();
      // inertial_gps_vel.header.frame_id = "/odom";
      // inertial_gps_vel.twist.linear.x = x_est[1];
      // inertial_gps_vel.twist.linear.y = y_est[1];
      // inertial_gps_vel.twist.linear.z = baro_inertial_z_late;
      // inertial_gps_vel_pub.publish(inertial_gps_vel);

          //UPDATE DISTORT STATE TO USE IN MSF
      check_state();
    }
    if (pose_ts != poseMsg.header.stamp) {
      geometry_msgs::PoseWithCovarianceStamped poseAndOffsetMsg;
      poseAndOffsetMsg = poseMsg;
      poseAndOffsetMsg.pose.pose.position.x += 2*gps_offset.x();
      poseAndOffsetMsg.pose.pose.position.y += 2*gps_offset.y();
      //instead cutoff we add large std to force state estimation ignore the signal
      if(!GPS_STATE) {
        poseAndOffsetMsg.pose.covariance[0]=poseAndOffsetMsg.pose.covariance[7]=poseAndOffsetMsg.pose.covariance[21]=
        poseAndOffsetMsg.pose.covariance[28]=999;
      }
      pose.publish(poseAndOffsetMsg);
      pose_ts = poseMsg.header.stamp;
    }
    if(state_fix_ts != state_out.header.stamp) {
      state_fix_ts = state_out.header.stamp;
      state_fixMsg.header = state_out.header;
      Location GPS_F;
      //move current position or move home is the same manner.
      float ln_in_m = state_out.data[px] - 2*gps_offset.x();
      float lat_in_m = state_out.data[py] - 2*gps_offset.y();
      map_reveseprojector(&GPS_F,GPS_HOME,
                          ln_in_m,
                          lat_in_m);
      state_fixMsg.longitude = GPS_F.lng;
      state_fixMsg.latitude = GPS_F.lat;
      state_fix_pub.publish(state_fixMsg);
    }
    if(baroaltABS_ts != baroaltABSMsg.header.stamp) {
      baroaltABS_ts = baroaltABSMsg.header.stamp;
      baro_alt_abs_pub.publish(baroaltABSMsg);
    }
    // if (posenav_ts != pose_navMsg.header.stamp) {
    //   posenav.publish(pose_navMsg);
    //   posenav_ts = pose_navMsg.header.stamp;
    // }
    // if (gpsraw_ts != gpsrawMsg.header.stamp) {
    //   gpsraw.publish(gpsrawMsg);
    //   gpsraw_ts = gpsrawMsg.header.stamp;
    // }
    if (navvel_ts != navvelOdomMsg.header.stamp) {
      //convert twist msg to body-frame due to ROS REF
      tf::Quaternion q_vec(navvelMsg.twist.linear.x,
                           navvelMsg.twist.linear.y,
                           navvelMsg.twist.linear.z,
                           0);
      q_vec = Qwi_.inverse() * q_vec * Qwi_;
      tf::Vector3 v_body = q_vec.getAxis();
      navvelOdomMsg.twist.twist.linear.x = v_body.x();
      navvelOdomMsg.twist.twist.linear.y = v_body.y();
      navvelOdomMsg.twist.twist.linear.z = v_body.z();
      navvelOdomMsg.twist.covariance[0] = navvelOdomMsg.twist.covariance[7] = GPS_hAcc;
      navvel.publish(navvelOdomMsg);
      navvel_ts = navvelOdomMsg.header.stamp;
    }
    // if (sonar_ts != sonarMsg.header.stamp) {
    //   sonar.publish(sonarMsg);
    //   sonar_ts = sonarMsg.header.stamp;
    // }
    // if (alt_ts != altMsg.header.stamp) {
    //   alt.publish(altMsg);
    //   alt_ts = altMsg.header.stamp;
    // }
    // if (des_nav_ts != desire_navMSG.header.stamp) {
    //   des_nav.publish(desire_navMSG);
    //   des_nav_ts = desire_navMSG.header.stamp;
    // }
    if (odom_pub_ts != odom_pose_filtered.header.stamp) {
      odom_pub.publish(odom_pose_filtered);
      odom_pub_ts = odom_pose_filtered.header.stamp;
    }
    // if (mag_ts != magMsg.header.stamp) {
    //   mag_pub.publish(magMsg);
    //   mag_ts = magMsg.header.stamp;
    // }










    if(v_avoid_ts != v_avoid_msgs.header.stamp) {
      v_avoid_ts = v_avoid_msgs.header.stamp;

      if(REALSENSE_ENABLE)
      {
        OBV_IS_ENABLED = true;

        if(OBV_IS_ENABLED && IS_AGL_NOT_TOO_LOW()) {
          OBV_IS_READY=true;
        }else{
          OBV_IS_READY=false;
        }

        if(OBV_IS_READY)
          FIND_OBSTACLE();

      }else
      {
        OBV_IS_ENABLED = false;
        OBV_IS_READY = false;
      }

    }

    //AVOID CHECK TIMEOUT
    static ros::Time depth_task_stamp = cur_time;

    if(cur_time - v_avoid_ts > ros::Duration(0.1)) { //Depth from realsense task 30 hz 
      depth_task_stamp = cur_time;
      OBV_IS_ENABLED = false;
      OBV_IS_READY = false;
    }




    //UPDATE STATUS
    static ros::Time low_task_stamp = cur_time;

    if(cur_time - low_task_stamp > ros::Duration(2) || status_updated) { //low priority task 0.5 hz    
      low_task_stamp = cur_time;

      status_updated=false;
      tf::Vector3 gps_off_ = 2*gps_offset;
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
      StatusMsg.SCALE = state_out.data[L];
      StatusMsg.MSF = msf_is_running;

      status_pub.publish(StatusMsg);



      static mavros_msgs::State last_state = quad_stateMsg;

      if( quad_stateMsg.armed != last_state.armed || 
        quad_stateMsg.mode != last_state.mode) {

        ROS_INFO_STREAM("WAIT OFFB now " 
          << (quad_stateMsg.armed ? "" : "DIS")
          << "ARMED\t--" 
          << quad_stateMsg.mode 
          );
        last_state= quad_stateMsg;
      }

    }

    static ros::Time tenhz_task = cur_time;
    if(cur_time - tenhz_task > ros::Duration(0.1)) {
      tenhz_task = cur_time;
      CHECK_ORIGIN();
      geometry_msgs::TwistStamped vis_vel;

      vis_vel.header.stamp = imuMsg.header.stamp;
      if(VISION_STATE)
      {  
        vis_vel.twist.linear.y = 2.5;
      }else{
        vis_vel.twist.linear.y = 0.5;
      }

      if(slam_stage == READY_TO_INIT || slam_stage == INITING) {
        vis_vel.twist.linear.y = 1.5;
        ROS_INFO_THROTTLE(5,"Slam ready wait MSF to prescale...");
      }

      if(msf_is_running) {
        vis_vel.twist.linear.x = -1.0;
      }else{
        vis_vel.twist.linear.x = 0.0;
      }
      vis_status_pub.publish(vis_vel);


      if( quad_stateMsg.mode == "OFFBOARD") {
        /* use bool for utility other check state eg.
         battery voltage , load cpu etc*/
        DRIVE_PATH(true);
      }
      else
      {
        /* we must continue to send msg although not offb mode */
        RESET_POSE_INIT_REQUEST=true;
        float des_hx_init = lpeMsg.pose.pose.position.x;
        float des_hy_init = lpeMsg.pose.pose.position.y;
        float des_hz_init = lpeMsg.pose.pose.position.z;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = des_hx_init;
        pose.pose.position.y = des_hy_init;
        pose.pose.position.z = des_hz_init;

        //send a few setpoints before starting "DO THIS EVERY TIME BEFORE OFFBOARD"
        for(int i = 4; ros::ok() && i > 0; --i){
          local_pos_pub.publish(pose);
        }
      }
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


    //////////////////////////////SCALE SLAM////////////////////////////////////////////////////////
    // slam_prescale(cur_time);

    ///////////////////////////////TIMEOUT CHECK////////////////////////////////////////////////////
        // SLAM Timeout check if not using rovio
    if(cur_time - rovioMsg.header.stamp > ros::Duration(1.0)
      && cur_time - odom_data.header.stamp > ros::Duration(1.0))
    {
        slam_stage = NOT_OK;
        VISION_IS_READY = false;
        VISION_STATE = false;
    }
    
    // If quad armed but msf is not running.. try to re init
    static ros::Time msf_timeout_stamp = cur_time;
    if(cur_time - msf_timeout_stamp > ros::Duration(1.0) && quad_stateMsg.armed && !msf_is_running) {
      msf_timeout_stamp = cur_time;
      FILTER_INIT_REQUEST=true;
    }
    //Check if ready to init (should wait for GPS and VISION information to initialize)
    if(FILTER_INIT_REQUEST && (ORIGIN==INDOOR || VISION_IS_READY || GPS_IS_READY)) {
      static uint16_t countdown = 0;
      if(countdown>2000) {
        Init_filter();
        countdown = 0;
        FILTER_INIT_REQUEST = false;
      }else countdown ++ ;
    }

    //Check if state unstable (high velocity, highcov?)
    bool MSF_BAD_BEHAVIOR = ((fabs(state_out.data[vx])+fabs(state_out.data[vy])+fabs(state_out.data[vz]))>20.0 ? true:false );
    if(MSF_BAD_BEHAVIOR && (VISION_IS_READY || GPS_IS_READY)) {
        Init_filter();
    }


  }

  return 0;

}

inline bool IS_ORIGIN_SET() {
  //already SET = true
  return (ORIGIN==NONE ? false:true);
}
inline void CHECK_ORIGIN() {
  int status = 0;
//  printf("check origin msf %d status %d GPS %d slam stage %d\n", msf_is_running, status, GPS_IS_READY, slam_stage);
  if(!msf_is_running) {
    if(GPS_IS_READY) {
        status = OUTDOOR; 
    }else{
      if(slam_stage>=READY_TO_INIT) {
        status = INDOOR; 
      }
    }
  }
  if(!IS_ORIGIN_SET() && status!=0) {
    ORIGIN=status; 
    if(ORIGIN==INDOOR) Privilege_INDOOR=true;
    printf("\n\n------------SET AS %s ORIGIN-----------------\n\n", (ORIGIN==INDOOR ? "INDOOR":"OUTDOOR"));
  }
  return;
}



inline void slam_prescale(ros::Time cur_time) {

    static float alt_start_collect;
    static float slam_start_collect;
    static uint8_t count_collect = 0;
    static float scale_collect[50];
    static bool flag_reset_slam = false;

    
    
    if (slam_stage == NOT_OK) count_collect = 0;
    if (slam_stage == READY_TO_INIT && msf_is_running) {
      alt_start_collect = baro_inertial_z_late;
      slam_start_collect = slam_before_scale.z();
      slam_stage = INITING;
      printf("START_COLLECT_DATA alt=%.2f\tslam=%.2f\n", alt_start_collect, slam_start_collect);
    }
    if (slam_stage == INITING && fabs(alt_start_collect - baro_inertial_z_late) > ALT_DIFF_SCALE) {
      //more than 20cm
      scale_slam = (fabs(slam_start_collect - slam_before_scale.z()) / fabs(alt_start_collect - baro_inertial_z_late));

      if (count_collect < NUM_SAMPLE_PRESCALE) {
        scale_collect[count_collect] = scale_slam;
        if (count_collect == NUM_SAMPLE_PRESCALE - 1) {
          slam_stage = READY_TO_USE;
          count_collect = 0;
        } else {
          slam_stage = READY_TO_INIT;
          printf("collect data #%d alt=%.2f\tslam=%.2f scale=%.2f\n", count_collect, baro_inertial_z_late, slam_before_scale.z(), scale_slam);
          count_collect++;
        }
      }

      if (slam_stage == READY_TO_USE) {
        scale_slam = 0;
        for (int i = 0; i < NUM_SAMPLE_PRESCALE; i++) {
          scale_slam += scale_collect[i];
        }
        scale_slam /= NUM_SAMPLE_PRESCALE;
        if (scale_slam <= VIS_SCALE_ACCEPTION) {
          printf("WRONG INIT reset request ...\n"); 
          slam_stage = NOT_OK;
          flag_reset_slam = true;
        }else {
          printf("END collect data alt=%.2f\tslam=%.2f\n", baro_inertial_z_late, slam_before_scale.z());
          printf("\n Scale inited ----------------%.2f-----------------\n", scale_slam);
        }
      }

    }
    
    //update slam reset topic every 2hz
    static ros::Time reset_slam_stamp = cur_time;
    if(cur_time - reset_slam_stamp > ros::Duration(0.5)) {
      reset_slam_stamp = cur_time;
      std_msgs::Bool slam_resetMsg;
      if(flag_reset_slam) {slam_resetMsg.data = true; 
                           flag_reset_slam=false;}
                           
      else                slam_resetMsg.data = false;
      slam_reset_pub.publish(slam_resetMsg);
    }

}






inline float get_pose_nav_z_buffer(float z_estimate) 
{
  // Try to pop acc for compensate delay 0.25 ms
  static uint16_t index = 0;
  static bool index_full = false;
  const int size_of_buffer = (delay_slam*68); //delay * navmsg_hz- > 18hz
  static float z_buffer[size_of_buffer];
  float z;
  //push
  z_buffer[index]=z_estimate;

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
  odom_data = *a;
  crossover_nav::odom_data data = *a;
  static float yaw_offset;
  static float roll_offset;
  static float pitch_offset;

  static int sum_good = 0;
  float cov_tran, cov_orient;

  if (data.odom_state == 2) {
    if (sum_good < 40)
      sum_good++;
    if (slam_stage == READY_TO_USE) {
      cov_tran = Max(cov_tran*0.95,VIS_POSITION_NOISE);
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

  if(VISION_STATE && GPS_STATE) {
    cov_tran=VIS_POSITION_NOISE*2*GPS_HACC_CUTOFF/Min(GPS_HACC_CUTOFF,GPS_hAcc);
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

  //--------------------------------------------------------


  tf::Quaternion pose_vc(data.pose.pose.position.x, 
                           data.pose.pose.position.y, 
                           data.pose.pose.position.z,
                           0);
  tf::Quaternion q_vc_rebuild(data.pose.pose.orientation.x, 
                              data.pose.pose.orientation.y, 
                              data.pose.pose.orientation.z, 
                              data.pose.pose.orientation.w);
  q_vc_rebuild.normalize();



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
  



// NOTE : 
// STATE MACHINE
//  NOT_OK = 0,  -> lost and waiting to reset
//  READY_TO_INIT , --> tracked but wait for scaling 
//  INITING --> prepare for 
//  READY_TO_USE -->scalling 
// };
  static tf::Vector3 Pwv0(0,0,0);
  static tf::Quaternion Qwv0;


  //working with VIN init system-----------------------------
  if(slam_stage == READY_TO_INIT && data.Scale_Inited) {
    slam_stage = INITING;
    Twv_Inited = false;
    Pwv0.setZero();
    Qwv0 = tf::Quaternion::getIdentity();
  printf("\n\n------VINS system scaled-----\n\n");
  }

        //fixed value (please move to param ) (assume i frame and c frame are same place)
        tf::Quaternion Qic (0,0,0,1);
        tf::Quaternion Pic (0,0,0,0);


  //Prepare offset
  if (!Twv_Inited && slam_stage == INITING)
  {
    //We using imu quaternion buffer for delay compensate to achieve more accurate than current. then we should check imu are filled and not outdate.
    if(QwiIsReady && (PwiIsReady || ORIGIN==INDOOR) )
    {
        Twv_Inited = true;
    slam_stage = READY_TO_USE;
    //INDOOR case we stuck when no Pwi from state due to not init filter so let Pwi zero
    if(ORIGIN==INDOOR) Pwi_late.setZero();
        //This below compute Twv that offset from world frame by Pic Qic relation.
        //vision respected by camera is , (The translation Pcv is a rotated negative of Pvc:) Pcv = -Rvc^T*Pvc
        tf::Quaternion pose_vc_minus(-data.pose.pose.position.x, 
                                     -data.pose.pose.position.y, 
                                     -data.pose.pose.position.z,
                                     0);
        tf::Quaternion Pcv_ = (q_vc_rebuild.inverse()*pose_vc_minus*q_vc_rebuild);
        tf::Quaternion Piv_ = Pic + Qic*Pcv_*Qic.inverse() ;
        tf::Quaternion Pwi_late_ (Pwi_late.x(),Pwi_late.y(),Pwi_late.z(),0);
        //check this eq Pwi should not be here 
        tf::Quaternion Pwv0_ = Pwi_late_ + Qwi_late*(Piv_)*Qwi_late.inverse();
        Pwv0 = Pwv0_.getAxis();
        Qwv0 = Qwi_late * Qic * q_vc_rebuild.inverse();
       
        sayend("\n\n----------INIT OFFSET Twv----------\n\n");
    }else{
      ROS_INFO_THROTTLE(2,"Waiting for IMU and MSF for init Twv");
    }
  }



  //ORIENTATION ROTATE ---------------------
  //note cam_to_vision give us rpy that cam rotated
  

  /* Use method ref http://ros-users.122217.n3.nabble.com/tf-and-quaternions-td2206128.html
  tf::Quaternion gyroscope_offset(tf::Vector3(0,0,1), M_PI/4.0);  // Axis/angle 
  tf::Quaternion robot_rotation = stransform * gyroscope_offset.inverse(); 
  */

  // q_wv = q_wi qic qvc* this is offset if qic = identity (1,0,0,0) then qwv=qwi*qvc
  // tf::Quaternion q_wv;
  // q_wv.setRPY(roll_offset, pitch_offset+CAMERA_PITCH,yaw_offset);
  tf::Quaternion q_wc = Qwv0 * q_vc_rebuild;
  q_wc.normalize();
  //just for get raw yaw that cam is -- to calibrate yaw offset online -- not the same place at 
  tf::Matrix3x3 M_wc(q_wc);
  double r_,p_,y_;
  M_wc.getRPY(r_,p_,y_);

  /*if(Twv_Inited) {
    saytab(ROLL_IMU);saytab(PITCH_IMU);saytab(YAW_IMU);
    saytab(r_);saytab(p_);sayend(y_);
  }*/
  //Public to global -- > for switching conditioning
  YAW_SLAM_CALED = y_;


  //VECTOR ROTATE ------------------------
  // tf::Quaternion pose_vc(data.pose.pose.position.z, -data.pose.pose.position.x, -data.pose.pose.position.y,0);

  // pose_vc = q_wv*pose_vc*q_wv.inverse();

  //cut term Pic because we send cam term (post compensate with)
  tf::Vector3 Pwc = Pwv0 + tf::Quaternion(Qwv0*pose_vc*Qwv0.inverse() /*- Qwi_late*Pic*Qwi_late.inverse()*/).getAxis();


  //Send out slam z position to pre-scale (outdated)
  slam_before_scale = Pwc; //save for scale


  if (slam_stage == READY_TO_USE) 
  {

    vision_scaled = (Pwc)/scale_slam;


    odom_pose_filtered.header.stamp = data.header.stamp;

    odom_pose_filtered.pose.pose.position.x = vision_scaled.x() + 2*vision_offset.x();
    odom_pose_filtered.pose.pose.position.y = vision_scaled.y() + 2*vision_offset.y();
    odom_pose_filtered.pose.pose.position.z = vision_scaled.z() + 2*vision_offset.z();

    odom_pose_filtered.pose.pose.orientation.w = q_wc.w();
    odom_pose_filtered.pose.pose.orientation.x = q_wc.x();
    odom_pose_filtered.pose.pose.orientation.y = q_wc.y();
    odom_pose_filtered.pose.pose.orientation.z = q_wc.z();

	/*printf("%.2f %.2f %.2f\n", Pwv0.x()
						   , Pwv0.y()
						   , Pwv0.z());*/

    static tf::TransformBroadcaster br;
    tf::Transform send_tf;
    send_tf.setOrigin( Pwv0 + 2*vision_offset + tf::Vector3(state_out.data[pwvx],state_out.data[pwvy],state_out.data[pwvz]));
    send_tf.setRotation(Qwv0);
    br.sendTransform(tf::StampedTransform(send_tf, ros::Time::now(), "world", "vision"));

    tf::Transform Tvc_scaled;
    Tvc_scaled.setOrigin( pose_vc.getAxis()/state_out.data[L]);
    Tvc_scaled.setRotation(q_vc_rebuild);
    br.sendTransform(tf::StampedTransform(Tvc_scaled, ros::Time::now(), "vision", "cam_caled"));

  }



}
tf::Vector3 PwiBuff(const tf::Vector3& Pwi_in) 
{
  static uint16_t index = 0;
  static bool index_full=  false;
  const int size_of_buffer = (delay_slam*68);
  static tf::Vector3 Pwi[size_of_buffer];
  tf::Vector3 Pwi_late;
  static ros::Time last_stamp = state_out.header.stamp;

  if(last_stamp-state_out.header.stamp > ros::Duration(0.1) ) 
  {
    //clear buffer
    for(int i=0;i<20;i++) Pwi[i] = tf::Vector3(0,0,0);
    index = 0;
    index_full = false;
    ROS_WARN_THROTTLE(2,"Haven't received any STATE data or timeout or discontinue > 0.1 s");
    PwiIsReady=false;
    return tf::Vector3(0,0,0);
  }
  //assume quaternion data is continue
  //push
  Pwi[index]=Pwi_in;

  //this is end of buffer?
  if(index==size_of_buffer-1)
    index_full=true;

  //check if full then pop
  if(index_full)
    Pwi_late=Pwi[(index+1)%size_of_buffer];

  //update index
  index++;
  index=(index)%size_of_buffer;

  PwiIsReady=index_full;
  return Pwi_late;
}

tf::Quaternion QwiBuff(const tf::Quaternion& Qwi_in) 
{
  static uint16_t index = 0;
  static bool index_full=  false;
  const int size_of_buffer = (delay_slam*162);
  static tf::Quaternion Qwi[size_of_buffer];
  tf::Quaternion Qwi_late;
  static ros::Time last_stamp = imuMsg.header.stamp;
  if(last_stamp-imuMsg.header.stamp >ros::Duration(0.1)) 
  {
    //clear buffer
    for(int i=0;i<20;i++) Qwi[i] = tf::Quaternion::getIdentity();
    index = 0;
    index_full = false;
    ROS_WARN_THROTTLE(2,"Haven't received any IMU data or timeout or discontinue > 0.1 s");
    return tf::Quaternion::getIdentity();
  }
  //assume quaternion data is continue
  //push
  Qwi[index]=Qwi_in;

  //this is end of buffer?
  if(index==size_of_buffer-1)
    index_full=true;

  //check if full then pop
  if(index_full)
    Qwi_late=Qwi[(index+1)%size_of_buffer];

  //update index
  index++;
  index=(index)%size_of_buffer;

  QwiIsReady=index_full;
  return Qwi_late;
}


inline void Init_filter() {
      dynamic_reconfigure::ReconfigureRequest srv_req;
      dynamic_reconfigure::ReconfigureResponse srv_resp;
      dynamic_reconfigure::BoolParameter bool_param;
      dynamic_reconfigure::Config conf;

      bool_param.name = "core_init_filter";
      bool_param.value = true;
      conf.bools.push_back(bool_param);

      srv_req.config = conf;

      ros::service::call("/MSF/position_pose_pressure_sensor/set_parameters", srv_req, srv_resp);
}
inline void RovioRESET(const tf::Vector3& p, const tf::Quaternion& q)
{

      rovio::SrvResetToPose srv;
      geometry_msgs::Pose pose;
      pose.position.x = p.x();
      pose.position.y = p.y();
      pose.position.z = p.z();
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      srv.request.T_WM = pose;

      if (clientrovio.call(srv))
      {
        ROS_INFO("reset ROVIO");
      }

}


void imu_callback(const sensor_msgs::Imu::ConstPtr& data) {
  imuMsg = *data;

  Qwi_=tf::Quaternion(imuMsg.orientation.x,
                      imuMsg.orientation.y,
                      imuMsg.orientation.z,
                      imuMsg.orientation.w);
  tf::Matrix3x3 m(Qwi_);
  m.getRPY(ROLL_IMU, PITCH_IMU, YAW_IMU);
  //stack for vision alignment
  Qwi_late = QwiBuff(Qwi_);
  Qwi = Qwi_;
}
void baro_alt_callback(const sensor_msgs::FluidPressure::ConstPtr& data) {
  // 10hz
  static bool setZRef = true;
  static bool old_state_arm = quad_stateMsg.armed;
  if(old_state_arm ==0 && quad_stateMsg.armed) 
    setZRef=true;
  old_state_arm = quad_stateMsg.armed;


  static uint8_t count = 0;
  static float baro_buff[20];
  if(setZRef) {
    if(count==20) {
      count = 21;
      for(int i=0;i<20;i++) 
        baroaltABSMsg_init+=baro_buff[i];
      baroaltABSMsg_init/=20;
      printf("============Init baro at %.2f============\n", baroaltABSMsg_init);
      setZRef = false;
      count = 0;
      FILTER_INIT_REQUEST = true;
    } else if(count<20) {
      baro_buff[count] = data->fluid_pressure;
      count++;
      return;
    }
  } else {
    baroaltABSMsg.header = data->header;
    baroaltABSMsg.point.z = data->fluid_pressure - baroaltABSMsg_init;

    //experiment using complementary z to preinit scale 
    static uint16_t index = 0;
    static bool index_full = false;
    const int size_of_buffer = 20; //delay * navmsg_hz- > 18hz
    static float z_buffer[size_of_buffer];
    //push
    z_buffer[index]=baroaltABSMsg.point.z;

    //check if full then pop
    if(index_full)
    {
          for(uint8_t j=0;j<size_of_buffer;j++)
            baro_maf+=z_buffer[j];
          baro_maf/=size_of_buffer;
    }
    else if(index==size_of_buffer-1)
      index_full=true;
    else 
      baro_maf=0;
    //update index
    index++;
    index=(index)%size_of_buffer;

  }

  // //get data z to use scale slam
  // baro_inertial_z_late = get_pose_nav_z_buffer(baroaltABSMsg.point.z);

}
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& data) {
  gpsrawMsg = *data;
  static bool IsHomeSet = false;
  static uint8_t old_state_arm = 33;

  // if(old_state_arm != quad_stateMsg.armed && GPS_IS_READY)
  //   IsHomeSet=false;

  // if(!GPS_STATE) IsHomeSet = true;
  //this part shouldn't happen when GPS bad
  old_state_arm = quad_stateMsg.armed;
  if(!IsHomeSet) {
    IsHomeSet = true;
    DoSetHome();
  }

  GPS.lat = gpsrawMsg.latitude;
  GPS.lng = gpsrawMsg.longitude;
  GPS.alt = gpsrawMsg.altitude;
  double x;
  double y;
  // saytab(gpsrawMsg.latitude);sayend(gpsrawMsg.longitude);
  map_projector(GPS,
    GPS_HOME,
    &x,
    &y);
  poseMsg.header = gpsrawMsg.header;
  poseMsg.pose.pose.position.x = x;
  poseMsg.pose.pose.position.y = y;
  poseMsg.pose.pose.position.z = GPS.alt - GPS_HOME.alt;
  poseMsg.pose.pose.orientation = imuMsg.orientation;
  poseMsg.pose.covariance[0]=poseMsg.pose.covariance[7]=GPS_hAcc*invSqrt(5); //discretized by 5 hz (m/sqr(hz))
  poseMsg.pose.covariance[14]=9999;
  poseMsg.pose.covariance[21]=GPS_velAcc;
  poseMsg.pose.covariance[28]=GPS_velAcc;
  poseMsg.pose.covariance[35]=GPS_velAcc;

}
void gps_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& data) {
  navvelMsg = *data;

  navvelOdomMsg.header.stamp = data->header.stamp;
}
void quad_state_callback(const mavros_msgs::State::ConstPtr& data) {
  quad_stateMsg = *data;
  // printf("ARMED = %d", quad_stateMsg.armed);
}
void lpe_callback(const nav_msgs::Odometry::ConstPtr& data) {
  lpeMsg = *data;

  // baroaltABSMsg.header = data->header;
  // baroaltABSMsg.point.z = data->pose.pose.position.z;

  // //get data z to use scale slam  # move to using state z for init slam scale
  // baro_inertial_z_late = get_pose_nav_z_buffer(baroaltABSMsg.point.z);
}
void rcin_callback(const mavros_msgs::RCIn::ConstPtr& data) 
{
  RCInMsg = *data;
}
void rovio_callback(const nav_msgs::Odometry::ConstPtr& data)
{
  rovioMsg = *data;

  static float yaw_offset;
  static float roll_offset;
  static float pitch_offset;

  static int sum_good = 0;
  float cov_tran, cov_orient;

  bool is_laging = (ros::Time::now().toSec()-rovioMsg.header.stamp.toSec() > 0.5 ? true:false);
  bool is_high_cov = (rovioMsg.pose.covariance[0]>2 ? true:false);
  //watchdog
  if(is_laging || is_high_cov)
  {
    tf::Vector3 pos_init;
    tf::Quaternion q_init = Qwi_;

    if(msf_is_running)
    {
      pos_init = tf::Vector3(state_out.data[px],
                             state_out.data[py],
                             state_out.data[pz]);
    }else
      pos_init = tf::Vector3(0,0,0);

    RovioRESET(pos_init,q_init);
    sum_good = 0;
  }
    

  if (!is_high_cov) {
    ++sum_good;
    sum_good = constrain(sum_good, 0 , 40);
  //   if (sum_good < 40)
  //     sum_good++;
  //   if (slam_stage == READY_TO_USE) {
  //     cov_tran = Max(cov_tran*0.95,VIS_POSITION_NOISE);
  //   } else {
  //     cov_tran = Min(cov_tran*1.15,10);
  //   }
  // } else {
  //   if(sum_good<=0) sum_good=0;
  //   else          --sum_good;
  //   cov_tran = Min(cov_tran*1.15,10);
  }else{
    --sum_good;
    sum_good = constrain(sum_good,0,40);
  }

  // if(cov_tran>10) cov_tran = 10;
  // if(cov_tran<VIS_POSITION_NOISE) cov_tran =VIS_POSITION_NOISE;

  // //try 
  // if(VISION_STATE) {
  //   cov_tran=VIS_POSITION_NOISE;
  // }else{
  //   cov_tran=10;
  // }

  // if(VISION_STATE && GPS_STATE) {
  //   cov_tran=VIS_POSITION_NOISE*2*GPS_HACC_CUTOFF/Min(GPS_HACC_CUTOFF,GPS_hAcc);
  // }

  // //TODO  - depend it with number of tracked feature
  // cov_orient = cov_tran*0.5;
  //Updata covariance
  odom_pose_filtered.pose.covariance = rovioMsg.pose.covariance;
  // odom_pose_filtered.pose.covariance[7] = cov_tran;
  // odom_pose_filtered.pose.covariance[14] = cov_tran;
  // odom_pose_filtered.pose.covariance[21] = cov_orient;
  // odom_pose_filtered.pose.covariance[28] = cov_orient;
  // odom_pose_filtered.pose.covariance[35] = cov_orient;

  //--------------------------------------------------------


  tf::Quaternion pose_vc(rovioMsg.pose.pose.position.x, 
                           rovioMsg.pose.pose.position.y, 
                           rovioMsg.pose.pose.position.z,
                           0);
  tf::Quaternion q_vc_rebuild(rovioMsg.pose.pose.orientation.x, 
                              rovioMsg.pose.pose.orientation.y, 
                              rovioMsg.pose.pose.orientation.z, 
                              rovioMsg.pose.pose.orientation.w);
  q_vc_rebuild.normalize();

  // printf("sumgood=%d \t stage = %d \n", sum_good,slam_stage);
  if(slam_stage!=40)
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
  



// NOTE : 
// STATE MACHINE
//  NOT_OK = 0,  -> lost and waiting to reset
//  READY_TO_INIT , --> tracked but wait for scaling 
//  INITING --> prepare for 
//  READY_TO_USE -->scalling 
// };
  static tf::Vector3 Pwv0(0,0,0);
  static tf::Quaternion Qwv0;


  //working with VIN init system-----------------------------
  if(slam_stage == READY_TO_INIT/* && data.Scale_Inited*/) {
    slam_stage = INITING;
    Twv_Inited = false;
    Pwv0.setZero();
    Qwv0 = tf::Quaternion::getIdentity();
  printf("\n\n------VINS system scaled-----\n\n");
  }

        //fixed value (please move to param ) (assume i frame and c frame are same place)
        tf::Quaternion Qic (0,0,0,1);
        tf::Quaternion Pic (0,0,0,0);


  //Prepare offset
  if (!Twv_Inited && slam_stage == INITING)
  {
    //We using imu quaternion buffer for delay compensate to achieve more accurate than current. then we should check imu are filled and not outdate.
    if(QwiIsReady && (PwiIsReady || ORIGIN==INDOOR) )
    {
        Twv_Inited = true;
    slam_stage = READY_TO_USE;
    //INDOOR case we stuck when no Pwi from state due to not init filter so let Pwi zero
    if(ORIGIN==INDOOR) Pwi_late.setZero();
        //This below compute Twv that offset from world frame by Pic Qic relation.
        //vision respected by camera is , (The translation Pcv is a rotated negative of Pvc:) Pcv = -Rvc^T*Pvc
        tf::Quaternion pose_vc_minus(-rovioMsg.pose.pose.position.x, 
                                     -rovioMsg.pose.pose.position.y, 
                                     -rovioMsg.pose.pose.position.z,
                                     0);
        tf::Quaternion Pcv_ = (q_vc_rebuild.inverse()*pose_vc_minus*q_vc_rebuild);
        tf::Quaternion Piv_ = Pic + Qic*Pcv_*Qic.inverse() ;
        tf::Quaternion Pwi_late_ (Pwi_late.x(),Pwi_late.y(),Pwi_late.z(),0);
        //check this eq Pwi should not be here 
        tf::Quaternion Pwv0_ = Pwi_late_ + Qwi_late*(Piv_)*Qwi_late.inverse();
        Pwv0 = Pwv0_.getAxis();
        Qwv0 = Qwi_late * Qic * q_vc_rebuild.inverse();
       
        sayend("\n\n----------INIT OFFSET Twv----------\n\n");
    }else{
      ROS_INFO_THROTTLE(2,"Waiting for IMU and MSF for init Twv");
    }
  }



  //ORIENTATION ROTATE ---------------------
  //note cam_to_vision give us rpy that cam rotated
  

  /* Use method ref http://ros-users.122217.n3.nabble.com/tf-and-quaternions-td2206128.html
  tf::Quaternion gyroscope_offset(tf::Vector3(0,0,1), M_PI/4.0);  // Axis/angle 
  tf::Quaternion robot_rotation = stransform * gyroscope_offset.inverse(); 
  */

  // q_wv = q_wi qic qvc* this is offset if qic = identity (1,0,0,0) then qwv=qwi*qvc
  // tf::Quaternion q_wv;
  // q_wv.setRPY(roll_offset, pitch_offset+CAMERA_PITCH,yaw_offset);
  tf::Quaternion q_wc = Qwv0 * q_vc_rebuild;
  q_wc.normalize();
  //just for get raw yaw that cam is -- to calibrate yaw offset online -- not the same place at 
  tf::Matrix3x3 M_wc(q_wc);
  double r_,p_,y_;
  M_wc.getRPY(r_,p_,y_);

  /*if(Twv_Inited) {
    saytab(ROLL_IMU);saytab(PITCH_IMU);saytab(YAW_IMU);
    saytab(r_);saytab(p_);sayend(y_);
  }*/
  //Public to global -- > for switching conditioning
  YAW_SLAM_CALED = y_;


  //VECTOR ROTATE ------------------------
  // tf::Quaternion pose_vc(rovioMsg.pose.pose.position.z, -rovioMsg.pose.pose.position.x, -rovioMsg.pose.pose.position.y,0);

  // pose_vc = q_wv*pose_vc*q_wv.inverse();

  //cut term Pic because we send cam term (post compensate with)
  tf::Vector3 Pwc = Pwv0 + tf::Quaternion(Qwv0*pose_vc*Qwv0.inverse() /*- Qwi_late*Pic*Qwi_late.inverse()*/).getAxis();


  //Send out slam z position to pre-scale (outdated)
  slam_before_scale = Pwc; //save for scale


  if (slam_stage == READY_TO_USE) 
  {

    vision_scaled = (Pwc)/scale_slam;


    odom_pose_filtered.header.stamp = rovioMsg.header.stamp;

    odom_pose_filtered.pose.pose.position.x = vision_scaled.x() + 2*vision_offset.x();
    odom_pose_filtered.pose.pose.position.y = vision_scaled.y() + 2*vision_offset.y();
    odom_pose_filtered.pose.pose.position.z = vision_scaled.z() + 2*vision_offset.z();

    odom_pose_filtered.pose.pose.orientation.w = q_wc.w();
    odom_pose_filtered.pose.pose.orientation.x = q_wc.x();
    odom_pose_filtered.pose.pose.orientation.y = q_wc.y();
    odom_pose_filtered.pose.pose.orientation.z = q_wc.z();

  /*printf("%.2f %.2f %.2f\n", Pwv0.x()
               , Pwv0.y()
               , Pwv0.z());*/

    static tf::TransformBroadcaster br;
    tf::Transform send_tf;
    send_tf.setOrigin( Pwv0 + 2*vision_offset + tf::Vector3(state_out.data[pwvx],state_out.data[pwvy],state_out.data[pwvz]));
    send_tf.setRotation(Qwv0);
    br.sendTransform(tf::StampedTransform(send_tf, ros::Time::now(), "world", "vision"));

    tf::Transform Tvc_scaled;
    Tvc_scaled.setOrigin( pose_vc.getAxis()/state_out.data[L]);
    Tvc_scaled.setRotation(q_vc_rebuild);
    br.sendTransform(tf::StampedTransform(Tvc_scaled, ros::Time::now(), "vision", "cam_caled"));

  }


}



void v_avoid_callback(const geometry_msgs::TwistStamped::ConstPtr& data)
{
  v_avoid_msgs = *data;
}




void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data) {
  state_out = *data;
  //ROS_INFO_STREAM("state\t"  << state_out.data[4] << "\t"  << state_out.data[5] << "\n" );
  float x_ef = state_out.data[px];
  float y_ef = state_out.data[py];
  float z_ef = state_out.data[pz];
  float vx_ef = state_out.data[vx];
  float vy_ef = state_out.data[vy];
  float vz_ef = state_out.data[vz];
  float tz_ef = state_out.data[tz];

  
  // send_msf_data(fd, x_ef, y_ef, z_ef, vx_ef, vy_ef, vz_ef);

  //get data z to use scale slam
  baro_inertial_z_late = get_pose_nav_z_buffer(z_ef);
  // baro_inertial_z_late = get_pose_nav_z_buffer(z_ef-tz_ef);
  Pwi_late = PwiBuff(tf::Vector3(x_ef,y_ef,z_ef));

}




void DoSetHome() {
  printf("============GPS RESET HOME============\n");
  //HOME reset by start or re-armed

  Location GPS_HOME_BY_VISION;
  Location ZERO;
  ZERO.lat = 0;
  ZERO.lng = 0;
  if(msf_is_running && GPS_STATE) {
      //if msf is running and GPS offset process done, move home to zero of estimator reference
      //Real form is gps(meters)+gps_Offset  - state(meters)  but need to convert both state and GPS offset to GEO unit
      float ln_in_m = state_out.data[px]-StatusMsg.GPS_OFFSET.x;
      float lat_in_m = state_out.data[py]-StatusMsg.GPS_OFFSET.y;
      map_reveseprojector(&GPS_HOME_BY_VISION,ZERO,
                          ln_in_m,
                          lat_in_m);
  }else{
    GPS_HOME_BY_VISION.lat = 0;
    GPS_HOME_BY_VISION.lng = 0;
  }
    GPS_HOME.lat = gpsrawMsg.latitude - GPS_HOME_BY_VISION.lat;
    GPS_HOME.lng = gpsrawMsg.longitude - GPS_HOME_BY_VISION.lng;
    GPS_HOME.alt = gpsrawMsg.altitude;
}
void map_projector(Location _GPS, Location _GPS_HOME, double *x, double *y) {
  // use within 1km fine
   *y = ((double)(_GPS.lat - _GPS_HOME.lat) * LATLON_TO_CM)*0.01*10e6/**0.3+0.7*hy*/;        //deg to m
   *x = ((double)(_GPS.lng - _GPS_HOME.lng) * LATLON_TO_CM)*0.01*10e6/**0.3+0.7*hx*/;  //deg to m
}
void map_reveseprojector(Location *_GPS, Location _GPS_HOME, double x, double y) {
  // use within 1km fine 7.006603, 100.502211
  _GPS->lat = _GPS_HOME.lat /*7.006603*/ + (y+HOME_OFF_y)/(LATLON_TO_CM * 0.01 * 10e6);
  _GPS->lng = _GPS_HOME.lng /*100.502211*/ + (x+HOME_OFF_x)/(LATLON_TO_CM * 0.01 * 10e6);
}







inline float handle_offset(tf::Vector3& offset, const tf::Vector3& state_,const tf::Vector3& source_,bool flag) {
  tf::Vector3 source = source_;
  tf::Vector3 state = state_;

  source = source + offset;
  tf::Vector3 different = state-source;
  tf::Vector3 different2= different-offset;
  float distance = 1/invSqrt(different2.x()*different2.x()+different2.y()*different2.y());

  if(distance > HANDLE_OFF_RADIUS && flag) {
    offset = offset*(1-HANDLE_OFF_SPD) + HANDLE_OFF_SPD*(different);
    ROS_INFO_THROTTLE(0.2,"%f\t%f\t%f\n",different2.x(),different2.y(),distance);  //print every 200ms =5hz
    // gps_inertial_offset=offset;
  }
  return distance;
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
  else {
    VISION_IS_READY=false;
    VISION_STATE=false;
  }
}


inline void check_gps() {

  // ROS_INFO_THROTTLE(0.5,"GPShacc = %f\n",GPS_hAcc);

  // if(GPS_hAcc < GPS_HACC_CUTOFF) 
  if(gps_hacc_analysis()) 
    GPS_IS_READY=true;
  else if(!VISION_STATE && GPS_hAcc < GPS_HACC_CUTOFF)  // If there is no VISION then use old method check for prevent divergence
    GPS_IS_READY=true;
  else
    GPS_IS_READY=false;
}


inline void check_state() {
  check_vision();
  check_gps();


  bool temp = MSF_IS_RUNNING();
  if(temp!=msf_is_running) { msf_is_running=temp; ROS_WARN("MSF_IS_NOW : %d\n", temp);}

  static tf::Vector3 state_msf(0,0,0);
  if(msf_is_running) {
    //if msf running get state value
    state_msf[0]=state_out.data[px];
    state_msf[1]=state_out.data[py];
    state_msf[2]=state_out.data[pz];
  }
  //SELEECTOR------------------------------------------
  static bool v_flag=true;
  static bool gps_flag =true;


  if(VISION_IS_READY) {

    //WHEN VISION SCALED
    if(msf_is_running && !VISION_STATE) { //if state not init yet handle before use it

      ROS_INFO_THROTTLE(2,"Slam initializing...");
      //UPDATE OFFSET BEFOR USE IT----------------------
      //GET VALUE
      tf::Vector3 vision = vision_scaled;
      
      //CHECK ORIGIN FUNCTION will give us Privilege_INDOOR right to init vision without waiting offset
      if(ORIGIN==INDOOR && Privilege_INDOOR && fabs(wrap_pi(YAW_SLAM_CALED-YAW_IMU)) < VIS_SWITCH_ANGLE) {
        Privilege_INDOOR = false;
        VISION_STATE=true;
      }
      if(handle_offset(vision_offset, state_msf,  vision, v_flag) <= VIS_SWITCH_RADIUS && 
           fabs(wrap_pi(YAW_SLAM_CALED-YAW_IMU)) < VIS_SWITCH_ANGLE) { //RADIUS and ANGLE must be aligned 
            //ready to use state
            VISION_STATE=true;
      }
      //reset some flag and swag
      if(VISION_STATE) {
            //reset flag
            v_flag = false;
            ROS_INFO("VISION OFFSET IS OK\n\n\n");
            //ready to publish new vis offset
            status_updated = true; 
      }

    }/*else{
      //WHEN VISION OK BUT MSF NOT RUNNING , PREPARE TO USE
    // VISION_STATE=true;
    ROS_INFO_THROTTLE(2,"Slam ready...waiting for MSF");
    }*/
    if(!msf_is_running) {
      ROS_INFO_THROTTLE(2,"Slam ready...waiting for MSF");
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
      tf::Vector3 gps(poseMsg.pose.pose.position.x,poseMsg.pose.pose.position.y,state_msf[2]);


      if(handle_offset(gps_offset, state_msf, gps, gps_flag) <= GPS_SWITCH_RADIUS) {
        //if gps is not init -> add gps_offset to force it know where's home (vision home)
        // if(GPS_INERTIAL_INITED == false) { 
        //   gps_offset = gps_inertial_offset;
        //   gps_inertial_offset.setZero();
        //   GPS_INERTIAL_INITED = true;
        // }
        
        //reset flag
        gps_flag=false;
        //ready to use state
        GPS_STATE = true;
        ROS_INFO("GPS OFFSET IS OK\n\n\n");
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

  Distort_Service();
  /////////////////////////////////////////////////////////////////////
}


inline void Distort_Service() 
{
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
}



inline bool gps_hacc_analysis() {
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
  float hacc_constrained = Min(Max(cur_hacc,min_hacc),max_hacc);
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
    score=Min(Max(score,LOWER_LIMIT),UPPER_LIMIT);

  // ROS_WARN_THROTTLE(0.25,"grad=%f  GPS score = %f\tstatus  = %s",grad_hacc,score,(score>0 ? "BAD" : "GOOD"));
  return (score>0 ? 0:1);
}


/* Drive path equation over 10hz */
void DRIVE_PATH(bool READY ) {
  if(!READY) return;
  static unsigned long start_time;
  static bool flag_init = false;

  static float des_hx_init = lpeMsg.pose.pose.position.x;
  static float des_hy_init = lpeMsg.pose.pose.position.y;
  static float des_hz_init = lpeMsg.pose.pose.position.z;
  static tf::Quaternion Q_init(imuMsg.orientation.x,
                     imuMsg.orientation.y,
                     imuMsg.orientation.z,
                     imuMsg.orientation.w);
  if(RESET_POSE_INIT_REQUEST)
  {
    RESET_POSE_INIT_REQUEST=false;
    flag_init=false;
    des_hx_init = lpeMsg.pose.pose.position.x;
    des_hy_init = lpeMsg.pose.pose.position.y;
    des_hz_init = lpeMsg.pose.pose.position.z;
  Q_init = tf::Quaternion(imuMsg.orientation.x,
                     imuMsg.orientation.y,
                     imuMsg.orientation.z,
                     imuMsg.orientation.w);
    ROS_INFO_STREAM("INITED START EIGHT PATH AT "<<des_hx_init<<"\t"<<des_hy_init<<"\t"<<des_hz_init<<"\n");
  } 
  // static float M_PI = 3.14159265359;
  float des_hx,des_hy,des_hz;
  unsigned long cur_time = millis();

  if(DRIVE_EIGHT_EQUATION) {
    if(!flag_init) {
      start_time = cur_time;
      flag_init = true;  
    }
    float t = 2*M_PI*(cur_time - start_time)*0.001/15 - M_PI;  // 8 sec to complete cycle
    if(t>=M_PI) start_time=cur_time;  

    des_hx = des_hx_init + sin(t);  
    des_hy = des_hy_init + sin(t)*cos(t);  
    des_hz = des_hz_init;
    // saytab(des_hx);
    // saytab(des_hy);
    // sayend(t);
  }
  else{
    const float pwm_to_vel = (2.0)/600;  //1.0 m/s desired in each direction

    float pitch_stick = -(RCInMsg.channels[1]-1500);
    float roll_stick  = -(RCInMsg.channels[0]-1500);
    float thot_stick  =  (RCInMsg.channels[2]-1500);
    applyDeadband(pitch_stick, 50);
    applyDeadband(roll_stick, 50);
    applyDeadband(thot_stick, 70);

    //convert rcin to velocity (check direction plz)
    tf::Quaternion vd_b(pitch_stick*pwm_to_vel,
                        roll_stick*pwm_to_vel,
                        thot_stick*pwm_to_vel,
                         0);
    // printf("vd=%.2f,%.2f,%.2f\n", vd_b.x()
                                // , vd_b.y()
                                // , vd_b.z());
    //Convert BODY to ENU frame
    tf::Vector3 vd = (Qwi_ * vd_b * Qwi_.inverse()).getAxis();


    des_hx_init+=(v_avoid.x()*0.1 + vd.x()*0.1);
    des_hy_init+=(v_avoid.y()*0.1 + vd.y()*0.1);
    des_hz_init+=vd.z()*0.1;

    des_hx = des_hx_init;
    des_hy = des_hy_init;
    des_hz = des_hz_init;
  }


  //prepare to out
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = des_hx;
  pose.pose.position.y = des_hy;
  pose.pose.position.z = des_hz;
  pose.pose.orientation.x = Q_init.x();
  pose.pose.orientation.y = Q_init.y();
  pose.pose.orientation.z = Q_init.z();
  pose.pose.orientation.w = Q_init.w();


  local_pos_pub.publish(pose);

}








inline void FIND_OBSTACLE() 
{

      tf::Quaternion vel_b_q(v_avoid_msgs.twist.linear.x,
                             v_avoid_msgs.twist.linear.y,
                             v_avoid_msgs.twist.linear.z,
                             0);
      v_avoid = (Qwi_*vel_b_q*Qwi_.inverse()).getAxis();

      printf("avoid = %.2f\t%.2f\n", vel_b_q.x(),vel_b_q.y()/**(((float)num_grid)/400)*/);
}
