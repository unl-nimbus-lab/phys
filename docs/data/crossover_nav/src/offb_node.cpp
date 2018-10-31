/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <crossover_nav/odom_data.h>



#define DRIVE_EIGHT_EQUATION 1

/* Auxilary function */
#define sayend(x) (std::cout << x << std::endl)
#define saytab(x) (std::cout << x << "\t")
#define saycomma(x) (std::cout << x << " ,")
inline long millis() {
  return (1e3 * ros::Time::now().sec + 1e-6 * ros::Time::now().nsec);
}
/*--------------------*/

mavros_msgs::State current_state;
nav_msgs::Odometry local_pose;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}
void local_pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
  local_pose = *msg;
}


/* ros publicher */
ros::Publisher local_pos_pub;
ros::Publisher vis_status_pub;
ros::Publisher vis_pose_pub;


/* Drive path equation over 20hz */
bool RESET_POSE_INIT_REQUEST = true;
void DRIVE_PATH(bool READY ) ;
/* send msg for indicate that we will offb */
void offb_wtf();
/* test send vision valid (slam valid) via velocity topic to px4 */
void slamcb(const crossover_nav::odom_data::ConstPtr& slam);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state", 10, state_cb);
  ros::Subscriber local_pose_sub = nh.subscribe<nav_msgs::Odometry>
    ("mavros/local_position/odom",10, local_pose_cb);
  ros::Subscriber slamsub = nh.subscribe<crossover_nav::odom_data>
    ("/slam", 10, slamcb);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    ("mavros/setpoint_position/local", 10);
  vis_status_pub = nh.advertise<geometry_msgs::TwistStamped>
    ("/mavros/vision_speed/speed_twist", 10);
  // vis_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
  //   ("/mavros/vision_pose/pose", 10);

  /*ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
   ("mavros/cmd/arming");
   ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
   ("mavros/set_mode");*/

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  // geometry_msgs::PoseStamped pose;
  // pose.pose.position.x = 0;
  // pose.pose.position.y = 0;
  // pose.pose.position.z = 0;

  // //send a few setpoints before starting "DO THIS EVERY TIME BEFORE OFFBOARD"
  // for(int i = 100; ros::ok() && i > 0; --i){
  //   local_pos_pub.publish(pose);
  //   ros::spinOnce();
  //   rate.sleep();
  // }

  //Prepare set mode
  // mavros_msgs::SetMode offb_set_mode;
  // offb_set_mode.request.custom_mode = "OFFBOARD";

  //Prepare set arm
  // mavros_msgs::CommandBool arm_cmd;
  // arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();


  //State Machine
  while(ros::ok()){
    static mavros_msgs::State last_state = current_state;

    if( current_state.armed != last_state.armed || 
      current_state.mode != last_state.mode) {

      ROS_INFO_STREAM("WAIT OFFB now " 
        << (current_state.armed ? "" : "DIS")
        << "ARMED\t--" 
        << current_state.mode 
        );
      last_state= current_state;
    }

    if( current_state.armed && 
      current_state.mode == "OFFBOARD") {

      /* use bool for utility other check state eg.
       battery voltage , load cpu etc*/
      DRIVE_PATH(true);

    }else{
      /* we must continue to send msg although not offb mode */
      offb_wtf();
      RESET_POSE_INIT_REQUEST=true;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

/* Drive path equation over 20hz */
void DRIVE_PATH(bool READY ) {
  if(!READY) return;

  static float des_hx_init = local_pose.pose.pose.position.x;
  static float des_hy_init = local_pose.pose.pose.position.y;
  static float des_hz_init = local_pose.pose.pose.position.z;

  if(RESET_POSE_INIT_REQUEST)
  {
    RESET_POSE_INIT_REQUEST=false;
    des_hx_init = local_pose.pose.pose.position.x;
    des_hy_init = local_pose.pose.pose.position.y;
    des_hz_init = local_pose.pose.pose.position.z;
  }
  // static float M_PI = 3.14159265359;
  float des_hx,des_hy,des_hz;

  if(DRIVE_EIGHT_EQUATION) {
    static unsigned long start_time;
    static bool flag_init = false;
    if(!flag_init) {
      start_time = millis();
      flag_init = true;
    }
    float t = 2*M_PI*(millis() - start_time)*0.001/15 - M_PI;  // 8 sec to complete cycle
    if(t>=M_PI) start_time=millis();

    des_hx = des_hx_init + sin(t);
    des_hy = des_hy_init + sin(t)*cos(t);
    des_hz = des_hz_init;
    saytab(des_hx);
    saytab(des_hy);
    sayend(t);
  }
  else{
    des_hx = des_hx_init;
    des_hy = des_hy_init;
    des_hz = des_hz_init;
  }


  //prepare to out
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = des_hx;
  pose.pose.position.y = des_hy;
  pose.pose.position.z = des_hz;

  local_pos_pub.publish(pose);

}


void offb_wtf() {
  float des_hx_init = local_pose.pose.pose.position.x;
  float des_hy_init = local_pose.pose.pose.position.y;
  float des_hz_init = local_pose.pose.pose.position.z;

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = des_hx_init;
  pose.pose.position.y = des_hy_init;
  pose.pose.position.z = des_hz_init;

  //send a few setpoints before starting "DO THIS EVERY TIME BEFORE OFFBOARD"
  for(int i = 4; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
  }
}

void slamcb(const crossover_nav::odom_data::ConstPtr& slam) {
  
  geometry_msgs::TwistStamped vis_vel;

  vis_vel.header.stamp = slam->header.stamp;
  if(slam->odom_state == 2)
  {  
    vis_vel.twist.linear.y = 2.5;
  }
  else{
    vis_vel.twist.linear.y = 0.5;
  }

  ROS_INFO_STREAM("SLAM SEND");
  vis_status_pub.publish(vis_vel);
  // vis_pose_pub.publish(vis_pose);
}
