#include <ros/ros.h>
#include <ros/time.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Range.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "crossover_nav/Ack.h"
#include "crossover_nav/Navdata.h"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

long millis() { return ros::Time::now().nsec/1000000; }
long micros() { return ros::Time::now().nsec/1000; }

//global 

    sensor_msgs::Imu imuMsg;
    sensor_msgs::MagneticField compassMsg;
    sensor_msgs::Joy remoteMsg;
    sensor_msgs::FluidPressure baroMsg;
    sensor_msgs::Temperature tempMsg;
    sensor_msgs::NavSatFix gpsrawMsg;
    sensor_msgs::Range sonarMsg;

    geometry_msgs::PoseWithCovarianceStamped poseMsg;
    geometry_msgs::TwistWithCovarianceStamped navvelMsg;
    nav_msgs::Odometry pose_navMsg, msf_data;
    nav_msgs::Odometry altMsg,desire_navMSG;
    
    crossover_nav::Ack ackMsg;
    crossover_nav::Navdata NavMsg;

    
    // imuMsg.angular_velocity_covariance = {1.2184696791468346e-7, 0 , 0, 0 , 1.2184696791468346e-7, 0, 0 , 0 , 1.2184696791468346e-7};
    // imuMsg.linear_acceleration_covariance = {8.99999999999e-8 , 0 , 0, 0 , 8.99999999999e-8, 0, 0 , 0 , 8.99999999999e-8};
    /*
    #rospy.init_node('imu_node2', anonymous=True)
    time_start=rospy.Time.now()
    imuMsg.orientation_covariance = [1 , 0 , 0, 0, 1, 0, 0, 0, 1]
    imuMsg.angular_velocity_covariance = [1.2184696791468346e-7, 0 , 0, 0 , 1.2184696791468346e-7, 0, 0 , 0 , 1.2184696791468346e-7]
    imuMsg.linear_acceleration_covariance = [8.99999999999e-8 , 0 , 0, 0 , 8.99999999999e-8, 0, 0 , 0 , 8.99999999999e-8]
    */


ros::Time  time_start;
#include "Ulink_capter.h"



void msf_callback(nav_msgs::Odometry data);

int main(int argc, char **argv)
{

  if(argv[1]!=0) port = argv[1];

  //init ros node
  ros::init(argc, argv, "quad_talker");
  ros::NodeHandle n;
  ros::Rate r(400);

  //publisher
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_max", 10);
  ros::Publisher pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 10);
  ros::Publisher navvel = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/imu_max/Navvel", 10);
  ros::Publisher posenav = n.advertise<nav_msgs::Odometry>("/imu_max/pose_nav", 10);
  ros::Publisher gpsraw = n.advertise<sensor_msgs::NavSatFix>("/imu_max/Gpsraw", 10);
  ros::Publisher sonar = n.advertise<sensor_msgs::Range>("/sonar", 10);
  ros::Publisher Nav = n.advertise<crossover_nav::Navdata>("/imu_max/Navdata", 10);
  ros::Publisher alt = n.advertise<nav_msgs::Odometry>("/imu_max/alt_odometry", 10);
  ros::Publisher des_nav = n.advertise<nav_msgs::Odometry>("/imu_max/pose_des", 10);

  //subscriber
  ros::Subscriber msf_sub = n.subscribe<nav_msgs::Odometry>("/msf_core/odometry", 10,msf_callback);

    // pose = rospy.Publisher('/imu_max/pose', PoseWithCovarianceStamped, queue_size=2)
    // navvel = rospy.Publisher('/imu_max/Navvel', TwistWithCovarianceStamped,queue_size=10)
    // posenav = rospy.Publisher('/imu_max/pose_nav', Odometry, queue_size=10)
    // gpsraw = rospy.Publisher('/imu_max/Gpsraw', NavSatFix, queue_size=2)
    // sonar  = rospy.Publisher('/sonar', Range, queue_size=10)
    // Nav    = rospy.Publisher('/imu_max/Navdata',Navdata, queue_size=10)
    // alt     = rospy.Publisher('/imu_max/alt_odometry', Odometry, queue_size=10)
    // des_nav = rospy.Publisher('/imu_max/pose_des',Odometry, queue_size=1)


  //initialize imu covariance 
  for(int i=0;i<9;i++) 
    {
      imuMsg.orientation_covariance[i]=0;
      imuMsg.angular_velocity_covariance[i]=0;
      imuMsg.linear_acceleration_covariance[i]=0;
      gpsrawMsg.position_covariance[i]=0;
      altMsg.pose.covariance[i]=0;
    }
    // memset(&altMsg.pose.covariance,0,sizeof(altMsg.pose.covariance));
    for(int i=0;i<36;i++) 
    {
      poseMsg.pose.covariance[i]=0;
      navvelMsg.twist.covariance[i]=0;
    }
  imuMsg.orientation_covariance[0]=imuMsg.orientation_covariance[4]=imuMsg.orientation_covariance[8]=1;
  imuMsg.angular_velocity_covariance[0]=imuMsg.angular_velocity_covariance[4]=imuMsg.angular_velocity_covariance[8]=1.2184696791468346e-7;
  imuMsg.linear_acceleration_covariance[0]=imuMsg.linear_acceleration_covariance[4]=imuMsg.linear_acceleration_covariance[8]=8.99999999999e-8;
            // poseMsg.pose.covariance[0]=poseMsg.pose.covariance[7]=d2[1]*0.01;
          // poseMsg.pose.covariance[21]=poseMsg.pose.covariance[28]=poseMsg.pose.covariance[35]=1;

          // navvelMsg.twist.covariance[0]=navvelMsg.twist.covariance[7]=d2[1]*0.01;
          // navvelMsg.twist.covariance[14]=0.1;


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
  ros::Time imu_ts , Nav_ts ,pose_ts,posenav_ts,gpsraw_ts,navvel_ts, sonar_ts,alt_ts,des_nav_ts,rosspin_ts;
  imu_ts = Nav_ts =pose_ts=posenav_ts=gpsraw_ts=navvel_ts= sonar_ts=alt_ts=des_nav_ts=rosspin_ts=ros::Time::now();
  
  int rosspin_count=0;
  ros::Time tomorrow = ros::Time::now() + ros::Duration(24*60*60);
  ros::Duration negative_one_day = ros::Time::now() - tomorrow;

time_start = ros::Time::now();
ros::Time cur_time = time_start;

while(ros::ok()) 
{
   serial_handle(fd_ptr);

   // r.sleep();
   cur_time = ros::Time::now();
   if(imu_ts!=imuMsg.header.stamp){
    imu_pub.publish(imuMsg);
    Nav.publish(NavMsg);
    imu_ts=imuMsg.header.stamp;
    
   }
   if(pose_ts!=poseMsg.header.stamp){
    pose.publish(poseMsg);
    pose_ts=poseMsg.header.stamp;
   }
   if(posenav_ts!=pose_navMsg.header.stamp){
    posenav.publish(pose_navMsg);
    posenav_ts=pose_navMsg.header.stamp;
   }
   if(gpsraw_ts!=gpsrawMsg.header.stamp){
    gpsraw.publish(gpsrawMsg);
    gpsraw_ts=gpsrawMsg.header.stamp;
   }
   if(navvel_ts!=navvelMsg.header.stamp){
    navvel.publish(navvelMsg);
    navvel_ts=navvelMsg.header.stamp;
   }
   if(sonar_ts!=sonarMsg.header.stamp){
    sonar.publish(sonarMsg);
    sonar_ts=sonarMsg.header.stamp;
   }
   if(alt_ts!=altMsg.header.stamp){
    alt.publish(altMsg);
    alt_ts=altMsg.header.stamp;
   }
   if(des_nav_ts!=desire_navMSG.header.stamp){
    des_nav.publish(desire_navMSG);
    des_nav_ts=desire_navMSG.header.stamp;
   }
   rosspin_count++;
   if(rosspin_count>10) {
   // if(cur_time - rosspin_ts > ros::Duration(0.02))
   //  {
      ros::spinOnce();
    //   rosspin_ts = cur_time;
    // }
    rosspin_count = 0;
   }
   // r.sleep();
    // pose = rospy.Publisher('/imu_max/pose', PoseWithCovarianceStamped, queue_size=2)
    // navvel = rospy.Publisher('/imu_max/Navvel', TwistWithCovarianceStamped,queue_size=10)
    // posenav = rospy.Publisher('/imu_max/pose_nav', Odometry, queue_size=10)
    // gpsraw = rospy.Publisher('/imu_max/Gpsraw', NavSatFix, queue_size=2)
    // sonar  = rospy.Publisher('/sonar', Range, queue_size=10)
    // Nav    = rospy.Publisher('/imu_max/Navdata',Navdata, queue_size=10)
    // alt     = rospy.Publisher('/imu_max/alt_odometry', Odometry, queue_size=10)
    // des_nav = rospy.Publisher('/imu_max/pose_des',Odometry, queue_size=1)




}
  
  // GThread* serial_thread;
  // GError* err;
  // if (!g_thread_supported())
  // {
  //   g_thread_init(NULL);
  //   // Only initialize g thread if not already done
  // }

  // Run indefinitely while the ROS and serial threads handle the data
  if (!silent)
    printf("\nREADY, waiting for serial/ROS data.\n");

  // if ((serial_thread = g_thread_create((GThreadFunc)serial_wait, (void *)fd_ptr, TRUE, &err)) == NULL)
  // {
  //   printf("Failed to create serial handling thread: %s!!\n", err->message);
  //   g_error_free(err);
  // }

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
   * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
   */
  ros::spin();

  close_port(fd);

  //g_thread_join(serial_thread);
  //exit(0);

  return 0;

}



void msf_callback(nav_msgs::Odometry data)
{
  msf_data=data;
  send_msf_data(fd,data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z);
  //ROS_INFO("%f %f %f",data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z);
}