#include <ros/ros.h>
#include <ros/time.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

//test-----------interactive control
//#include <interactive_markers/interactive_marker_server.h>

 //test-----tf transform------------
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

//function convert arduino to ros
//------------ROS version-------
// #include <iostream>
long millis() { return ros::Time::now().nsec/1000000; }
long micros() { return ros::Time::now().nsec/1000; }

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define delay(x) ros::Duration(x/1000.0).sleep()
#define say(x) (std::cout << x)
#define sayend(x) (std::cout << x << std::endl)
#define saytab(x) (std::cout << x << "\t")
#define saycomma(x) (std::cout << x << " ,")
//----------------------------------
#include "useful_fn.h"
#include "config.h"
#include "MAF.h"
#include "mpu6050.h"
#include "compass.h"
#include "ahrs.h"
#include "alt.h"
#include "controlsystem.h"
#include "motor.h"

//1 ahrs  ok
//2 alt   ok
//3 remote cmd
//4 control



sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField compass_msg;
sensor_msgs::Temperature temp_msg;
sensor_msgs::FluidPressure baro_msg;
sensor_msgs::Joy rc_msg;
sensor_msgs::JointState motorout;
geometry_msgs::PoseWithCovarianceStamped pose_gps,pose_nav;
geometry_msgs::PoseStamped RC;



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

ff


double roll, pitch, yaw;



void imucallback(sensor_msgs::Imu data)
{
  imu_msg=data;

  // tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
  // tf::Matrix3x3 m(q);
  // m.getRPY(roll, pitch, yaw);
  //ROS_INFO("ax:[%f]", yaw);
}
void compasscallback(sensor_msgs::MagneticField data)
{
  compass_msg=data;
}
void barocallback(sensor_msgs::FluidPressure data)
{
  baro_msg=data;
}
void tempcallback(sensor_msgs::Temperature data)
{
  temp_msg=data;
}
void remotecallback(sensor_msgs::Joy data)
{
  rc_msg=data;
}
void pose_gps_callback(geometry_msgs::PoseWithCovarianceStamped data)
{
  pose_gps=data;
  // ROS_INFO("pz:[%f]", data.pose.pose.position.z);
}
void pose_nav_callback(geometry_msgs::PoseWithCovarianceStamped data)
{
  pose_nav=data;
  // ROS_INFO("pose_nav_pz:[%f]", data.pose.pose.position.z);
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
  ros::init(argc, argv, "quadgg");
  ros::NodeHandle n;
  ros::Rate r(200);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu_max", 200, imucallback);
  ros::Subscriber baro_sub = n.subscribe<sensor_msgs::FluidPressure>("/imu_max/Baro", 200, barocallback);
  ros::Subscriber temp_sub = n.subscribe<sensor_msgs::Temperature>("/imu_max/Temp", 200, tempcallback);
  ros::Subscriber compass_sub = n.subscribe<sensor_msgs::MagneticField>("/imu_max/Mag", 200, compasscallback);
  ros::Subscriber rc_sub = n.subscribe<sensor_msgs::Joy>("/imu_max/Remote", 200, remotecallback);
  ros::Subscriber pos_gps_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 200, pose_gps_callback);
  ros::Subscriber pos_nav_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose_nav", 200, pose_nav_callback);

  ros::Publisher motor_pub = n.advertise<sensor_msgs::JointState>("/motor_out", 200);


  ahrs_initialize();
  delay(20);
  setupFourthOrder();

  bool init_quad_complete = false;


  int8_t baro_num=0;
  //wait for baro
          sayend("wating baro");
         do{
            ros::spinOnce();
            press = baro_msg.fluid_pressure;
            temperature = temp_msg.temperature;
            pressure = baropress.process(press);
            // saytab(press);sayend(pressure);
            // delay(200);
            r.sleep();
            if (press!=0) baro_num ++;

          }while((press==0 || getAltitude2>1000) || baro_num <80);

          alt_trim = getAltitude;
          alt_     = getAltitude;

 while (ros::ok())
  {
      //100 hz task
    // while(imu_msg.linear_acceleration.x * imu_msg.angular_velocity.x * compass_msg.magnetic_field.x==0 && !init_quad_complete && !alt_flag)
    // {
    //   say(".");
    //   ros::spinOnce();
    //   delay(1000);

    //   if(imu_msg.linear_acceleration.x * imu_msg.angular_velocity.x * compass_msg.magnetic_field.x != 0 && alt_flag) {
    //     init_quad_complete=true;
    //     flag_trim = true;
    //     sayend("ok");
    //     saytab("alt_trim = ");saytab(alt_trim);saytab("temp = ");sayend(temperature);
    //   }
    // }

      ros::spinOnce();
      r.sleep();//for 200hz stamped time

      currentTime = micros();


      if (currentTime - previousTime > 10000)
      {
        frameCounter++;

        G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
        if(G_Dt >0.02) G_Dt =0.01;  //Prevent error from time peak to 1.83e13

        hundredHZpreviousTime = currentTime;

        accel[0] = imu_msg.linear_acceleration.x/9.80655;
        accel[1] = imu_msg.linear_acceleration.y/9.80655;
        accel[2] = imu_msg.linear_acceleration.z/9.80655;
        gyro[0] = imu_msg.angular_velocity.x;
        gyro[1] = imu_msg.angular_velocity.y;
        gyro[2] = imu_msg.angular_velocity.z;
        c_magnetom_x = compass_msg.magnetic_field.x;
        c_magnetom_y = compass_msg.magnetic_field.y;
        c_magnetom_z = compass_msg.magnetic_field.z;





        Acc_f[0] = computeFourthOrder(accel[XAXIS], &fourthOrder[XAXIS]);
        Acc_f[1] = computeFourthOrder(accel[YAXIS], &fourthOrder[YAXIS]);
        Acc_f[2] = computeFourthOrder(accel[ZAXIS], &fourthOrder[ZAXIS]);

        ahrs_updateMARG(gyro[XAXIS], gyro[YAXIS], gyro[ZAXIS], Acc_f[0], Acc_f[1], Acc_f[2], c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);



        update(Acc_f[0], Acc_f[1], Acc_f[2], alt_ - alt_trim, G_Dt); //update baro acc fusion


        saytab(ahrs_r);saytab(ahrs_p);saytab(ahrs_y);saytab(estimated_altitude);sayend(estimated_velocity);

        //Test Nav-----------------

        // adaptive_filter(G_Dt) ;
//                          float attcom[2] ={hx*0.01,hy*0.01}; //convert m to cm
//                          float acccom[2] ={accel_ef.x,accel_ef.y};
//                          compute_attitude(G_Dt,acccom,attcom);
      //////////////////////////////////////////////////////////////////////////////////////////////
      ARM_DECISION();
      //////////////////////////////////////////////////////////////////////////////////////////////
      CONTROLSYSTEM();
      //////////////////////////////////////////////////////////////////////////////////////////////
      tomotor();
      motorout.velocity.clear();
      motorout.velocity.push_back(Mout1);
      motorout.velocity.push_back(Mout2);
      motorout.velocity.push_back(Mout3);
      motorout.velocity.push_back(Mout4);
      motor_pub.publish(motorout);
      //////////////////////////////////////////////////////////////////////////////////////////////





      if (frameCounter % TASK_50HZ == 0) {
         G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
         fiftyHZpreviousTime = currentTime;

         _barolast_update = millis();

         press = baro_msg.fluid_pressure;
         temperature = temp_msg.temperature;
         pressure = baropress.process(press);
         alt_ = getAltitude;
         // saytab("alt...");saytab(pressure);saytab(press);saytab(temperature);saytab(alt_);saytab(estimated_altitude);sayend(estimated_velocity);
        if (FLIGHT_MODE_CHECK == ALT_MODE || FLIGHT_MODE_CHECK == TAKE_OFF || FLIGHT_MODE_CHECK == VEL_HOLD)
        {
          // take off test
           altitude_holding_on(G_Dt);
        }/*else{ altitude_holding_off(); }*/

      }
      ////////////////////////////////////////33 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_33HZ == 0) {
          //alignment here RF_ROLL, RF_PIT, CH_THR, RF_YAW, AUX_1, AUX_2
          RF_ROLL = rc_msg.axes[0];
          RF_PIT = rc_msg.axes[1];
          CH_THR = rc_msg.axes[2];
          RF_YAW = rc_msg.axes[3];
          AUX_1 = rc_msg.axes[4];
          AUX_2 = rc_msg.axes[5];
        // rx_update();
        // Mag5883Read();


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

            // sayend("TEST10hz");

            // correction_adaptive_filter(G_Dt);
            // convert_latlon_xy();
            // debias_horizontal(G_Dt);

            //

//            saycomma(G_Dt);
//        Serial3.print(vx_theory_ef*100);Serial3.print(F(","));
//        Serial3.print(vy_theory_ef*100);Serial3.print(F(","));
//        Serial3.print(GPS_vel_x_fillered);Serial3.print(F(","));
//        Serial3.print(GPS_vel_y_fillered);Serial3.print(F(","));
            //        Serial3.print(vx_fillered);Serial3.print(F(","));
            ////                  Serial3.print(get_distance( _GPS.lat, _GPS.lng,lat_home,long_home));Serial3.print(F(","));


//            saycomma(estimated_attitude[0]);
//            saycomma(estimated_attitude[1]);
//            saycomma(vx


//            saycomma(vx_fillered);  //cm
//            saycomma(vy_fillered);
//            saycomma(GPS_vel_x_fillered);
//            saycomma(GPS_vel_y_fillered);
//
//////            saycomma(_GPS.lat);
//////            saycomma(_GPS.lng);
//            saycomma(hx*0.01);
//            saycomma(hy*0.01);
//            saycomma(Dhx_b);
//            saycomma(Dhy_b);
//            sayend(GPS_hDOP);
//        Serial3.print(_GPS.lat,7);Serial3.print(F(","));
//        Serial3.print(_GPS.lng,7);Serial3.print(F(","));
//
//        Serial3.print(_position.x);Serial3.print(F(","));
//        Serial3.print(_position.y);Serial3.print(F(","));
////        Serial3.print(_position.z);Serial3.print(F(","));
//        Serial3.print(get_latitude());Serial3.print(F(","));
//        Serial3.print(get_longitude());Serial3.print(F(","));
//
//        Serial3.print(_velocity.x);Serial3.print(F(","));
//        Serial3.print(_velocity.y);Serial3.print(F(","));
//        Serial3.print(GPS_hDOP);Serial3.println(F(""));

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
