/*!

@mainpage
  drrobot_player is a driver for motion control system on Jaguar series mobile robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@section usage Usage
@par After start roscore, you need load robot configuration file to parameter server first.

$ drrobot_player
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
- @b "motor_cmd_sub"/std_msgs::string : motor commands to drive the robot.

Publishes to (name / type):
-@b drrobot_motor/MotorDataArray: will publish MotorDataArray Message. Please referee the message file.
-@b drrobot_motorboard/MotorDriverBoardArray: will publish MotorBoardArray Message. Please referee the message file.
-@b drrobot_gps/GPSInfo: will publish GPS Message
-@b drrobot_imu/IMUData: will publish IMU Message
<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotBaseIP (string) : robot main WiFi module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number first serial port, and as default the value increased by one will be second port number.
- @b RobotSerialPort (int) : specify the serial port name if you choose serial communication in RobotCommMethod, default /dev/ttyS0"
- @b RobotType (string) : specify the robot type, now should in list: Jaguar
- @b MotorDir (int) : specify the motor control direction
- @b WheelRadius (double) : wheel radius
- @b WheelDistance (double) : the distance between two driving wheels
- @b EncoderCircleCnt (int) : one circle encoder count
- @b MinSpeed (double) : minimum speed, unit is m/s.
- @b MaxSpeed (double) : maximum speed, unit is m/s.
 */

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp> 
#include <boost/algorithm/clamp.hpp>
#include <cmath> 

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <jaguar4x4/MotorData.h>
#include <jaguar4x4/MotorDataArray.h>
#include <jaguar4x4/MotorBoardInfoArray.h>
#include <jaguar4x4/MotorBoardInfo.h>

#include <DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM 4
#define MOTOR_BOARD_NUM 3
//#define slope 500
using namespace std;
using namespace DrRobot_MotionSensorDriver;

class Jaguar_Controller_Node {
  public:
    ros::NodeHandle node_;

    ros::Publisher motorInfo_pub_;
    ros::Publisher motorBoardInfo_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher imu_pub_;
    //ros::Publisher compass_pub_;
    ros::Subscriber motor_cmd_sub_;	
    std::string robot_prefix_;
    
    Jaguar_Controller_Node() {
      ros::NodeHandle private_nh("~");

      robotID_ = "DrRobot";
      private_nh.getParam("RobotID",robotID_);
      ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

      robotType_ = "Jaguar";
      private_nh.getParam("RobotType",robotType_);
      ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

      robotCommMethod_ = "Network";
      private_nh.getParam("RobotCommMethod",robotCommMethod_);
      ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

      robotIP_ = "192.168.0.60";
      private_nh.getParam("RobotBaseIP",robotIP_);
      ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

      commPortNum_ = 10001;
      private_nh.getParam("RobotPortNum",commPortNum_);
      ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

      robotSerialPort_ = "/dev/ttyS0";
      private_nh.getParam("RobotSerialPort",robotSerialPort_);
      ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

      motorDir_ = 1;
      private_nh.getParam("MotorDir", motorDir_);
      ROS_INFO("I get MotorDir: [%d]", motorDir_);

      wheelRadius_ = 0.135;
      private_nh.getParam("WheelRadius", wheelRadius_);
      ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

      wheelDis_ = 0.52;
      private_nh.getParam("WheelDistance", wheelDis_);
      ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

      minSpeed_ = 0.1;
      private_nh.getParam("MinSpeed", minSpeed_);
      ROS_INFO("I get Min Speed: [%f]", minSpeed_);

      maxSpeed_ = 1.0;
      private_nh.getParam("MaxSpeed", maxSpeed_);
      ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

      encoderOneCircleCnt_ = 300;
      private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
      ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

      if (robotCommMethod_ == "Network") {
        robotConfig1_.commMethod = Network;
      }
      else {
        robotConfig1_.commMethod = Serial;
      }

      if (robotType_ == "Jaguar") {
        robotConfig1_.robotType = Jaguar;
      }
      robotConfig1_.portNum = commPortNum_;

      strcpy(robotConfig1_.robotIP,robotIP_.c_str());

      strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());

      //create publishers for sensor data information
      motorInfo_pub_ = node_.advertise<jaguar4x4::MotorDataArray>("drrobot_motor", 1);
      motorBoardInfo_pub_ = node_.advertise<jaguar4x4::MotorBoardInfoArray>("drrobot_motorboard", 1);
      gps_pub_ = node_.advertise<sensor_msgs::NavSatFix>("gps", 1);
      imu_pub_ = node_.advertise<sensor_msgs::Imu>("imu", 1);
      //compass_pub_ = node_.advertise<geometry_msgs::Vector3Stamped>("compass", 1);

      drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
      if (  (robotType_ == "Jaguar") ) {
        drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
      }
    }

    ~Jaguar_Controller_Node() {}

    int start() {
    	int res = -1;
    	if (  (robotType_ == "Jaguar")) {
    		res = drrobotMotionDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
    		if (res == 0) {
    			ROS_INFO("open port number at: [%d]", robotConfig1_.portNum);
    		}
    		else {
    			ROS_INFO("could not open network connection to [%s,%d]",  robotConfig1_.robotIP,robotConfig1_.portNum);
    		}
    	}

    	drrobotMotionDriver_ -> sendCommand("MMW !MG", 7);

    	average_angle_vel_x = 0.0;
    	average_angle_vel_y = 0.0;
    	average_angle_vel_z = 0.0;
    	drrobotMotionDriver_->readIMUSensorData(&imuSensorData);
    	for (int i = 0; i < 1000; i++) {
    		drrobotMotionDriver_->readIMUSensorData(&imuSensorData);
    		average_angle_vel_x += (float)imuSensorData.gyro_x;
    		average_angle_vel_y += (float)imuSensorData.gyro_y;
    		average_angle_vel_z += (float)imuSensorData.gyro_z;
    		ros::Duration(0.005).sleep();
    	}
    	average_angle_vel_x /= 1000.0;
    	average_angle_vel_y /= 1000.0;
    	average_angle_vel_z /= 1000.0;

    	ROS_INFO("E-brake released!");
    	drrobotMotionDriver_ -> sendCommand("MMW !MG", 7);
    	motor_cmd_sub_ = node_.subscribe("cmd_vel", 1, &Jaguar_Controller_Node::cmdReceived, this);

    	return(0);
    }

    int stop() {
        drrobotMotionDriver_ -> sendCommand("MMW !M 0 0", 10);
        ros::Duration(5).sleep();
        drrobotMotionDriver_ -> sendCommand("MMW !EX", 7);
        drrobotMotionDriver_ -> close();

        usleep(1000000);
        return(0);
    }

    void cmdReceived(const geometry_msgs::Twist& msg) {
      //vx = boost::algorithm::clamp(msg.linear.x, -1.0, 1.0);
      //vtheta = boost::algorithm::clamp(msg.angular.z, -1.0, 1.0);
      //left_vel = round(((vx - vtheta)+2)*slope-1000);
      //right_vel = round(((-vx - vtheta)+2)*slope-1000);
      left_vel = msg.linear.x - msg.angular.z;
      right_vel = -msg.linear.x - msg.angular.z;
      double large_speed = std::max(std::abs(left_vel), std::abs(right_vel));
      if (large_speed > maxSpeed_) {
        left_vel *= maxSpeed_ / large_speed;
        right_vel *= maxSpeed_ / large_speed;
      }
      LEFT_vel = round(left_vel*1000);
      RIGHT_vel = round(right_vel*1000);
      string p = "MMW !M ";
      p += to_string(LEFT_vel) + " " + to_string(RIGHT_vel);
      int nLen = p.length();
      drrobotMotionDriver_ -> sendCommand(p.c_str(), nLen); // Send command to robot
    }

    void doUpdate() {
      //int test = imuSensorData.seq;
      drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
      drrobotMotionDriver_->readMotorBoardData(&motorBoardData_);
      drrobotMotionDriver_->readGPSSensorData(&gpsSensorData_);

      jaguar4x4::MotorDataArray motorDataArray;
      motorDataArray.motorData.resize(MOTOR_NUM);
      for (uint32_t i = 0 ; i < MOTOR_NUM; ++i) {
        motorDataArray.motorData[i].header.stamp = ros::Time::now();
        motorDataArray.motorData[i].header.frame_id = string("drrobot_motor_");
        motorDataArray.motorData[i].header.frame_id += boost::lexical_cast<std::string>(i);

        motorDataArray.motorData[i].motorPower = motorSensorData_.motorSensorPWM[i];
        motorDataArray.motorData[i].encoderPos = motorSensorData_.motorSensorEncoderPos[i];
        motorDataArray.motorData[i].encoderVel = motorSensorData_.motorSensorEncoderVel[i];
        motorDataArray.motorData[i].encoderDiff = motorSensorData_.motorSensorEncoderPosDiff[i];
        motorDataArray.motorData[i].motorTemp = motorSensorData_.motorSensorTemperature[i];
        motorDataArray.motorData[i].motorCurrent = motorSensorData_.motorSensorCurrent[i];
      }

      //ROS_INFO("publish motor info array [%d",motorSensorData_.motorSensorEncoderPos[4]);
      motorInfo_pub_.publish(motorDataArray);

      jaguar4x4::MotorBoardInfoArray motorBoardInfoArray;
      motorBoardInfoArray.motorBoardInfo.resize(MOTOR_BOARD_NUM);
      for (uint32_t i = 0 ; i < MOTOR_BOARD_NUM; ++i) {
        motorBoardInfoArray.motorBoardInfo[i].header.stamp = ros::Time::now();
        motorBoardInfoArray.motorBoardInfo[i].header.frame_id = string("drrobot_battery_");
        motorBoardInfoArray.motorBoardInfo[i].header.frame_id += boost::lexical_cast<std::string>(i);

        motorBoardInfoArray.motorBoardInfo[i].status = motorBoardData_.status[i];
        motorBoardInfoArray.motorBoardInfo[i].temp1 = motorBoardData_.temp1[i];
        motorBoardInfoArray.motorBoardInfo[i].temp2 = motorBoardData_.temp2[i];
        motorBoardInfoArray.motorBoardInfo[i].temp3 = motorBoardData_.temp3[i];

        motorBoardInfoArray.motorBoardInfo[i].volMain = motorBoardData_.volMain[i];
        motorBoardInfoArray.motorBoardInfo[i].vol12V = motorBoardData_.vol12V[i];
        motorBoardInfoArray.motorBoardInfo[i].vol5V = motorBoardData_.vol5V[i];
        motorBoardInfoArray.motorBoardInfo[i].dinput = motorBoardData_.dinput[i];
        motorBoardInfoArray.motorBoardInfo[i].doutput = motorBoardData_.doutput[i];
        motorBoardInfoArray.motorBoardInfo[i].ack = motorBoardData_.ack[i];
      }
      //ROS_INFO("publish motor driver board info array");
      motorBoardInfo_pub_.publish(motorBoardInfoArray);

      // Warn about battery status
      if ((motorBoardData_.volMain[0]<20.0)&&(motorBoardData_.volMain[1]<20.0)&&(motorBoardData_.volMain[2]<20.0)) {
        ROS_INFO("Voltage is low. Please recharge batteries as soon as possible.");
      }

      sensor_msgs::Imu imuData;
      //imuData.header.seq = imuSensorData.seq;
      imuData.header.stamp = ros::Time::now();
      imuData.header.frame_id = string("imu_link");
      imu_roll_velocity_accumulator = 0;
      imu_pitch_velocity_accumulator = 0;
      imu_yaw_velocity_accumulator = 0;
      imu_x_acceleration_accumulator = 0;
      imu_y_acceleration_accumulator = 0;
      imu_z_acceleration_accumulator = 0;
      for (int i = 0; i < 10; i++) {
      	drrobotMotionDriver_->readIMUSensorData(&imuSensorData);
      	imu_roll_velocity_accumulator += imuSensorData.gyro_x;
      	imu_pitch_velocity_accumulator += imuSensorData.gyro_y;
      	imu_yaw_velocity_accumulator += imuSensorData.gyro_z;
      	imu_x_acceleration_accumulator += imuSensorData.accel_x;
      	imu_y_acceleration_accumulator += imuSensorData.accel_y;
      	imu_z_acceleration_accumulator += imuSensorData.accel_z;
      }

      imuData.linear_acceleration.x = (((float)imu_x_acceleration_accumulator)/10)/256*9.80665;
      imuData.linear_acceleration.y = (((float)imu_y_acceleration_accumulator)/10)/256*9.80665;
      imuData.linear_acceleration.z = (((float)imu_z_acceleration_accumulator)/10)/256*9.80665;
      imuData.linear_acceleration_covariance[0] = 0.01;
      imuData.linear_acceleration_covariance[4] = 0.01;
      imuData.linear_acceleration_covariance[8] = 0.01;

      imuData.angular_velocity.x = (((float)imu_roll_velocity_accumulator)/10 - average_angle_vel_x) / 14.375 * M_PI / 180;
      imuData.angular_velocity.y = (((float)imu_pitch_velocity_accumulator)/10 - average_angle_vel_y) / 14.375 * M_PI / 180;
      imuData.angular_velocity.z = (((float)imu_yaw_velocity_accumulator)/10 - average_angle_vel_z) / 14.375 * M_PI / 180;
      imuData.angular_velocity_covariance[0] = 0.01;
      imuData.angular_velocity_covariance[4] = 0.01;
      imuData.angular_velocity_covariance[8] = 0.01;
      // ROS_INFO("publish IMU sensor data");
        //imu_pub_.publish(imuData);

        //ROS_INFO("compass x: %d, compass y: %d, compass z: %d", imuSensorData.comp_x, imuSensorData.comp_y, imuSensorData.comp_z);
        //geometry_msgs::Vector3Stamped imuMagFieldData;
        //imuMagFieldData.header.seq = imuSensorData.seq;
        //imuMagFieldData.header.stamp = ros::Time::now();
        //imuMagFieldData.header.frame_id = string("imu_link");
        //imuMagFieldData.vector.x = (float)imuSensorData.comp_x;
        //imuMagFieldData.vector.y = (float)imuSensorData.comp_y;
        //imuMagFieldData.vector.z = (float)imuSensorData.comp_z;
        // ROS_INFO("publish compass sensor data");
        //compass_pub_.publish(imuMagFieldData);

        sensor_msgs::NavSatFix gpsInfo;
        gpsInfo.header.stamp = ros::Time::now();
        gpsInfo.header.frame_id = string("gps_link");

        gpsInfo.status.status = gpsSensorData_.gpsStatus;
      	gpsInfo.latitude = gpsSensorData_.latitude;
      	gpsInfo.longitude = gpsSensorData_.longitude;
        gpsInfo.position_covariance_type = 0;
      	// ROS_INFO("publish GPS Info");
      	gps_pub_.publish(gpsInfo);

      // send ping command here
      drrobotMotionDriver_->sendCommand("PING",4);
    }
  private:
    DrRobotMotionSensorDriver* drrobotMotionDriver_;

    struct DrRobotMotionConfig robotConfig1_;

    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct MotorBoardData motorBoardData_;
    struct IMUSensorData imuSensorData;
    struct GPSSensorData gpsSensorData_;

    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    int  commPortNum_;
    int  encoderOneCircleCnt_;
    double wheelDis_;
    double wheelRadius_;
    int motorDir_;
    double minSpeed_;
    double maxSpeed_;

    double left_vel, right_vel;
    int LEFT_vel, RIGHT_vel;
    double vx, vtheta;

    float average_angle_vel_x, average_angle_vel_y, average_angle_vel_z;
    int imu_roll_velocity_accumulator, imu_pitch_velocity_accumulator, imu_yaw_velocity_accumulator;
    int imu_x_acceleration_accumulator, imu_y_acceleration_accumulator, imu_z_acceleration_accumulator;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "jaguar4x4_node");

  Jaguar_Controller_Node drrobotPlayer;
  ros::NodeHandle n;
  // Start up the robot
  if (drrobotPlayer.start() != 0) {
      exit(-1);
  }
  /////////////////////////////////////////////////////////////////

  ros::Rate loop_rate(50); //10Hz

  while (n.ok()) {
    drrobotPlayer.doUpdate();

    ros::spinOnce();
    loop_rate.sleep();
  }
  /////////////////////////////////////////////////////////////////
  // Stop the robot
  drrobotPlayer.stop();

  return(0);
}
