//============================================================================
// Name        : Vicon Xbee Communication
// Author      : Britsk Nguyen
// Version     :
// Copyright   : Love me if you use my code
// Description : ros node to communicate with vicon via serial interface
//============================================================================
//#define ROS_MASTER_URI		"http://localhost:11311"
//#define ROS_ROOT		"/opt/ros/indigo/share/ros"
#include <iostream>
#include "stdio.h"
#include <stdlib.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <cstring>
#include <unistd.h>
#include <sstream>
#include <signal.h>
#include <math.h>
#include <viconxbee/viconPoseMsg.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign/list_of.hpp>
#include <errno.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>

//#include <mavros/Imu.h>

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

#define USE_MIT_SERIAL
#define BAUDRATE            57600
#define BAUD_MACRO          B57600
#define DFLT_PORT           "/dev/ttyUSB0"
#define RCV_THRESHOLD       46					//Receive frame's length
#define FPS                 40					//Frames per second
#define DFLT_NODE_RATE      20                  //Update frequency
#define DFLT_COMM_TIMEOUT   250
#define	UP                  0xFF
#define DOWN                0x00
#define HEADER_NAN          0x7FAAAAAA

using namespace std;
serial::Serial fd;

uint8_t		frame32[RCV_THRESHOLD];
uint8_t		rcvdFrame[RCV_THRESHOLD];
uint8_t		backupFrame[RCV_THRESHOLD];
uint8_t		flagIncompleteFrame = 0;
int			rcvdBytesCount = 0;
uint8_t		backupRcvBytesCount = 0;
uint8_t		misAlgnFlag = DOWN;
uint8_t		msgFlag = DOWN;
uint8_t		viconUpdateFlag = DOWN;
uint8_t		publishFlag = DOWN;
uint8_t		headerIndex = 0xFF;
uint32_t	seqCount = 0;
uint32_t viconStartTimestamp;
ros::Time viconRosStartTimestamp;

#define VICON_MSG_HDR       *(uint32_t *)rcvdFrame
#define VICON_MSG_X         *(float *)(rcvdFrame + 4)
#define VICON_MSG_Y         *(float *)(rcvdFrame + 8)
#define VICON_MSG_Z         *(float *)(rcvdFrame + 12)
#define VICON_MSG_XD        *(float *)(rcvdFrame + 16)
#define VICON_MSG_YD        *(float *)(rcvdFrame + 20)
#define VICON_MSG_ZD        *(float *)(rcvdFrame + 24)
#define VICON_MSG_ROLL      *(float *)(rcvdFrame + 28)
#define VICON_MSG_PITCH     *(float *)(rcvdFrame + 32)
#define VICON_MSG_YAW      *(float *)(rcvdFrame + 36)
#define VICON_MSG_STMP      *(uint32_t *)(rcvdFrame + 40)
#define VICON_MSG_CSA       *(uint8_t *)(rcvdFrame + 44)
#define VICON_MSG_CSB       *(uint8_t *)(rcvdFrame + 45)

long validCount = 0;
long faultCount = 0;
long freeCount = 0;
long loopCount = 0;

void signal_handler_IO(int status)
{
    msgFlag = UP;
}

int synchronize(ros::Rate rate, double timeOut);

int main(int argc, char **argv)
{
    //Create ros handler to node
    ros::init(argc, argv, "viconXbeeNode");
    ros::NodeHandle viconXbeeNode("~");

    string serialPortName = string(DFLT_PORT);
    if(viconXbeeNode.getParam("viconSerialPort", serialPortName))
        printf(KBLU"Retrieved value %s for param 'viconSerialPort'!\n"RESET, serialPortName.data());
    else
    {
        printf(KRED "Couldn't retrieve param 'viconSerialPort', program closed!\n"RESET);
        return 0;
    }
    int viconNodeRate = DFLT_NODE_RATE;
    if(viconXbeeNode.getParam("viconNodeRate", viconNodeRate))
        printf(KBLU"Retrieved value %d for param 'viconnNodeRate'\n"RESET, viconNodeRate);
    else
        printf(KYEL "Couldn't retrieve param 'viconnNodeRate', applying default value %dHz\n"RESET, viconNodeRate);

    int viconCommTimeout = DFLT_COMM_TIMEOUT;
    if(viconXbeeNode.getParam("viconCommTimeout", viconCommTimeout))
        printf(KBLU"Retrieved value %d [ms] for param 'viconCommTimeout'\n"RESET, viconCommTimeout);
    else
        printf(KYEL "Couldn't retrieve param 'viconCommTimeout', applying default value %dms\n"RESET, viconCommTimeout);

    //ros Rate object to control the update rate
    ros::Rate rate = ros::Rate(viconNodeRate);

    ros::Publisher viconPosePublisher = viconXbeeNode.advertise<viconxbee::viconPoseMsg>("viconPoseTopic", 1);
    ros::Publisher viconMocapPublisher = viconXbeeNode.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);
    ros::Publisher viconIMUPublisher = viconXbeeNode.advertise<sensor_msgs::Imu>("mocap/imu", 1);
    ros::Publisher twistPublisher = viconXbeeNode.advertise<geometry_msgs::TwistWithCovarianceStamped>("mocap/twist", 1);
    tf::Quaternion last_quaternion(0,0,0);
    geometry_msgs::PoseStamped poseStamped;
    geometry_msgs::TwistWithCovarianceStamped twist;
    twist.twist.covariance[0*7] = 0.25;
    twist.twist.covariance[1*7] = 0.25;
    twist.twist.covariance[2*7] = 0.25;
    twist.twist.covariance[3*7] = 0.03;
    twist.twist.covariance[4*7] = 0.03;
    twist.twist.covariance[5*7] = 0.03;




    //-------------------Initialize Serial Connection---------------------------------------------------
    fd.setPort(serialPortName);
    fd.setBaudrate(BAUDRATE);
    fd.setTimeout(5, 10, 0, 10, 0);
    fd.open();
    if (fd.isOpen())
    {
        fd.flushInput();
        printf(KBLU "Connection established\n\n" RESET);
    }
    else
    {
        printf(KRED "serialInit: Failed to open port %s\n" RESET, serialPortName.data());
        return 0;
    }
    //-----------------------------------Serial Initialization-------------------------------------
    //Looping to catch the intial of frame
    if( synchronize(rate, viconCommTimeout/1000.0) < 0)
    {
        printf("Timeout synchronizing with stream. Exitting!");
        exit(1);
    }

    viconStartTimestamp = VICON_MSG_STMP;
    viconRosStartTimestamp = ros::Time::now();

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    //start reading and publishing
    poseStamped.header.stamp =  ros::Time::now();
    while(ros::ok())
    {
        uint32_t bytes_avail = fd.available();
        printf("Bytes detected: %lu\n", fd.available());
        //Extract and publish all messages if there are enough bytes in waiting

            if(bytes_avail > RCV_THRESHOLD)
                while(fd.available() >= RCV_THRESHOLD)
                {
                    fd.read(rcvdFrame, RCV_THRESHOLD);
                    //Since intial synchronization, first 4 bytes must be headers otherwise there must have been an error
                    uint32_t headerWord = VICON_MSG_HDR;
                    if(headerWord != HEADER_NAN)
                    {
                        //If there is some error, call the synchronize procedure
                        printf("Lost synchronization. Attempting to synchronize back.\n");
                        if( synchronize(rate, viconCommTimeout/1000.0) < 0)
                        {
                            printf("Timeout synchronizing with stream. Exitting!");
                            exit(1);
                        }
                        printf("Synchronization recovered.\n");

                    }
                    else
                    {
                        //All things are good so far
                        printf(KBLU"Aligned Header found: "RESET);
                        for(int i = 0; i < RCV_THRESHOLD; i++)
                            printf(KGRN"%2x ", rcvdFrame[i]);
                        printf("\n"RESET);

                        //Checksum
                        bool validCRC = false;
                        uint8_t CSA = 0, CSB = 0;
                        for(uint8_t i = sizeof(HEADER_NAN); i < RCV_THRESHOLD - 2; i++)
                        {
                            CSA = CSA + rcvdFrame[i];
                            CSB = CSB + CSA;
                        }

                        if(CSA == VICON_MSG_CSA && CSB == VICON_MSG_CSB)
                        {
                            validCRC = true;
                            printf(KBLU"CRC valid, msg published!!\n"RESET);
                        }
                        else
                            printf(KRED"CRC error, msg discarded!!\n"RESET);

                        if(validCRC)
                        {
                            viconxbee::viconPoseMsg viconPose;
                            double timeDiff = (int32_t)(VICON_MSG_STMP - viconStartTimestamp)/1000.0;
                            if(timeDiff < 0)
                            {
                                printf(KYEL"Vicon time must have reset.\n");
                                viconStartTimestamp = VICON_MSG_STMP;
                                viconRosStartTimestamp = ros::Time::now();
                            }
                            viconPose.time_stamp = viconRosStartTimestamp + ros::Duration((VICON_MSG_STMP - viconStartTimestamp)/1000.0);
                            viconPose.x = VICON_MSG_X;
                            viconPose.y = VICON_MSG_Y;
                            viconPose.z = VICON_MSG_Z;
                            viconPose.dx = VICON_MSG_XD;
                            viconPose.dy = VICON_MSG_YD;
                            viconPose.dz = VICON_MSG_ZD;
                            viconPose.roll = VICON_MSG_ROLL;
                            viconPose.pitch = VICON_MSG_PITCH;
                            viconPose.yaw = VICON_MSG_YAW;
                            viconPosePublisher.publish(viconPose);


                            //publish to mocap/pose topic
                            std_msgs::Header header;
                            header.frame_id = "vicon";
                            header.stamp = ros::Time::now();
                            double dt = poseStamped.header.stamp.toSec()-header.stamp.toSec();
                            poseStamped.header = header;
                            poseStamped.pose.position.x = viconPose.x;
                            poseStamped.pose.position.y = viconPose.y;
                            poseStamped.pose.position.z = viconPose.z;
                            tf::Quaternion q = tf::createQuaternionFromRPY(viconPose.roll, viconPose.pitch, viconPose.yaw);
                            poseStamped.pose.orientation.x = q.x();
                            poseStamped.pose.orientation.y = q.y();
                            poseStamped.pose.orientation.z = q.z();
                            poseStamped.pose.orientation.w = q.w();
                            viconMocapPublisher.publish(poseStamped);
                            seqCount++;


                            twist.header = poseStamped.header;
                            twist.twist.twist.linear.x = viconPose.dx;
                            twist.twist.twist.linear.y = viconPose.dy;
                            twist.twist.twist.linear.z = viconPose.dz;
                            tf::Quaternion quaternion;
                            tf::quaternionMsgToTF(poseStamped.pose.orientation, quaternion);
                            tf::Quaternion d_q = last_quaternion.inverse()*quaternion;
                            tf::Matrix3x3 m(d_q);
                            double roll, pitch, yaw;
                            m.getRPY(roll, pitch, yaw);
                            last_quaternion = quaternion;
                            twist.twist.twist.angular.x = roll/dt;
                            twist.twist.twist.angular.y = pitch/dt;
                            twist.twist.twist.angular.z = yaw/dt;

                            sensor_msgs::Imu imu;
                            imu.header = poseStamped.header;
                            imu.orientation = poseStamped.pose.orientation;
                            imu.linear_acceleration.x = poseStamped.pose.position.x;
                            imu.linear_acceleration.y = poseStamped.pose.position.y;
                            imu.linear_acceleration.z = poseStamped.pose.position.z;
                            viconIMUPublisher.publish(imu);

                            transform.setOrigin(tf::Vector3(viconPose.x, viconPose.y, viconPose.z));
                            transform.setRotation(q);
                            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "vicon", "robot_base"));
                        }
                    }
                }
        else
            printf(KBLU"No message has arrived...\n"RESET);
        printf("Bytes still buffered: %lu\n\n", fd.available());
        rate.sleep();
    }
    return 0;
}

int synchronize(ros::Rate rate, double timeOut)
{
    ros::Time synchStartTime = ros::Time::now();
    while(ros::ok())
    {
        uint32_t bytes_avail = fd.available();
        printf("Bytes in buffer%d\n", bytes_avail);

        if(bytes_avail < RCV_THRESHOLD*2)
        {
            rate.sleep();
            continue;
        }
        else
        {
            //dummy read to clean up the buffer
            fd.read(rcvdFrame, 4);
            uint32_t headerWord = VICON_MSG_HDR;
            while((headerWord != HEADER_NAN) || (bytes_avail >= RCV_THRESHOLD*2))
            {
                bytes_avail = fd.available();
                headerWord = (headerWord >> 8) & (0x00FFFFFF);
                fd.read(rcvdFrame, 1);
                headerWord = headerWord | (rcvdFrame[0] << 24);
                printf("%x\n", headerWord);
                if(headerWord == HEADER_NAN)
                {
                    VICON_MSG_HDR = HEADER_NAN;
                    fd.read(rcvdFrame + 4, RCV_THRESHOLD - 4);
                    headerWord = HEADER_NAN;
                }
            }
            break;
        }

        if((ros::Time::now() - synchStartTime).toSec() > timeOut)
            return -1;
    }
    return (int)((ros::Time::now() - synchStartTime).toSec()*1000);
}
