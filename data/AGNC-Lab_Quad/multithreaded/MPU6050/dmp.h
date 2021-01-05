
#ifndef _DMP_H
#define _DMP_H



#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2C/i2c.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <fstream>
#include <ctime>
#include <vector>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu_head(0x68);

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// #define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
// #define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpHeadReady = false;  // set true if DMP init was successful
uint8_t mpu_headIntStatus;   // holds actual interrupt status byte from MPU
uint8_t mpu_headStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t headPacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t headFifoCount;     // count of all bytes currently in FIFO
uint8_t headFifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gx, gy, gz;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu_head.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu_head.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    
    // load and configure the DMP
    printf("Initializing DMP...\n");
    mpu_headStatus = mpu_head.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (mpu_headStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu_head.setDMPEnabled(true);
        //mpu_body.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpu_headIntStatus = mpu_head.getIntStatus();
        //mpu_bodyIntStatus = mpu_head.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpHeadReady = true;
        //dmpBodyReady = true;

        // get expected DMP packet size for later comparison
        headPacketSize = mpu_head.dmpGetFIFOPacketSize();
        //bodyPacketSize = mpu_body.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", mpu_headStatus);
        
    }
    
    
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


int getDMP() {
    // if programming failed, don't try to do anything
    //while(true) {
	if (dmpHeadReady)
	{
	    // get current FIFO count
	    headFifoCount = mpu_head.getFIFOCount();

        //printf("FIFOSize: %d\n", headFifoCount);

        if (headFifoCount >= 42) {
        // read a packet from FIFO
        mpu_head.getFIFOBytes(headFifoBuffer, headPacketSize);

        mpu_head.getRotation(&gx, &gy, &gz);
        mpu_head.dmpGetQuaternion(&q, headFifoBuffer);
        mpu_head.dmpGetGravity(&gravity, &q);
        mpu_head.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu_head.dmpGetAccel(&aa, headFifoBuffer);
        //mpu_head.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //mpu_head.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        // printf("areal %f %f %f  \n", gravity.x, gravity.y, gravity.z);
        // printf("areal %d %d %d  \n", aa.x, aa.y, aa.z);

        return 1;

        }

	    if (headFifoCount == 1024) {
		// reset so we can continue cleanly
		mpu_head.resetFIFO();
		printf("FIFO overflow!\n");

        return 0;

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	    } 
	} 
    return 0;
}

#endif