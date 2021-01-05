#ifndef _H_CONTROL_THREAD_
#define _H_CONTROL_THREAD_

#include "control/QuatRotEuler.h"
#include "control/MathFuncs.h"
#include "control/AttitudeControl.h"
#include "control/PID_3DOF.h"
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>
#include <cstdio>
#include "threads/stateMachine.h"
//#include "rosserial/ros.h"
#include "rosserial/qcontrol_defs/PVA.h"
#include "control/MatricesAndVectors.h"
#define PI 3.1415
#include <Eigen/Dense>
using Eigen::Matrix;

using namespace neosmart;

extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
extern neosmart_event_t e_AttControl_trigger, e_PosControl_trigger;

extern pthread_mutex_t attRefJoy_Mutex;
extern pthread_mutex_t ThrustJoy_Mutex;
extern pthread_mutex_t attRefPosControl_Mutex;
extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t PCA_Mutex;
extern pthread_mutex_t PID_Mutex;
extern pthread_mutex_t PVA_Vicon_Mutex;
extern pthread_mutex_t posRefJoy_Mutex;
extern pthread_mutex_t posRefClient_Mutex;
extern pthread_mutex_t ThrustPosControl_Mutex;
extern pthread_mutex_t PVA_Kalman_Mutex;


extern Matrix<float, 3, 1> attRefJoy, angVelRefJoy;
extern Matrix<float, 4, 1> Contr_Input;
extern Matrix<float, 4, 1> PCA_Data;
extern Matrix<float, 3, 1> IMU_Data_RPY;
extern Matrix<float, 4, 1> IMU_Data_Quat, IMU_Data_Quat_ViconYaw;
extern Matrix<float, 3, 1> IMU_Data_AngVel;
extern PID_3DOF PID_angVel, PID_att, PID_pos; 	//APIDs structures
extern qcontrol_defs::PVA PVA_quadKalman, PVA_RefJoy, PVA_quadVicon, PVA_RefClient;
extern float ThrustPosControl;
extern Matrix<float, 3, 3> Rdes_PosControl;


extern pthread_mutex_t Contr_Input_Mutex;
extern float ThrustJoy;

extern int threadCount;	
extern int currentState;


//Timer for attitude controller
void *AttControl_Timer(void *threadID);

//Attitude controller
void *AttControl_Task(void *threadID);

//Timer for position controller
void *PosControl_Timer(void *threadID);

//Position controller
void *PosControl_Task(void *threadID);

#endif