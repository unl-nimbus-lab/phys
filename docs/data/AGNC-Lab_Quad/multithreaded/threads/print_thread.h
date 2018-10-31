
#ifndef _H_PRINT_THREAD_
#define _H_PRINT_THREAD_

#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>
#include "control/QuatRotEuler.h"
#include <cstdio>
#include "rosserial/qcontrol_defs/PVA.h"
#include "control/MatricesAndVectors.h"
#include "control/PID_3DOF.h"
#include "threads/stateMachine.h"

#define PI 3.1415

using namespace neosmart;


extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t PCA_Mutex;
extern pthread_mutex_t Contr_Input_Mutex;
extern pthread_mutex_t attRef_Mutex;
extern pthread_mutex_t PVA_Vicon_Mutex;
extern pthread_mutex_t attRefJoy_Mutex;
extern pthread_mutex_t PID_Mutex;
extern pthread_mutex_t ThrustPosControl_Mutex;
extern pthread_mutex_t PVA_Kalman_Mutex;
extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent


extern Matrix<float, 3, 1> attRefJoy;
extern Matrix<float, 4, 1> PCA_Data;
extern Matrix<float, 3, 1> IMU_Data_RPY;
extern Matrix<float, 4, 1> IMU_Data_Quat;
extern Matrix<float, 3, 1> IMU_Data_Accel;
extern Matrix<float, 3, 1> IMU_Data_AngVel;
extern Matrix<float, 4, 1> Contr_Input;
extern qcontrol_defs::PVA PVA_quadVicon;
extern qcontrol_defs::PVA PVA_quadKalman;
extern PID_3DOF PID_att;
extern float ThrustPosControl;

extern int threadCount;	
extern int currentState;
extern neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_KeyESC;
extern neosmart_event_t e_Key6, e_Key7, e_Key8, e_Key9;
//Control printing to terminal
void *PrintTask(void *threadID);

#endif