#ifndef _H_LOG_THREAD_
#define _H_LOG_THREAD_

#include "threads/stateMachine.h"
#include "control/QuatRotEuler.h"
#include "control/MatricesAndVectors.h"
#include "rosserial/qcontrol_defs/PVA.h"
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>

using std::string;
using std::ostringstream;
using namespace std;
using namespace neosmart;

extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t PVA_Vicon_Mutex;

extern Matrix<float, 3, 1> IMU_Data_RPY;
extern Matrix<float, 4, 1> IMU_Data_Quat;
extern Matrix<float, 3, 1> IMU_Data_Accel;
extern Matrix<float, 3, 1> IMU_Data_AngVel;

extern neosmart_event_t e_Log_trigger;
extern neosmart_event_t e_Timeout;

extern int threadCount;	
extern int currentState;

extern qcontrol_defs::PVA PVA_quadVicon;

void *Logger_Timer(void *threadID);

void *Log_Task(void *threadID);

#endif