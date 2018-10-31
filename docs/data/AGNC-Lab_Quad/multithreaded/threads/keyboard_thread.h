

#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "pevents/pevents.h"
#include "threads/stateMachine.h"
#include <pthread.h>
#include <unistd.h>
#include <termios.h>
#include "control/PID_3DOF.h"


using namespace neosmart;

extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t ThrustJoy_Mutex;
extern pthread_mutex_t PID_Mutex;
extern neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_Key6, e_Key7, e_Key8, e_Key9, e_KeyESC;
extern neosmart_event_t e_Motor_Up, e_Motor_Down, e_Motor_Kill;
extern neosmart_event_t e_SwitchYawSource;
extern int threadCount;
extern int currentState;
extern float ThrustJoy;

extern PID_3DOF PID_angVel, PID_att, PID_pos; 	//Control PIDs

#define Thrust_Max 1.5
#define Thrust_Min 0.0
#define Thrust_Inc 0.15


//Keyboard inputs
void *KeyboardTask(void *threadID);

//Control  motor speed with keyboard
void *Motor_KeyboardControl(void *threadID);


#endif