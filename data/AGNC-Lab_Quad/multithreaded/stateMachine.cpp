
#include "threads/stateMachine.h"
#include "pevents/pevents.cpp"



extern pthread_mutex_t stateMachine_Mutex;
extern int threadCount;	
extern int currentState;
extern neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_KeyESC;
extern neosmart_event_t e_Key6, e_Key7, e_Key8, e_Key9, e_endInit;
