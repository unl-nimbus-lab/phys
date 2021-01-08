
#include "threads/keyboard_thread.h"

char getch() { //This function allows to capture char without needing to press 'ENTER'
	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 0;
	old.c_cc[VTIME] = 5;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		    perror("tcsetattr ICANON");
	if (read(0, &buf, 1) < 0)
		    perror ("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		    perror ("tcsetattr ~ICANON");
	return (buf);
}

void *KeyboardTask(void *threadID)
{
	char ch;	//Char for storing keystroke
	printf("KeyboardTask has started!\n");
	int Key1 = 0, Key2 = 0, Key3 = 0, Key4 = 0, Key5 = 0, Key6 = 0, Key7 = 0, Key8 = 0, Key9 = 0;//Values for key states (0 or 1)
	int localCurrentState;

	while(1){
		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}


		ch = getch();
		if (ch == '1'){
			Key1 = (Key1 + 1) % 2;
			if (Key1 == 1)
			    SetEvent(e_Key1);
			else
			    ResetEvent(e_Key1);
			printf("Key1 State: %d\n",Key1);
		}
		else if (ch == '2'){
			Key2 = (Key2 + 1) % 2;
			if (Key2 == 1)
			    SetEvent(e_Key2);
			else
			    ResetEvent(e_Key2);
			printf("Key2 State: %d\n",Key2);
		}
		else if (ch == '3'){
			Key3 = (Key3 + 1) % 2;
			if (Key3 == 1)
			    SetEvent(e_Key3);
			else
			    ResetEvent(e_Key3);
			printf("Key3 State: %d\n",Key3);
		}
		else if (ch == '4'){
			Key4 = (Key4 + 1) % 2;
			if (Key4 == 1)
			    SetEvent(e_Key4);
			else
			    ResetEvent(e_Key4);
			printf("Key4 State: %d\n",Key4);
		}
		else if (ch == '5'){
			Key5 = (Key5 + 1) % 2;
			if (Key5 == 1)
			    SetEvent(e_Key5);
			else
			    ResetEvent(e_Key5);
			printf("Key5 State: %d\n",Key5);
		}
		else if (ch == '6'){
			Key6 = (Key6 + 1) % 2;
			if (Key6 == 1)
			    SetEvent(e_Key6);
			else
			    ResetEvent(e_Key6);
			printf("Key6 State: %d\n",Key6);
		}
		else if (ch == '7'){
			Key7 = (Key7 + 1) % 2;
			if (Key7 == 1)
			    SetEvent(e_Key7);
			else
			    ResetEvent(e_Key7);
			printf("Key7 State: %d\n",Key7);
		}
		else if (ch == '8'){
			Key8 = (Key8 + 1) % 2;
			if (Key8 == 1)
			    SetEvent(e_Key8);
			else
			    ResetEvent(e_Key8);
			printf("Key8 State: %d\n",Key8);
		}
		else if (ch == '9'){
			Key9 = (Key9 + 1) % 2;
			if (Key9 == 1)
			    SetEvent(e_Key9);
			else
			    ResetEvent(e_Key9);
			printf("Key9 State: %d\n",Key9);
		}
		else if (ch == '0'){
			pthread_mutex_lock(&PID_Mutex);
				updatePar(&PID_att, &PID_angVel, &PID_pos,"configAtt.txt","configPos.txt");
				PrintVec3(PID_att.K_p, "PID_att Kp");
			    PrintVec3(PID_att.K_i, "PID_att Ki");
			    PrintVec3(PID_att.K_d, "PID_att Kd");
			    PrintVec3(PID_angVel.K_p, "PID_w Kp");
			    PrintVec3(PID_angVel.K_i, "PID_w Ki");
			    PrintVec3(PID_angVel.K_d, "PID_w Kd");
			    PrintVec3(PID_pos.K_p, "PID_pos Kp");
			    PrintVec3(PID_pos.K_i, "PID_pos Ki");
			    PrintVec3(PID_pos.K_d, "PID_pos Kd");
			pthread_mutex_unlock(&PID_Mutex);
		}
		else if (ch == 'w') { //the 'w' key
		    //motor up
		    SetEvent(e_Motor_Up);
		}
		else if (ch == 's') { //the 's' key
		    //motor down
		    SetEvent(e_Motor_Down);
		}
		else if (ch == 'k') { //the 'k' key
		    //motor kill
		    SetEvent(e_Motor_Kill);
		}
		else if (ch == 'v') { //the 'v' key
			//Switch attitude measurement vicon / IMU
		    SetEvent(e_SwitchYawSource);
		}
		else if (ch == 27){	//ESC character
			SetEvent(e_KeyESC);
			printf("ESC has been activated!\n\n");
			//break;
		}

	}
	
	printf("KeyboardTask stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}

void *Motor_KeyboardControl(void *threadID){

	printf("Motor_Control has started!\n");
	int SamplingTime = 10;	//Sampling time in milliseconds
	float localThrust;
	int localCurrentState;

        //Initialize here
	
	while(1){
		WaitForEvent(e_Timeout,SamplingTime);
		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}
		
		if (WaitForEvent(e_Motor_Up, 0) == 0) {
		    //motor up
		    pthread_mutex_lock(&ThrustJoy_Mutex);
		    if (ThrustJoy < Thrust_Max)
				ThrustJoy += Thrust_Inc;
			localThrust = ThrustJoy;
		    pthread_mutex_unlock(&ThrustJoy_Mutex);
		    printf("Motor Speed: %f\n", localThrust);
		}
		else if (WaitForEvent(e_Motor_Down, 0) == 0) {
		    //motor down
		    pthread_mutex_lock(&ThrustJoy_Mutex);
		    if (ThrustJoy > Thrust_Min)
				ThrustJoy -= Thrust_Inc;
			localThrust = ThrustJoy;
		    pthread_mutex_unlock(&ThrustJoy_Mutex);
		    printf("Motor Speed: %f\n", localThrust);
		}
		else if (WaitForEvent(e_Motor_Kill, 0) == 0) {
		    //motor kill
		    pthread_mutex_lock(&ThrustJoy_Mutex);
		    ThrustJoy = 0;
		    localThrust = ThrustJoy;
		    pthread_mutex_unlock(&ThrustJoy_Mutex);
		    printf("Motor Speed: %f\n", localThrust);
		}
	}
	
	printf("Motor_Control stopping...\n");
	//Shutdown here
	threadCount -= 1;
	pthread_exit(NULL);
}