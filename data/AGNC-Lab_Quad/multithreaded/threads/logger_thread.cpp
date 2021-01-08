
#include "logger_thread.h"

void *Logger_Timer(void *threadID){

	printf("Logger_Timer has started!\n");
	int SamplingTime = 5;	//Sampling time in milliseconds
	int localCurrentState;

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

		SetEvent(e_Log_trigger);

	}
	
	printf("Logger_Timer stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}

void *Log_Task(void *threadID){
	printf("Log_Task has started!\n");

	Matrix<float, 3, 1> IMU_localData_RPY;
	Matrix<float, 3, 1> IMU_localData_Accel;
	Matrix<float, 4, 1> IMU_localData_Quat;
	Matrix<float, 3, 1> IMU_localData_AngVel;
	qcontrol_defs::PVA localPVA_quadVicon;
	int SamplingTime = 5;
	long k = 0;

	int localCurrentState;

	std::ofstream ofs ("LogData.txt", std::ofstream::out);

	while(1){
		
		WaitForEvent(e_Log_trigger,500);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

		pthread_mutex_lock(&IMU_Mutex);
			IMU_localData_RPY = IMU_Data_RPY;
			IMU_localData_Accel = IMU_Data_Accel;
			IMU_localData_Quat = IMU_Data_Quat;
			IMU_localData_AngVel = IMU_Data_AngVel;
		pthread_mutex_unlock(&IMU_Mutex);

 	pthread_mutex_lock(&PVA_Vicon_Mutex);	
		localPVA_quadVicon = PVA_quadVicon;
  	pthread_mutex_unlock(&PVA_Vicon_Mutex);

		//ofs << localPVA_quadVicon.t.sec+localPVA_quadVicon.t.nsec*pow(10,-9) << ", ";
  		ofs << double(k*SamplingTime)/1000.0 << ", ";
		ofs << IMU_localData_RPY(0) << ", " << IMU_localData_RPY(1) << ", " << IMU_localData_RPY(2) << ", ";
		ofs << IMU_localData_Accel(0) << ", " << IMU_localData_Accel(1) << ", " << IMU_localData_Accel(2) << ", ";
		ofs << IMU_localData_AngVel(0) << ", " << IMU_localData_AngVel(1) << ", " << IMU_localData_AngVel(2) << ", ";
		ofs << IMU_localData_Quat(0) << ", " << IMU_localData_Quat(1) << ", " << IMU_localData_Quat(2) << ", " << IMU_localData_Quat(3) << ", ";
		ofs << localPVA_quadVicon.pos.position.x << ", " << localPVA_quadVicon.pos.position.y << ", " << localPVA_quadVicon.pos.position.z << ", ";
		ofs << localPVA_quadVicon.pos.orientation.w << ", " << localPVA_quadVicon.pos.orientation.x << ", " << localPVA_quadVicon.pos.orientation.y << ", " << localPVA_quadVicon.pos.orientation.z << endl;


		k += 1;
			
	}
	ofs.close();

	printf("Log_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);

}