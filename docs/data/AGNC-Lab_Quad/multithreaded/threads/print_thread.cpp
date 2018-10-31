
#include "threads/print_thread.h"


void *PrintTask(void *threadID){
	printf("PrintTask has started!\n");
	Matrix<float, 3, 1> localIMU_Data_RPY;
	Matrix<float, 4, 1> localIMU_Data_Quat;
	Matrix<float, 3, 1> localIMU_Data_Accel;
	Matrix<float, 3, 1> localIMU_Data_Vel;
	Matrix<float, 3, 1> localattRef;
	Matrix<float, 3, 1> localViconPos;
	Matrix<float, 3, 1> localViconVel;
	Matrix<float, 4, 1> localContr_Input;
	Matrix<float, 4, 1> localPCA_Data;
	float localMotor_Speed;
	int localCurrentState;

	while(1){
	    WaitForEvent(e_Timeout,100);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

	    if(WaitForEvent(e_Key1, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
			localIMU_Data_RPY = IMU_Data_RPY;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_RPY = %-7f %-7f %-7f\n", localIMU_Data_RPY(0)*180/PI, localIMU_Data_RPY(1)*180/PI, localIMU_Data_RPY(2)*180/PI);
	    }
	    if(WaitForEvent(e_Key2, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
			localIMU_Data_Quat = IMU_Data_Quat;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Quat = %-7f %-7f %-7f %-7f\n", localIMU_Data_Quat(1), localIMU_Data_Quat(2), localIMU_Data_Quat(3), localIMU_Data_Quat(0));
	    }
	    if(WaitForEvent(e_Key3, 0) == 0)
	    {	
		pthread_mutex_lock(&PVA_Kalman_Mutex);
			localIMU_Data_Accel(0) = PVA_quadKalman.acc.linear.x;
			localIMU_Data_Accel(1) = PVA_quadKalman.acc.linear.y;
			localIMU_Data_Accel(2) = PVA_quadKalman.acc.linear.z;
		pthread_mutex_unlock(&PVA_Kalman_Mutex);
		//WaitForEvent(e_Timeout,5);
		// pthread_mutex_lock(&IMU_Mutex);
		// 	localIMU_Data_Accel = IMU_Data_Accel;
		// pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Accel = %-7f %-7f %-7f\n", localIMU_Data_Accel(0), localIMU_Data_Accel(1), localIMU_Data_Accel(2));
	    }
	    if(WaitForEvent(e_Key4, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&Contr_Input_Mutex);
			localContr_Input = Contr_Input;
		pthread_mutex_unlock(&Contr_Input_Mutex);
		pthread_mutex_lock(&PCA_Mutex);
			localPCA_Data = PCA_Data;
		pthread_mutex_unlock(&PCA_Mutex);
		PrintVec4(localContr_Input, "Contr_Input");
		PrintVec4(localPCA_Data, "PCA_Data");
	    }
	    if(WaitForEvent(e_Key5, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
			localIMU_Data_Vel = IMU_Data_AngVel;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Vel = %-7f %-7f %-7f\n", localIMU_Data_Vel(0), localIMU_Data_Vel(1), localIMU_Data_Vel(2));
	    }
	    if(WaitForEvent(e_Key6, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&attRefJoy_Mutex);
			localattRef = attRefJoy;
		pthread_mutex_unlock(&attRefJoy_Mutex);
		printf("Attitide Reference (RPY) = %-7f %-7f %-7f\n", localattRef(0), localattRef(1), localattRef(2));
	    }
	    if(WaitForEvent(e_Key7, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&PVA_Vicon_Mutex);	
			localViconPos(0) = PVA_quadVicon.pos.position.x;
			localViconPos(1) = PVA_quadVicon.pos.position.y;
			localViconPos(2) = PVA_quadVicon.pos.position.z;
			localViconVel(0) = PVA_quadVicon.vel.linear.x;
			localViconVel(1) = PVA_quadVicon.vel.linear.y;
			localViconVel(2) = PVA_quadVicon.vel.linear.z;
	  	pthread_mutex_unlock(&PVA_Vicon_Mutex);	
		printf("Vicon Position = %-7f %-7f %-7f\n", localViconPos(0), localViconPos(1), localViconPos(2));
		printf("Vicon Velocity = %-7f %-7f %-7f\n", localViconVel(0), localViconVel(1), localViconVel(2));
	    }
	    if(WaitForEvent(e_Key8, 0) == 0)
	    {
			pthread_mutex_lock(&PID_Mutex);
				PrintVec3(PID_att.e_integ, "e_integ Att");
			pthread_mutex_unlock(&PID_Mutex);
	     }
	    if(WaitForEvent(e_Key9, 0) == 0)
	    {
			pthread_mutex_lock(&ThrustPosControl_Mutex);
				localMotor_Speed = ThrustPosControl;
			pthread_mutex_unlock(&ThrustPosControl_Mutex);
			printf("ThrustPosControl = %f\n",localMotor_Speed);
	     }




	}
	

	printf("PrintTask stopping...\n");	
	threadCount -= 1;
	pthread_exit(NULL);
}
