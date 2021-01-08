

#include "control_thread.h"


void *AttControl_Timer(void *threadID){

	printf("AttControl_Timer has started!\n");
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

		SetEvent(e_AttControl_trigger);
	}
	
	printf("AttControl_Timer stopping...\n");
	//Shutdown here
	threadCount -= 1;
	pthread_exit(NULL);
}


void *AttControl_Task(void *threadID){
	printf("AttControl_Task has started!\n");

	//Initialize PIDs (sets initial errors to zero)
	pthread_mutex_lock(&PID_Mutex);
	initializePID(&PID_att);
	initializePID(&PID_angVel);
	pthread_mutex_unlock(&PID_Mutex);
	float dt = 0.005; 			//Sampling time
	float takeOffThrust = 0.3; //Minimum thrust for resetting integral control
	Matrix<float, 3, 1> localAttRef;

	//Vectors of zeros
	Matrix<float, 3, 1> Zero_3x1 = Matrix<float, 3, 1>::Zero(3, 1);
	Matrix<float, 3, 3> Zero_3x3 = Matrix<float, 3, 3>::Zero(3, 3);
	Matrix<float, 3, 1> feedforward = Zero_3x1;

	Matrix<float, 4, 1> IMU_localData_Quat;
	Matrix<float, 3, 1> IMU_localData_Vel;
	Matrix<float, 3, 1> IMU_localData_RPY;
	Matrix<float, 3, 1> inputTorque;
	Matrix<float, 4, 1> PCA_localData;
	float localThrust = 0;
	int localCurrentState;
	int localYawSource;


	Matrix<float, 3, 1> error_att;
	Matrix<float, 3, 1> error_att_vel;

	Matrix<float, 3, 1> wDes;
	Matrix<float, 3, 3> Rdes = Zero_3x3;
	Matrix<float, 3, 3> Rbw;
	

	//Matrix<float, 3, 3> Rdes = RPY2Rot(0,0,0);
	//PrintMatrix<float, 3, 3>(Concatenate3Matrix<float, 3, 1>_2_Matrix<float, 3, 3>(PID_att.K_p, PID_att.K_d, PID_att.K_i));
	//PrintMatrix<float, 3, 3>(Concatenate3Matrix<float, 3, 1>_2_Matrix<float, 3, 3>(PID_angVel.K_p, PID_angVel.K_d, PID_angVel.K_i));

	while(1){

		WaitForEvent(e_AttControl_trigger,500);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

		//Grab attitude reference
		if(localCurrentState == ATTITUDE_MODE){
			pthread_mutex_lock(&attRefJoy_Mutex);	
			    localAttRef = attRefJoy;
			    feedforward = angVelRefJoy;
		    pthread_mutex_unlock(&attRefJoy_Mutex);
			//Throttle
		    pthread_mutex_lock(&ThrustJoy_Mutex);
				localThrust = ThrustJoy;
			pthread_mutex_unlock(&ThrustJoy_Mutex);
		    Rdes = RPY2Rot(localAttRef(0), localAttRef(1), localAttRef(2)); //fix this
		}
		else if (localCurrentState == POSITION_JOY_MODE){
			pthread_mutex_lock(&attRefPosControl_Mutex);
				Rdes = Rdes_PosControl;
			pthread_mutex_unlock(&attRefPosControl_Mutex);
		    pthread_mutex_lock(&ThrustPosControl_Mutex);
				localThrust = ThrustPosControl;
			pthread_mutex_unlock(&ThrustPosControl_Mutex);
		}
		else if (localCurrentState == POSITION_ROS_MODE){
			pthread_mutex_lock(&attRefPosControl_Mutex);
				Rdes = Rdes_PosControl;
			pthread_mutex_unlock(&attRefPosControl_Mutex);
		    pthread_mutex_lock(&ThrustPosControl_Mutex);
				localThrust = ThrustPosControl;
			pthread_mutex_unlock(&ThrustPosControl_Mutex);
		}
		else if (localCurrentState == MOTOR_MODE){
		    pthread_mutex_lock(&ThrustJoy_Mutex);
				localThrust = ThrustJoy;
			pthread_mutex_unlock(&ThrustJoy_Mutex);
			inputTorque = Zero_3x1;
		}
		else{
			localThrust = 0;
		}

		//Control attitude if in attitude / position modes
		if((localCurrentState == ATTITUDE_MODE) ||
		   (localCurrentState == POSITION_JOY_MODE) ||
		   (localCurrentState == POSITION_ROS_MODE)){
			
			pthread_mutex_lock(&YawSource_Mutex);
				localYawSource = YawSource;
			pthread_mutex_unlock(&YawSource_Mutex);

			//Grab attitude estimation
			if (localYawSource == _IMU){
				pthread_mutex_lock(&IMU_Mutex);
					IMU_localData_Quat = IMU_Data_Quat;
					IMU_localData_Vel = IMU_Data_AngVel;
					IMU_localData_RPY = IMU_Data_RPY;
				pthread_mutex_unlock(&IMU_Mutex);
			}
			else{
				pthread_mutex_lock(&PVA_Vicon_Mutex);
					IMU_localData_Quat = IMU_Data_Quat_ViconYaw;
				pthread_mutex_unlock(&PVA_Vicon_Mutex);
				pthread_mutex_lock(&IMU_Mutex);
					IMU_localData_Vel = IMU_Data_AngVel;
					IMU_localData_RPY = IMU_Data_RPY;
				pthread_mutex_unlock(&IMU_Mutex);
			}

			Rbw = Quat2rot(IMU_localData_Quat);

		    //Calculate attitude error
		    error_att = AttitudeErrorVector(Rbw, Rdes);
		    //PrintMatrix<float, 3, 1>(error_att, "error_att"); 

			//Update PID
			pthread_mutex_lock(&PID_Mutex);
				if(!isNanVec3(error_att)){
					updateErrorPID(&PID_att, feedforward, error_att, Zero_3x1, dt);
				}

				//Dont integrate integrator if not in minimum thrust
				if (localThrust < takeOffThrust){
					resetIntegralErrorPID(&PID_att);
				}
				
				//Reference for inner loop (angular velocity control)
				wDes = outputPID(PID_att);

				//Calculate angular velocity error and update PID
				error_att_vel = wDes-IMU_localData_Vel;
				updateErrorPID(&PID_angVel, feedforward, error_att_vel, Zero_3x1, dt);

				if (localThrust < takeOffThrust){
					resetIntegralErrorPID(&PID_angVel);
				}

				//This scale of 16.0 should be excluded eventually (incorporate it in gains)
				inputTorque = outputPID(PID_angVel)*(1.0/16.0);
			pthread_mutex_unlock(&PID_Mutex);

		}

		//Distribute power to motors
		pthread_mutex_lock(&Contr_Input_Mutex);
			Contr_Input << localThrust,
						   inputTorque;
			PCA_localData = u2pwmXshape(Contr_Input);
		pthread_mutex_unlock(&Contr_Input_Mutex);

		//Send motor commands
		pthread_mutex_lock(&PCA_Mutex);
			PCA_Data = PCA_localData;
		pthread_mutex_unlock(&PCA_Mutex);

	}
	
	printf("AttControl_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}

void *PosControl_Timer(void *threadID){

	printf("PosControl_Timer has started!\n");
	int SamplingTime = 10;	//Sampling time in milliseconds
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

		SetEvent(e_PosControl_trigger);
	}
	
	printf("PosControl_Timer stopping...\n");
	//Shutdown here
	threadCount -= 1;
	pthread_exit(NULL);
}

void *PosControl_Task(void *threadID){
	printf("PosControl_Task has started!\n");

	float dt = 0.010; 			//Sampling time
	double m = 0.26, g_z = 9.81;  //Mass and gravity for quadcopter
	double nominalThrust = 2.5;
	int localCurrentState;
	float yawDesired;
	Matrix<float, 3, 1> e_Pos, e_Vel; 			//error in position and velocity
	Matrix<float, 3, 1> acc_Ref;				//Desired acceleration
	Matrix<float, 3, 1> feedForward;			//Feedforward vector
	Matrix<float, 3, 1> Fdes;
	Matrix<float, 3, 1> z_bdes, x_cdes, y_bdes, x_bdes;
	Matrix<float, 4, 1> IMU_localData_Quat;
	qcontrol_defs::PVA localPVAEst_quadVicon, localPVA_quadVicon, localPVA_Ref;
	float cos_YawHalf, sin_YawHalf;

	Matrix<float, 3, 1> z_w;	//z vector of the inertial frame
	Matrix<float, 3, 1> z_b;	//z vector of the body frame
	z_w << 0,
		   0,
		   1;

	//Initialize PIDs (sets initial errors to zero)
	pthread_mutex_lock(&PID_Mutex);
		initializePID(&PID_pos);
	pthread_mutex_unlock(&PID_Mutex);

	while(1){

		WaitForEvent(e_PosControl_trigger,500);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

		//Grab attitude estimation
		// pthread_mutex_lock(&IMU_Mutex);
		// 	IMU_localData_Quat = IMU_Data_Quat;
		// pthread_mutex_unlock(&IMU_Mutex);
		pthread_mutex_lock(&PVA_Vicon_Mutex);
			IMU_localData_Quat = IMU_Data_Quat_ViconYaw;
		pthread_mutex_unlock(&PVA_Vicon_Mutex);

		//Grab position and velocity estimation
		pthread_mutex_lock(&PVA_Kalman_Mutex);
			localPVAEst_quadVicon = PVA_quadKalman;
		pthread_mutex_unlock(&PVA_Kalman_Mutex);
	 	pthread_mutex_lock(&PVA_Vicon_Mutex);
	 		localPVA_quadVicon = PVA_quadVicon;
	  	pthread_mutex_unlock(&PVA_Vicon_Mutex);
		// pthread_mutex_lock(&PVA_Vicon_Mutex);	
		// 	localPVA_quadVicon = PVA_quadVicon;
	 //  	pthread_mutex_unlock(&PVA_Vicon_Mutex);	

	  	//Grab joystick position and velocity reference
		if(localCurrentState == POSITION_JOY_MODE){
			pthread_mutex_lock(&posRefJoy_Mutex);	
				localPVA_Ref = PVA_RefJoy;
		  	pthread_mutex_unlock(&posRefJoy_Mutex);	
		  }
		else if(localCurrentState == POSITION_ROS_MODE){
			pthread_mutex_lock(&posRefJoy_Mutex);	
				localPVA_Ref = PVA_RefClient;
		  	pthread_mutex_unlock(&posRefJoy_Mutex);	
		  }

		//Grab yaw reference
		pthread_mutex_lock(&attRefJoy_Mutex);	
		    yawDesired = attRefJoy(2);
	    pthread_mutex_unlock(&attRefJoy_Mutex);

	  	//Calculate position and velocity errors
	  	e_Pos << localPVA_Ref.pos.position.x - localPVA_quadVicon.pos.position.x,
	  			 localPVA_Ref.pos.position.y - localPVA_quadVicon.pos.position.y,
	  			 localPVA_Ref.pos.position.z - localPVA_quadVicon.pos.position.z;
	  	e_Vel << localPVA_Ref.vel.linear.x - localPVAEst_quadVicon.vel.linear.x,
	  			 localPVA_Ref.vel.linear.y - localPVAEst_quadVicon.vel.linear.y,
	  			 localPVA_Ref.vel.linear.z - localPVAEst_quadVicon.vel.linear.z;
	  	
	  	//Get feedforward vector
	  	acc_Ref << localPVA_Ref.acc.linear.x,
	  			   localPVA_Ref.acc.linear.y,
	  			   localPVA_Ref.acc.linear.z;
	  	feedForward = z_w*nominalThrust + acc_Ref;
	  	// feedForward = Add3x1Vec(ScaleMatrix<float, 3, 1>(z_w, nominalThrust), ScaleMatrix<float, 3, 1>(acc_Ref, 1.0/gz));//feedForward = m*gz*z_w + m*ref_dotdot

		//Vehicle attitude
		Matrix<float, 3, 3> Rbw = Quat2rot(IMU_localData_Quat);
		z_b = Rbw*z_w; //z vector of the vehicle in inertial frame

		//Update data in PID, calculate results
		pthread_mutex_lock(&PID_Mutex);
			updateErrorPID(&PID_pos, feedForward, e_Pos, e_Vel, dt);

			//Dont integrate integrator if not in POS_Control mode
			if (localCurrentState != POSITION_JOY_MODE && localCurrentState != POSITION_ROS_MODE){
				resetIntegralErrorPID(&PID_pos);
			}

			//Calculate 3d world desired force for the quadcopter and normalize it
			Fdes = outputPID(PID_pos);
		pthread_mutex_unlock(&PID_Mutex);

		//Desired thrust in body frame
	    pthread_mutex_lock(&ThrustPosControl_Mutex);
			ThrustPosControl = Fdes.dot(z_b);
		pthread_mutex_unlock(&ThrustPosControl_Mutex);

		//Find desired attitude from desired force and yaw angle
		z_bdes = normalizeVec3(Fdes);

		cos_YawHalf = localPVA_Ref.pos.orientation.w;
		sin_YawHalf = localPVA_Ref.pos.orientation.z;

		//cos(2a) = 2cos^2(a) - 1
		//sin(2a) = 2*sin(a)cos(a)
		// x_cdes << 2*pow(cos_YawHalf,2.0) - 1.0, 
		// 		  2*sin_YawHalf*cos_YawHalf, 
		// 		  0;
		x_cdes << 1.0, 
				    0, 
					0;
		y_bdes = normalizeVec3(z_bdes.cross(x_cdes));
		x_bdes = y_bdes.cross(z_bdes);

		//Set desired rotation matrix for attitude
		pthread_mutex_lock(&attRefPosControl_Mutex);
			Rdes_PosControl << x_bdes, y_bdes, z_bdes;
		pthread_mutex_unlock(&attRefPosControl_Mutex);

		// PrintMatrix<float, 3, 3>(Rdes);


	}
	
	printf("PosControl_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}