

#include "threads/Ros_threads.h"


Matrix<float, 3, 1> RPY_Vicon;
Matrix<float, 4, 1> Quat_vicon;
Matrix<float, 4, 1> IMU_localData_QuatViconYaw, IMU_localData_QuatNoYaw;
Matrix<float, 4, 1> Vicon_YawQuat;
int ButtonX = 0;
int ButtonY = 0;
int ButtonA = 0;
int ButtonB = 0;
int ButtonStart = 0;
double SamplingTime = 1.0/20.0; //20Hz
double yawRefPosMode = 0;

void handle_client_pva_msg(const qcontrol_defs::PVA& msg){

	int localCurrentState;
	qcontrol_defs::PVA localPVA_quadVicon;

	pthread_mutex_lock(&PVA_Vicon_Mutex);
		localPVA_quadVicon = PVA_quadVicon;
	pthread_mutex_unlock(&PVA_Vicon_Mutex);

	pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
	pthread_mutex_unlock(&stateMachine_Mutex);

	if(localCurrentState == POSITION_ROS_MODE){
		// pthread_mutex_lock(&ThrustJoy_Mutex);
		// 	ThrustJoy = msg.axes[1] * maxThrust_AttMode;
		// pthread_mutex_unlock(&ThrustJoy_Mutex);
		pthread_mutex_lock(&posRefClient_Mutex);	
			PVA_RefClient.pos.position.x = msg.pos.position.x; //20hz
			PVA_RefClient.pos.position.y = msg.pos.position.y;
			PVA_RefClient.pos.position.z = msg.pos.position.z;
			PVA_RefClient.vel.linear.x = msg.vel.linear.x;
			PVA_RefClient.vel.linear.y = msg.vel.linear.y;
			PVA_RefClient.vel.linear.z = msg.vel.linear.z;
			PVA_RefJoy.acc.linear.x = msg.acc.linear.x;
			PVA_RefJoy.acc.linear.y = msg.acc.linear.y;
			PVA_RefJoy.acc.linear.z = msg.acc.linear.z;
	  	pthread_mutex_unlock(&posRefClient_Mutex);	
	}
	// else{
	// 	pthread_mutex_lock(&posRefClient_Mutex);	
	// 		PVA_RefClient.pos.position.x = localPVA_quadVicon.pos.position.x;
	// 		PVA_RefClient.pos.position.y = localPVA_quadVicon.pos.position.y;
	// 		PVA_RefClient.pos.position.z = localPVA_quadVicon.pos.position.z;
	// 		PVA_RefClient.vel.linear.x = 0;
	// 		PVA_RefClient.vel.linear.y = 0;
	// 		PVA_RefClient.vel.linear.z = 0;
	// 	pthread_mutex_unlock(&posRefClient_Mutex);	
	// }
}


void handle_mp_joy_msg(const sensor_msgs::Joy& msg){
	float yaw_ctr_pos, yaw_ctr_neg;
	int localCurrentState, localYawSource;
	Matrix<float, 3, 1> IMU_localData_RPY, IMU_localData_RPY_ViconYaw;
	qcontrol_defs::PVA localPVA_quadVicon;

	//Grab attitude estimation
	pthread_mutex_lock(&IMU_Mutex);
		IMU_localData_RPY = IMU_Data_RPY;
	pthread_mutex_unlock(&IMU_Mutex);

	pthread_mutex_lock(&PVA_Vicon_Mutex);
		IMU_localData_RPY_ViconYaw = IMU_Data_RPY_ViconYaw;
		localPVA_quadVicon = PVA_quadVicon;
	pthread_mutex_unlock(&PVA_Vicon_Mutex);

	pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
	pthread_mutex_unlock(&stateMachine_Mutex);

	pthread_mutex_lock(&YawSource_Mutex);
		localYawSource = YawSource;
	pthread_mutex_unlock(&YawSource_Mutex);

	yaw_ctr_pos = msg.axes[2]; //Command to increase yaw
	yaw_ctr_neg = msg.axes[5]; //Command to decrease yaw


	//If in attitude mode
	if(localCurrentState == MOTOR_MODE){ 	//If in motor mode
		//Set thrust
		pthread_mutex_lock(&ThrustJoy_Mutex);
			ThrustJoy = msg.axes[1] * maxThrust_MotorMode;
		pthread_mutex_unlock(&ThrustJoy_Mutex);

		//Set attitude with zero error
		pthread_mutex_lock(&attRefJoy_Mutex);	
			attRefJoy(0) = IMU_localData_RPY(0); //Set ref to actual IMU value
			attRefJoy(1) = IMU_localData_RPY(1); //Set ref to actual IMU value
			
			//Set yaw reference to measured yaw
			if(localYawSource == _IMU){
				attRefJoy(2) = IMU_localData_RPY(2);
			}
			else if (localYawSource == _VICON){
				attRefJoy(2) = IMU_localData_RPY_ViconYaw(2);
			}

			angVelRefJoy(0) = 0;
			angVelRefJoy(1) = 0;
			angVelRefJoy(2) = 0;
		pthread_mutex_unlock(&attRefJoy_Mutex);
	}
	
	if(localCurrentState == ATTITUDE_MODE){
		//Set thrust
		pthread_mutex_lock(&ThrustJoy_Mutex);
			ThrustJoy = msg.axes[1] * maxThrust_AttMode;
		pthread_mutex_unlock(&ThrustJoy_Mutex);

		//Set references
		pthread_mutex_lock(&attRefJoy_Mutex);	
			angVelRefJoy(0) = 0;
			angVelRefJoy(1) = 0;
			angVelRefJoy(2) = 0;
			attRefJoy(0) = -msg.axes[3]*PI/6; //roll
			attRefJoy(1) = msg.axes[4]*PI/6; //pitch

			//Set yaw to measured yaw if quad isnt flying
			if (msg.axes[1] <= 0) {
				if(localYawSource == _IMU){
					attRefJoy(2) = IMU_localData_RPY(2);
				}
				else if (localYawSource == _VICON){
					attRefJoy(2) = IMU_localData_RPY_ViconYaw(2);
				}
		    }
		    else{ //If quad is flying, increment yaw
		    	if (yaw_ctr_neg < 0) {
					attRefJoy(2) -= yaw_Inc;
				}								//yaw
				if (yaw_ctr_pos < 0) {
					attRefJoy(2) += yaw_Inc;
				}
		    }
		pthread_mutex_unlock(&attRefJoy_Mutex);
	}
	if(localCurrentState == POSITION_JOY_MODE){
    	if (yaw_ctr_neg < 0) {
			yawRefPosMode -= yaw_Inc;
		}								//yaw
		if (yaw_ctr_pos < 0) {
			yawRefPosMode += yaw_Inc;
		}
		pthread_mutex_lock(&ThrustJoy_Mutex);
			ThrustJoy = msg.axes[1] * maxThrust_AttMode;
		pthread_mutex_unlock(&ThrustJoy_Mutex);
		pthread_mutex_lock(&posRefJoy_Mutex);	
			//Integrate current position
			PVA_RefJoy.pos.position.x += msg.axes[4]*maxVel_PosMode*SamplingTime; //20hz
			PVA_RefJoy.pos.position.y += msg.axes[3]*maxVel_PosMode*SamplingTime;
			PVA_RefJoy.pos.position.z += (msg.buttons[5]-msg.buttons[4])*maxVel_PosMode*SamplingTime;
			PVA_RefJoy.pos.orientation.w = cos(yawRefPosMode/2);
			PVA_RefJoy.pos.orientation.x = 0;
			PVA_RefJoy.pos.orientation.y = 0;
			PVA_RefJoy.pos.orientation.z = sin(yawRefPosMode/2);
			PVA_RefJoy.vel.linear.x = msg.axes[4]*maxVel_PosMode;
			PVA_RefJoy.vel.linear.y = msg.axes[3]*maxVel_PosMode;
			PVA_RefJoy.vel.linear.z = (msg.buttons[5]-msg.buttons[4])*maxVel_PosMode;
			PVA_RefJoy.acc.linear.x = 0;
			PVA_RefJoy.acc.linear.y = 0;
			PVA_RefJoy.acc.linear.z = 0;
			//TODO: add yaw reference here in quaternion
	  	pthread_mutex_unlock(&posRefJoy_Mutex);	
	}
	else{ //If not in position mode, set position references to read values
		yawRefPosMode = IMU_localData_RPY_ViconYaw(2);
		pthread_mutex_lock(&posRefJoy_Mutex);
			PVA_RefJoy.pos.position.x = localPVA_quadVicon.pos.position.x;
			PVA_RefJoy.pos.position.y = localPVA_quadVicon.pos.position.y;
			PVA_RefJoy.pos.position.z = localPVA_quadVicon.pos.position.z;
			PVA_RefJoy.vel.linear.x = 0;
			PVA_RefJoy.vel.linear.y = 0;
			PVA_RefJoy.vel.linear.z = 0;
			PVA_RefJoy.acc.linear.x = 0;
			PVA_RefJoy.acc.linear.y = 0;
			PVA_RefJoy.acc.linear.z = 0;
		pthread_mutex_unlock(&posRefJoy_Mutex);	
	}

	if(localCurrentState != POSITION_ROS_MODE){
		pthread_mutex_lock(&posRefJoy_Mutex);	
			//Reference = current position
			PVA_RefClient.pos.position.x = localPVA_quadVicon.pos.position.x;
			PVA_RefClient.pos.position.y = localPVA_quadVicon.pos.position.y;
			PVA_RefClient.pos.position.z = localPVA_quadVicon.pos.position.z;
			PVA_RefClient.vel.linear.x = 0;
			PVA_RefClient.vel.linear.y = 0;
			PVA_RefClient.vel.linear.z = 0;
	  	pthread_mutex_unlock(&posRefJoy_Mutex);	
	}

	if((localCurrentState == INITIALIZING) || 
	   (localCurrentState == INITIALIZED) ||
	   (localCurrentState == TERMINATE)) { //All not flying modes: set everything to zero
		pthread_mutex_lock(&ThrustJoy_Mutex);
			ThrustJoy = 0;
		pthread_mutex_unlock(&ThrustJoy_Mutex);

		//Set attitude with zero error
		pthread_mutex_lock(&attRefJoy_Mutex);	
			attRefJoy(0) = IMU_localData_RPY(0); //Set ref to actual IMU value
			attRefJoy(1) = IMU_localData_RPY(1); //Set ref to actual IMU value
			
			//Set yaw to measured yaw if quad isnt flying
			if (msg.axes[1] <= 0) {
				if(localYawSource == _IMU){
					attRefJoy(2) = IMU_localData_RPY(2);
				}
				else if (localYawSource == _VICON){
					attRefJoy(2) = IMU_localData_RPY_ViconYaw(2);
				}
		    }
			angVelRefJoy(0) = 0;
			angVelRefJoy(1) = 0;
			angVelRefJoy(2) = 0;
		pthread_mutex_unlock(&attRefJoy_Mutex);
	}

	// printf("Angvel: %f %f %f", angVelRefJoy(0),angVelRefJoy(1), angVelRefJoy(2));
	
	//Compare joystick buttons with previously read (check if state changed)
	if (msg.buttons[0] && !ButtonA){
		SetEvent(e_buttonA);
		//printf("ButtonA Pushed!\n");
	}
	if (msg.buttons[1] && !ButtonB){
		SetEvent(e_buttonB);
		//printf("ButtonB Pushed!\n");
	}
	if (msg.buttons[2] && !ButtonX){
		SetEvent(e_buttonX);
		//printf("ButtonX Pushed!\n");
	}
	if (msg.buttons[3] && !ButtonY){
		SetEvent(e_buttonY);
		//printf("ButtonY Pushed!\n");
	}
	if(msg.buttons[7] && !ButtonStart){
		SetEvent(e_SwitchYawSource);
	}
	// if (msg.buttons[4] && !ButtonLB){
	// 	SetEvent(e_ButtonLB);
	// 	//printf("ButtonY Pushed!\n");
	// }
	// if (msg.buttons[5] && !ButtonRB){
	// 	SetEvent(e_ButtonRB);
	// 	//printf("ButtonY Pushed!\n");
	// }
	ButtonA = msg.buttons[0];
	ButtonB = msg.buttons[1];
	ButtonX = msg.buttons[2];
	ButtonY = msg.buttons[3];
	ButtonStart = msg.buttons[7];
	// ButtonLB = msg.buttons[4];
	// ButtonRB = msg.buttons[5];

}

void handle_Vicon(const geometry_msgs::TransformStamped& msg){

 	pthread_mutex_lock(&PVA_Vicon_Mutex);	
		PVA_quadVicon.pos.position.x = msg.transform.translation.x;
		PVA_quadVicon.pos.position.y = msg.transform.translation.y;
		PVA_quadVicon.pos.position.z = msg.transform.translation.z;

		PVA_quadVicon.t = msg.header.stamp;

		PVA_quadVicon.pos.orientation.w = msg.transform.rotation.w;
		PVA_quadVicon.pos.orientation.x = msg.transform.rotation.x;
		PVA_quadVicon.pos.orientation.y = msg.transform.rotation.y;
		PVA_quadVicon.pos.orientation.z = msg.transform.rotation.z;
  	pthread_mutex_unlock(&PVA_Vicon_Mutex);

  	//Get yaw from vicon measurement and include it into measured quaternion
  	Quat_vicon << msg.transform.rotation.w,
  				  msg.transform.rotation.x,
  				  msg.transform.rotation.y,
  				  msg.transform.rotation.z;

  	RPY_Vicon = Quat2RPY(Quat_vicon);

	Vicon_YawQuat(0) = cos(RPY_Vicon(2)/2);
	Vicon_YawQuat(1) = 0;
	Vicon_YawQuat(2) = 0;
	Vicon_YawQuat(3) = sin(RPY_Vicon(2)/2);

	pthread_mutex_lock(&IMU_Mutex);
		IMU_localData_QuatNoYaw = IMU_Data_QuatNoYaw;
	pthread_mutex_unlock(&IMU_Mutex);

	IMU_localData_QuatViconYaw = QuaternionProduct(Vicon_YawQuat, IMU_localData_QuatNoYaw);

	pthread_mutex_lock(&PVA_Vicon_Mutex);
		IMU_Data_Quat_ViconYaw = IMU_localData_QuatViconYaw;
		IMU_Data_RPY_ViconYaw = Quat2RPY(IMU_localData_QuatViconYaw);
	pthread_mutex_unlock(&PVA_Vicon_Mutex);
  	// kalman_v << kalman_state(3,0) << "," << kalman_state(4,0) << "," << kalman_state(5,0) << "\n";
  	// vicon_p << z(0,0) << "," << z(1,0) << "," << z(2,0) << "\n";

}