#include <iostream>
#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "hector_uav_msgs/MotorCommand.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <math.h>
#include <sys/time.h>
#include <stdio.h>
#include <vector>
#include <gazebo/common/Events.hh>

using namespace std;

//gain variables
int InitialCondition = 0;
float divider=10.0;
float kr=3.0/divider;
float ka=4.0/divider;
float kb=6.0/divider;

bool KeepGoing = true;
const double PI= 3.141592653589793;

float xpos;
float ypos;
float zpos;
float theta;

float x;
float y;

std::vector<float> W(4);
std::vector<float> T(4);
std::vector<float> F(4);
std::vector<float> V(4);

float Sr=50.0; //sampling rate in hz

float tsim; //simulation time
float t; //program time (tsim normalized to t0=0 in code)
float previous_t;
float delta_t;
bool initialTime = true;

float x_initial[]={-1.0,-1.0,4.0,1.0,-5.0};
float y_initial[]={-1.0,-6.0,-6.0,6.0,5.0};

//for data saving
const int fileQuantity =2;
float X[fileQuantity][6]; //state to be saved to file
void exportData(FILE *fp, float timestamp,  float stats[6])
{
	int x;

	fprintf(fp, "%f,", timestamp);
	for (x = 0; x < 5; x++)
		fprintf(fp, "%f,",stats[x]);
	fprintf(fp, "%f\n", stats[5]);
} 

int sgn(float sgnVal){
	if (sgnVal > 0) return 1;
	if (sgnVal < 0) return -1;
	return 0;
}

//for calculating angle differences and angle wrapping 
double minAngle(double angle){
	return atan2(sin(angle),cos(angle));
}

// ***** MAIN ***** //

int main(int argc, char **argv)
{
	//FILE CONFIGURATION FOR EXPERIMENT SAVING
	char file_1[250];
	sprintf(file_1,"leader_state_%i.csv",InitialCondition);
	char file_2[250];
	sprintf(file_2,"leader_input_%i.csv",InitialCondition);

	
	std::string fileNames[] = {file_1,file_2};
	FILE *fp[2]; //array of file pointers, one for each robot.
	//file columns initialization
	for (int i = 0; i < fileQuantity; i++){
		fp[i] = fopen(fileNames[i].c_str(),"w");
		//uncomment if column names are needed
		//fprintf(fp[i],"time(seconds), x, y, theta, dx,dy,dtheta\n");
	}
	
	//ROS SETUP
	ros::init(argc, argv, "quad_gz");
	ros::NodeHandle quadNode;
	ros::Rate loop_rate(Sr);

	//SETUP CLIENT TO GET MODEL STATE
	ros::service::waitForService("/gazebo/get_model_state");
	ros::ServiceClient getModelStateClient =
       quadNode.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getModelState;
	getModelState.request.model_name = "quadrotor";
	
	//SETUP AND CALL CLIENT SET MODEL STATE
	//Moves the robot to the desired initial pose/orientation
	ros::service::waitForService("/gazebo/set_model_state");
	ros::ServiceClient setModelStateClient = quadNode.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	
	geometry_msgs::Pose start_pose;
	start_pose.position.x = x_initial[InitialCondition];
	start_pose.position.y = y_initial[InitialCondition];
	start_pose.position.z = 2;
	start_pose.orientation.x = 0.0;
	start_pose.orientation.y = 0.0;
	start_pose.orientation.z = 0.0;
	start_pose.orientation.w = 0.0;

	geometry_msgs::Twist start_twist;
	start_twist.linear.x = 0.0;
	start_twist.linear.y = 0.0;
	start_twist.linear.z = 0.0;
	start_twist.angular.x = 0.0;
	start_twist.angular.y = 0.0;
	start_twist.angular.z = 0.0;

	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = (std::string) "quadrotor";
	modelstate.reference_frame = (std::string) "world";
	modelstate.pose = start_pose;
	modelstate.twist = start_twist;

	gazebo_msgs::SetModelState setModelState;
	setModelState.request.model_state = modelstate;
	setModelStateClient.call(setModelState);
	
	//COMMAND PUBLISHER TO THE ROBOT
	ros::Publisher pubQuad = quadNode.advertise<hector_uav_msgs::MotorCommand>("/command/motor", 1000);

	//Command variable
    hector_uav_msgs::MotorCommand motorMsg;

	//LOOP
	while (ros::ok())
  	{
  		//get simulation time
  		tsim = ros::Time::now().toSec();
  
  		
  		if(tsim>0){
  			//setup initial time t0=0
  			if(initialTime == true){
  				previous_t=tsim;
  				t=0;
  				delta_t=0;
  				initialTime = false;
  			} else {
  				delta_t = tsim-previous_t;
  				previous_t=tsim;
	  			t=t+delta_t;
  			}
	  		
	  		//get state from gazebo/vicon
	  		//better than odometry (provides data when robot
	  		//is moved by hand, and works for all models (not just robots))
	  		getModelStateClient.call(getModelState);
	  		tf::Pose pose;
	  		tf::poseMsgToTF(getModelState.response.pose, pose);
	  		
	  		xpos = getModelState.response.pose.position.x;
			ypos = getModelState.response.pose.position.y;
			zpos = getModelState.response.pose.position.z;
	  		theta = tf::getYaw(pose.getRotation());
			
			// *********** CONTROL LAW CALCULATION ********** //
			x = xpos;
			y = ypos;
			

			
			T[0]=1;
			T[1]=2;
			T[2]=1;
			T[3]=2;
			
			V[0]=1;
			V[1]=2;
			V[2]=1;
			V[3]=2;
			
			F[0]=1;
			F[1]=2;
			F[2]=1;
			F[3]=2;

			// ********** print and save data ***********//
			/*
			printf("t %2.2f r %2.2f a % 2.2f b % 2.2f\n",t,r,a,b);
			
			*/

			//robot data
			X[0][0]=xpos;
			X[0][1]=ypos;
			X[0][2]=theta;
			X[0][3]=x_initial[InitialCondition];
			X[0][4]=y_initial[InitialCondition];
			X[0][5]=InitialCondition;
			
			//inputs
			X[1][0]=W[0];
			X[1][1]=W[1];
			X[1][2]=W[2];
			X[1][3]=W[3];
			X[1][4]=0;
			X[1][5]=0;

			//Save data to text files for analysis
			for (int fileIndex=0; fileIndex < fileQuantity; fileIndex++) {
				exportData(fp[fileIndex],t, X[fileIndex]);
				}

	  		if(KeepGoing == true){
				
				W[0]=2;
				W[1]=2;
				W[2]=2;
				W[3]=2;
			
				ros::Time sim_time = ros::Time::now();
				motorMsg.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
				motorMsg.header.frame_id = "/base_link";//"/world";
				motorMsg.force = W;
				//motorMsg.torque = T;
				//motorMsg.voltage = V;
				//motorMsg.frequency = F;
				
				printf("x %2.2f y %2.2f z %2.2f theta %2.2f \n",xpos,ypos,zpos,theta);
				printf("w1 %2.2f w2 %2.2f w3 %2.2f w4 %2.2f \n\n",
				W[0],W[1],W[2],W[3]);
		  		pubQuad.publish(motorMsg);
		  		
		  	} else {
		  		std::cout << " Stopped!!! \n";
		  		//TODO: add safety
		  	}
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
  		for (int i = 0; i < 2; i++)
		fclose(fp[i]);

}
