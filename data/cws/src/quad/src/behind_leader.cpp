#include <iostream>
#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <math.h>
#include <sys/time.h>
#include <stdio.h>

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
float theta;

float xLeader;
float yLeader;
float thetaLeader;

float v; //constant for this exercise
float w;

//pose information
float r; //rho:   distance to goal
float a; //alpha: angle to goal
float b; //beta:  desired pose respect to the robot/goal line

float dr;
float da;
float db;

float delta_x; //goal x coordinate in a world frame located at robot
float delta_y; //goal y coordinate in a world frame located at robot

//inputs
float u_v;
float u_w;

int Sr=50; //sampling rate in hz

float tsim; //simulation time
float t; //program time (tsim normalized to t0=0 in code)
float previous_t;
float delta_t;
bool initialTime = true;

float x_initial[]={-4.5,-1.0,4.0,1.0,-5.0};
float y_initial[]={-5.6,-6.0,-6.0,6.0,5.0};

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
	sprintf(file_1,"follower_state_%i.csv",InitialCondition);
	char file_2[250];
	sprintf(file_2,"follower_i bnput_%i.csv",InitialCondition);

	
	std::string fileNames[] = {file_1,file_2};
	FILE *fp[2]; //array of file pointers, one for each robot.
	//file columns initialization
	for (int i = 0; i < fileQuantity; i++){
		fp[i] = fopen(fileNames[i].c_str(),"w");
		//uncomment if column names are needed
		//fprintf(fp[i],"time(seconds), x, y, theta, dx,dy,dtheta\n");
	}
	
	//ROS SETUP
	ros::init(argc, argv, "behind_leader");
	ros::NodeHandle goToGoalNode;
	ros::Rate loop_rate(Sr);

	//SETUP CLIENT TO GET MODEL STATE Follower
	ros::service::waitForService("/gazebo/get_model_state");
	ros::ServiceClient getModelStateClient =
       goToGoalNode.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getModelState;
	getModelState.request.model_name = "mobile_base";
	
	//SETUP CLIENT TO GET MODEL STATE Leader
	ros::service::waitForService("/gazebo/get_model_state");
	ros::ServiceClient getModelStateClientLeader =
       goToGoalNode.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getModelStateLeader;
	getModelStateLeader.request.model_name = "quadrotor";
	
	//SETUP AND CALL CLIENT SET MODEL STATE
	//Moves the robot to the desired initial pose/orientation
	ros::service::waitForService("/gazebo/set_model_state");
	ros::ServiceClient setModelStateClient = goToGoalNode.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	
	geometry_msgs::Pose start_pose;
	start_pose.position.x = x_initial[InitialCondition];
	start_pose.position.y = y_initial[InitialCondition];
	start_pose.position.z = 0.0;
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
	modelstate.model_name = (std::string) "mobile_base";
	modelstate.reference_frame = (std::string) "world";
	modelstate.pose = start_pose;
	modelstate.twist = start_twist;

	gazebo_msgs::SetModelState setModelState;
	setModelState.request.model_state = modelstate;
	setModelStateClient.call(setModelState);
	
	//COMMAND PUBLISHER TO THE ROBOT
	ros::Publisher pubName = goToGoalNode.advertise<geometry_msgs::Twist>("/tbot_ns/mobile_base/commands/velocity", 1000);


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
	  		theta = tf::getYaw(pose.getRotation());
	  		
	  		//get state from leader
	  		getModelStateClientLeader.call(getModelStateLeader);
	  		tf::Pose poseLeader;
	  		tf::poseMsgToTF(getModelStateLeader.response.pose, poseLeader);
	  		
	  		xLeader = getModelStateLeader.response.pose.position.x;
			yLeader = getModelStateLeader.response.pose.position.y;
	  		thetaLeader = tf::getYaw(poseLeader.getRotation());
			
			// *********** CONTROL LAW CALCULATION ********** //
			r=sqrt(pow(xLeader-xpos,2)+pow(yLeader-ypos,2));
			a=minAngle(atan2(yLeader-ypos,xLeader-xpos)-theta);
			b=minAngle(-theta-a);
			
			float error_delta = 0.1;
			if(r<error_delta && abs(minAngle(a)) < error_delta &&
			 abs(minAngle(b)) < error_delta){
				KeepGoing = false;
			}
			//go to goal can do forward/reverse
			v=kr*r*cos(a);
			w=ka*a+kr*sin(a)*cos(a)*minAngle(a-kb*b)/(a+0.000001);
			
			// ********** print and save data ***********//
			printf("t %2.2f r %2.2f a % 2.2f b % 2.2f\n",t,r,a,b);
			printf("x %2.2f y %2.2f theta %2.2f \n",xpos,ypos,theta);
			printf("v %2.2f w %2.2f \n\n",v,w);

		
			//robot data
			X[0][0]=xpos;
			X[0][1]=ypos;
			X[0][2]=theta;
			X[0][3]=x_initial[InitialCondition];
			X[0][4]=y_initial[InitialCondition];
			X[0][5]=InitialCondition;
			
			//robot data
			X[1][0]=r;
			X[1][1]=a;
			X[1][2]=b;
			X[1][3]=v;
			X[1][4]=w;
			X[1][5]=0;

			//Save data to text files for analysis
			for (int fileIndex=0; fileIndex < fileQuantity; fileIndex++) {
				exportData(fp[fileIndex],t, X[fileIndex]);
				}

	  		if(KeepGoing == true){
	  		geometry_msgs::Twist twistMsg;
			twistMsg.linear.x=v;
			twistMsg.linear.y=0.0;
			twistMsg.linear.z=0.0;
			twistMsg.angular.x=0.0;
			twistMsg.angular.y=0.0;
			twistMsg.angular.z=w;
		  	
		  	pubName.publish(twistMsg);
		  	} else {
		  		std::cout << " Stopped!!! \n";
			  	geometry_msgs::Twist twistMsg;
				twistMsg.linear.x=0.0;
				twistMsg.linear.y=0.0;
				twistMsg.linear.z=0.0;
				twistMsg.angular.x=0.0;
				twistMsg.angular.y=0.0;
				twistMsg.angular.z=0.0;
		  	
		  	pubName.publish(twistMsg);
		  	}
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
  		for (int i = 0; i < 2; i++)
		fclose(fp[i]);

}
