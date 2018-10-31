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

float neg_grad_x;
float neg_grad_y;
float x;
float y;

float v; //constant for this exercise
float w;

//inputs
float u_v;
float u_w;
float u_x;
float u_y;

int Sr=50; //sampling rate in hz

float tsim; //simulation time
float t; //program time (tsim normalized to t0=0 in code)
float previous_t;
float delta_t;
bool initialTime = true;

float x_initial[]={-5.0,-1.0,4.0,1.0,-5.0};
float y_initial[]={-5.0,-6.0,-6.0,6.0,5.0};

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
	ros::init(argc, argv, "pot_field");
	ros::NodeHandle potFieldNode;
	ros::Rate loop_rate(Sr);

	//SETUP CLIENT TO GET MODEL STATE
	ros::service::waitForService("/gazebo/get_model_state");
	ros::ServiceClient getModelStateClient =
       potFieldNode.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getModelState;
	getModelState.request.model_name = "quadrotor";
	
	//SETUP AND CALL CLIENT SET MODEL STATE
	//Moves the robot to the desired initial pose/orientation
	ros::service::waitForService("/gazebo/set_model_state");
	ros::ServiceClient setModelStateClient = potFieldNode.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	
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
	ros::Publisher pubName = potFieldNode.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


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
			
			// *********** CONTROL LAW CALCULATION ********** //
			x = xpos;
			y = ypos;
			
			//equations from matlab representing the negative
			//of the potential gradient including the goal
			//and objectives
			
			u_x = (2*x)/((101*x)/5 + (202*y)/5 + 2*x*x*y*y + 16*x*y + 4*x*y*y + 8*x*x*y + (141*x*x)/10 + 4*x*x*x + x*x*x*x + (261*y*y)/10 + 8*y*y*y + y*y*y*y + 10201/400) - x + (2*x)/((783*x)/5 + (522*y)/5 + 2*x*x*y*y + 48*x*y + 12*x*y*y + 8*x*x*y + (621*x*x)/10 + 12*x*x*x + x*x*x*x + (421*y*y)/10 + 8*y*y*y + y*y*y*y + 68121/400) + (2*x)/(521*x + (521*y)/5 + 2*x*x*y*y + 40*x*y + 20*x*y*y + 4*x*x*y + (1521*x*x)/10 + 20*x*x*x + x*x*x*x + (561*y*y)/10 + 4*y*y*y + y*y*y*y + 271441/400) + (2*x)/((521*x)/5 + 521*y + 2*x*x*y*y + 40*x*y + 4*x*y*y + 20*x*x*y + (561*x*x)/10 + 4*x*x*x + x*x*x*x + (1521*y*y)/10 + 20*y*y*y + y*y*y*y + 271441/400) + (2*x)/((2564*x)/5 + (2564*y)/5 + 2*x*x*y*y + 128*x*y + 16*x*y*y + 16*x*x*y + (1281*x*x)/10 + 16*x*x*x + x*x*x*x + (1281*y*y)/10 + 16*y*y*y + y*y*y*y + 410881/400) + (2*x)/((2043*x)/5 + 681*y + 2*x*x*y*y + 120*x*y + 12*x*y*y + 20*x*x*y + (1041*x*x)/10 + 12*x*x*x + x*x*x*x + (1681*y*y)/10 + 20*y*y*y + y*y*y*y + 463761/400) + 2/((101*x)/5 + (202*y)/5 + 2*x*x*y*y + 16*x*y + 4*x*y*y + 8*x*x*y + (141*x*x)/10 + 4*x*x*x + x*x*x*x + (261*y*y)/10 + 8*y*y*y + y*y*y*y + 10201/400) + 6/((783*x)/5 + (522*y)/5 + 2*x*x*y*y + 48*x*y + 12*x*y*y + 8*x*x*y + (621*x*x)/10 + 12*x*x*x + x*x*x*x + (421*y*y)/10 + 8*y*y*y + y*y*y*y + 68121/400) + 10/(521*x + (521*y)/5 + 2*x*x*y*y + 40*x*y + 20*x*y*y + 4*x*x*y + (1521*x*x)/10 + 20*x*x*x + x*x*x*x + (561*y*y)/10 + 4*y*y*y + y*y*y*y + 271441/400) + 2/((521*x)/5 + 521*y + 2*x*x*y*y + 40*x*y + 4*x*y*y + 20*x*x*y + (561*x*x)/10 + 4*x*x*x + x*x*x*x + (1521*y*y)/10 + 20*y*y*y + y*y*y*y + 271441/400) + 8/((2564*x)/5 + (2564*y)/5 + 2*x*x*y*y + 128*x*y + 16*x*y*y + 16*x*x*y + (1281*x*x)/10 + 16*x*x*x + x*x*x*x + (1281*y*y)/10 + 16*y*y*y + y*y*y*y + 410881/400) + 6/((2043*x)/5 + 681*y + 2*x*x*y*y + 120*x*y + 12*x*y*y + 20*x*x*y + (1041*x*x)/10 + 12*x*x*x + x*x*x*x + (1681*y*y)/10 + 20*y*y*y + y*y*y*y + 463761/400);


u_y =(2*y)/((101*x)/5 + (202*y)/5 + 2*x*x*y*y + 16*x*y + 4*x*y*y + 8*x*x*y + (141*x*x)/10 + 4*x*x*x + x*x*x*x + (261*y*y)/10 + 8*y*y*y + y*y*y*y + 10201/400) - y + (2*y)/((783*x)/5 + (522*y)/5 + 2*x*x*y*y + 48*x*y + 12*x*y*y + 8*x*x*y + (621*x*x)/10 + 12*x*x*x + x*x*x*x + (421*y*y)/10 + 8*y*y*y + y*y*y*y + 68121/400) + (2*y)/(521*x + (521*y)/5 + 2*x*x*y*y + 40*x*y + 20*x*y*y + 4*x*x*y + (1521*x*x)/10 + 20*x*x*x + x*x*x*x + (561*y*y)/10 + 4*y*y*y + y*y*y*y + 271441/400) + (2*y)/((521*x)/5 + 521*y + 2*x*x*y*y + 40*x*y + 4*x*y*y + 20*x*x*y + (561*x*x)/10 + 4*x*x*x + x*x*x*x + (1521*y*y)/10 + 20*y*y*y + y*y*y*y + 271441/400) + (2*y)/((2564*x)/5 + (2564*y)/5 + 2*x*x*y*y + 128*x*y + 16*x*y*y + 16*x*x*y + (1281*x*x)/10 + 16*x*x*x + x*x*x*x + (1281*y*y)/10 + 16*y*y*y + y*y*y*y + 410881/400) + (2*y)/((2043*x)/5 + 681*y + 2*x*x*y*y + 120*x*y + 12*x*y*y + 20*x*x*y + (1041*x*x)/10 + 12*x*x*x + x*x*x*x + (1681*y*y)/10 + 20*y*y*y + y*y*y*y + 463761/400) + 4/((101*x)/5 + (202*y)/5 + 2*x*x*y*y + 16*x*y + 4*x*y*y + 8*x*x*y + (141*x*x)/10 + 4*x*x*x + x*x*x*x + (261*y*y)/10 + 8*y*y*y + y*y*y*y + 10201/400) + 4/((783*x)/5 + (522*y)/5 + 2*x*x*y*y + 48*x*y + 12*x*y*y + 8*x*x*y + (621*x*x)/10 + 12*x*x*x + x*x*x*x + (421*y*y)/10 + 8*y*y*y + y*y*y*y + 68121/400) + 2/(521*x + (521*y)/5 + 2*x*x*y*y + 40*x*y + 20*x*y*y + 4*x*x*y + (1521*x*x)/10 + 20*x*x*x + x*x*x*x + (561*y*y)/10 + 4*y*y*y + y*y*y*y + 271441/400) + 10/((521*x)/5 + 521*y + 2*x*x*y*y + 40*x*y + 4*x*y*y + 20*x*x*y + (561*x*x)/10 + 4*x*x*x + x*x*x*x + (1521*y*y)/10 + 20*y*y*y + y*y*y*y + 271441/400) + 8/((2564*x)/5 + (2564*y)/5 + 2*x*x*y*y + 128*x*y + 16*x*y*y + 16*x*x*y + (1281*x*x)/10 + 16*x*x*x + x*x*x*x + (1281*y*y)/10 + 16*y*y*y + y*y*y*y + 410881/400) + 10/((2043*x)/5 + 681*y + 2*x*x*y*y + 120*x*y + 12*x*y*y + 20*x*x*y + (1041*x*x)/10 + 12*x*x*x + x*x*x*x + (1681*y*y)/10 + 20*y*y*y + y*y*y*y + 463761/400);

		
 			//scaling to fit quadrotors max/min inputs
 			divider=10;
 			u_x = u_x/divider;
 			u_y = u_y/divider;
			

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
			
			//robot data
			X[1][0]=u_x;
			X[1][1]=u_y;
			X[1][2]=divider; //scale from neg_pot_grad to input
			X[1][3]=0;
			X[1][4]=0;
			X[1][5]=0;

			//Save data to text files for analysis
			for (int fileIndex=0; fileIndex < fileQuantity; fileIndex++) {
				exportData(fp[fileIndex],t, X[fileIndex]);
				}

	  		if(KeepGoing == true){
	  		geometry_msgs::Twist twistMsg;
			twistMsg.linear.x=u_x;//vx
			twistMsg.linear.y=u_y;//vy
			twistMsg.linear.z=2.0;//vz
			twistMsg.angular.x=0.0;
			twistMsg.angular.y=0.0;
			twistMsg.angular.z=0;//w;
			
			printf("x %2.2f y %2.2f theta %2.2f \n",xpos,ypos,theta);
			printf("u_x %2.2f u_y %2.2f \n\n",u_x,u_y);
		  	
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
