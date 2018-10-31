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

const double PI= 3.141592653589793;

bool KeepGoing = true;
int q; //discrete state
float xpos;
float ypos;
float theta;

float ed; //error in d
float ded;//error in d dot

float v=0.4; //constant for this exercise
float w;

//pole information (-1, 2 imaginary poles at 45 degrees)
float alpha=0.9;//1.4142;
float xi=0.6536;
float beta;

//gain variables
float kp;
float kd;

//inputs
float ue1;
float ue2;
float u1;
float u2;

float r = 3.0; //wall circle
float c=0;//wall curvature
float d=0;//distance to wall
float dd; //distance to wall dot
float dr=1.0;//reference distance
float tt=0; //theta_t
float tp=0; //theta_p

int Sr=50; //sampling rate in hz

float tsim; //simulation time
float t; //program time (tsim normalized to t0=0 in code)
float previous_t;
float delta_t;
bool initialTime = true;

//for trajectory tracking
float arcTime=0;
bool lineTrajectory=true;
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
	std::string fileNames[] = {"wall_following_robot","wall_info"};
	FILE *fp[2]; //array of file pointers, one for each robot.
	//file columns initialization
	for (int i = 0; i < fileQuantity; i++){
		fp[i] = fopen(fileNames[i].c_str(),"w");
		//uncomment if column names are needed
		//fprintf(fp[i],"time(seconds), x, y, theta, dx,dy,dtheta\n");
	}
	
	//ROS SETUP
	ros::init(argc, argv, "wall_following");
	ros::NodeHandle wallFollowingNode;
	ros::Rate loop_rate(Sr);

	//SETUP CLIENT TO GET MODEL STATE
	ros::service::waitForService("/gazebo/get_model_state");
	ros::ServiceClient getModelStateClient =
       wallFollowingNode.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getModelState;
	getModelState.request.model_name = "mobile_base";
	
	//SETUP AND CALL CLIENT SET MODEL STATE
	//Moves the robot to the desired initial pose/orientation
	ros::service::waitForService("/gazebo/set_model_state");
	ros::ServiceClient setModelStateClient = wallFollowingNode.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	
	geometry_msgs::Pose start_pose;
	start_pose.position.x = 0;
	start_pose.position.y = 4;
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
	ros::Publisher pubName = wallFollowingNode.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);


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
	  		//is moved by hand, and works for all models (not just robots)
	  		getModelStateClient.call(getModelState);
	  		tf::Pose pose;
	  		tf::poseMsgToTF(getModelState.response.pose, pose);
	  		
	  		xpos = getModelState.response.pose.position.x;
			ypos = getModelState.response.pose.position.y;
	  		theta = tf::getYaw(pose.getRotation());
			
			// *********** CONTROL LAW CALCULATION ********** //
				

				
			if(xpos>=0 && xpos <r && ypos >0){
				c=0;
				tt=0;
				d=ypos-r;
				q=1;
			} else if(xpos>=r && ypos>=0) {
				c=0;
				tt=-PI/2;
				d=xpos-r;
				q=2;
			} else if(xpos>=0 && ypos <=0) {
				c=1/r;
				//p is the robot position vector p=[xpos,ypos]'
				float ap = atan2(ypos,xpos);
				tt= ap - PI/2;
				d=sqrt(pow(xpos,2)+pow(ypos,2))-r;
				q=3;
			} else if(xpos<0 && ypos <0) {
				c=0;
				tt=PI/2+PI/4;
				d=abs(xpos+ypos+r)/sqrt(2);
				/*
				if((abs(xpos)+1.7)<abs(ypos)) {
					//closer to the y axis
					tt=PI/2;
					d=abs(xpos);
				} else {
					//closer to the x axis
					tt=0;
					d=abs(ypos);
				}
				*/
				q=4;
			} else if(xpos<=0 && ypos >=0){
				c=1/r;
				//p is the robot position vector p=[xpos,ypos]'
				float ap = atan2(ypos,xpos);
				tt= ap - PI/2;
				d=sqrt(pow(xpos,2)+pow(ypos,2))-r;
				q=5;
			}
			
			tp=minAngle(theta-tt);
			dd=v*sin(tp);
			
			ed=dr-d;
			ded=-dd;
			kp=0.5;
			kd=2*sqrt(kp);
			
			w=c*((v*c*cos(tp))/(1-d*c))+((kp*ed+kd*ded)/(v*cos(tp)));

			
			printf("t %2.2f q %1d \n ed % 2.2f c % 2.2f tt % 2.2f tp % 2.2f \n",t,q,ed,c,tt,tp);
			printf("x %2.2f y %2.2f theta %2.2f d % 2.2f w % 2.2f \n \n",xpos,ypos,theta,d,w);

		
			//robot data
			X[0][0]=xpos;
			X[0][1]=ypos;
			X[0][2]=theta;
			X[0][3]=v*cos(theta);
			X[0][4]=v*sin(theta);
			X[0][5]=w;
			
			//robot data
			X[1][0]=d;
			X[1][1]=dd;
			X[1][2]=tt;
			X[1][3]=tp;
			X[1][4]=c;
			X[1][5]=w;

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
		  		std::cout << " Really Stopped!!!";
		  	}
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
  		for (int i = 0; i < 2; i++)
		fclose(fp[i]);

}
