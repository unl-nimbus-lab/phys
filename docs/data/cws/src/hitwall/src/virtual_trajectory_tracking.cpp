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
float xpos;
float ypos;
float theta;

float xr=0;
float yr=0;
float tr=0;

float dxr=0;
float dyr=0;
float dtr=0;

float ex;
float ey;
float et;

float e1;
float e2;
float e3;

float vr=0;
float wr=0;
float rr=0;

float v;
float w;

//pole information (-1, 2 imaginary poles at 45 degrees)
float alpha=0.9;//1.4142;
float xi=0.6536;
float beta;

//gain variables
float k1;
float k2;
float k3;

//inputs
float ue1;
float ue2;
float u1;
float u2;

int Sr=50; //sampling rate in hz

float tsim; //simulation time
float t; //program time (tsim normalized to t0=0 in code)
float previous_t;
float delta_t;
bool initialTime = true;

//for trajectory tracking
float arcTime=0;
bool lineTrajectory=true;
const int fileQuantity =3;
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
	std::string fileNames[] = {"RealRobot","VirtualRobot","InputsAndErrors"};
	FILE *fp[4]; //array of file pointers, one for each robot.
	//file columns initialization
	for (int i = 0; i < fileQuantity; i++){
		fp[i] = fopen(fileNames[i].c_str(),"w");
		//uncomment if column names are needed
		//fprintf(fp[i],"time(seconds), x, y, theta, dx,dy,dtheta\n");
	}
	
	//ROS SETUP
	ros::init(argc, argv, "virtual_trajectory_tracking");
	ros::NodeHandle trajectoryTrackingNode;
	ros::Rate loop_rate(Sr);

	//SETUP CLIENT TO GET MODEL STATE
	ros::service::waitForService("/gazebo/get_model_state");
	ros::ServiceClient getModelStateClient =
       trajectoryTrackingNode.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getModelState;
	getModelState.request.model_name = "mobile_base";
	
	//SETUP AND CALL CLIENT SET MODEL STATE
	//Moves the robot to the desired initial pose/orientation
	ros::service::waitForService("/gazebo/set_model_state");
	ros::ServiceClient setModelStateClient = trajectoryTrackingNode.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	
	geometry_msgs::Pose start_pose;
	start_pose.position.x = -1;
	start_pose.position.y = 1;
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
	ros::Publisher pubName = trajectoryTrackingNode.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);


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
				
				
				//reference describes semicircle/lines
				
				if(lineTrajectory==true){
					vr=0.2;
					wr=0;
					if(xr>5 || xr<-5) {
						lineTrajectory=false;
						arcTime=0;
					}
				} else {
					vr=0.2; // m/s
					wr=(2*PI)/60; // rad/s -> (1/20)rps
					arcTime=arcTime+delta_t;
					if(arcTime>=(PI/wr) && xr>=-5 && xr<=5){ //PI=half circle
						lineTrajectory=true;
					}
				} 
					
		
				//in xr,yr,tr coordinates
				dxr=vr*cos(tr);
				dyr=vr*sin(tr);
				dtr=wr;
				//X(k+1)= Ts*dX+ X(k);
				xr=delta_t*dxr + xr;
				yr=delta_t*dyr + yr;
				tr=minAngle(delta_t*dtr + tr);

				ex = xr-xpos;
				ey = yr-ypos;
				et = minAngle(tr-theta);

				e1=cos(theta)*ex+sin(theta)*ey;
				e2=-sin(theta)*ex+cos(theta)*ey;
				e3= et;
		
				//gains following precalculated poles
				beta = (pow(alpha,2)-pow(wr,2))/(pow(vr,2));
				k1=2*xi*sqrt(pow(wr,2)+beta*pow(vr,2));
				k2=beta*abs(vr);
				k3=k1;
				
			int CONTROL_LAW = 0; //0 Linearized, 1 and 2 according to class
			if(CONTROL_LAW==0){
			
				//inputs in error coordinates
				u1=-k1*e1;
				u2=-k2*sgn(vr)*e2-k3*e3;
			
				//input in robot coordinates
				v=vr*cos(e3)-u1;
				w=wr-u2;

			} else if(CONTROL_LAW==1){
				
				v=vr*cos(e3)+k1*e1;
				w=wr+k2*vr*e2+k3*vr*sin(e3);
				
			} else if(CONTROL_LAW==2){
			
				v=vr*cos(e3)+k1*e1;
				w=wr+k2*vr*(sin(e3)/(e3+0.0001))*e2+k3*e3;
			}
			
			printf("t %2.2f \n ex % 2.2f ey % 2.2f et % 2.2f \n e1 % 2.2f e2 % 2.2f e3 % 2.2f \n",t,ex,ey,et,e1,e2,e3);
			printf("k1 %2.2f k2 %2.2f k3 %2.2f \n u1 % 2.2f u2 % 2.2f v % 2.2f w % 2.2f \n \n",k1,k2,k3,u1,u2,v,w);

		
			//real robot data
			X[0][0]=xpos;
			X[0][1]=ypos;
			X[0][2]=theta;
			X[0][3]=v*cos(theta);
			X[0][4]=v*sin(theta);
			X[0][5]=w;
			//virtual robot data
			X[1][0]=xr;
			X[1][1]=yr;
			X[1][2]=tr;
			X[1][3]=dxr;
			X[1][4]=dyr;
			X[1][5]=dtr;
			//Inputs and errors
			X[2][0]=v;
			X[2][1]=w;
			X[2][2]=0;
			X[2][3]=ex;
			X[2][4]=ey;
			X[2][5]=et;
		
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
