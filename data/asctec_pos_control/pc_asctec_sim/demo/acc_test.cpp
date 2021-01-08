#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <string.h>

#define freq 10
#define maxV 0.2
#define tTest 1.0
#define tests 1

#define XBIAS  -0.15
#define YBIAS  0.3

#define k_ang 2.5
#define ANG_LIMIT M_PI/6
#define incA M_PI/16

using namespace std;

int state = 1;
bool isDone = true;
bool isTiming = false;
bool reset = false;

int testN = 1;
float testA = -M_PI/10;

int samples = 0;
double avg_acc = 0;

string quad_name;
pc_asctec_sim::pc_state quad_state;
geometry_msgs::Twist xCmd;

ros::Publisher traj_pub, sborder_pub, border_pub, start_pub, xmode_pub, xy_pub;
ros::Subscriber traj_sub, state_sub, joy_sub;

void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg)
{
	quad_state = *msg;
}

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0]) {
		reset = !reset;
	}
}

void timerCallback(const ros::TimerEvent&) 
{
	isTiming = false;
}

float limit(float value, float upper, float lower)
{
	float temp = value;
	if(value > upper) {
		temp = upper;

	}else if(value < lower) {
		temp = lower;
	}

	return temp;
}

void sendXaxis(float x) {
	xCmd.angular.x = limit(x,ANG_LIMIT,-ANG_LIMIT);
	xy_pub.publish(xCmd);
}

void setXmode(bool val) {
	std_msgs::Bool X;
	X.data = val;
	xmode_pub.publish(X);
}

void sendBraking()
{
	float tTravel = sqrt(pow(0 - quad_state.x,2) + pow(1.25 - quad_state.y,2) + pow((1 - quad_state.z),2)) / maxV;
	tTravel = limit(tTravel, 10, 1);

	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.y[0] = 1.25;
	cmd.z[0] = 1.0;
	cmd.wait_time[0] = 2.0;
	cmd.duration[0] = tTravel;
	cmd.points = 1;

	traj_pub.publish(cmd);
	isDone = false;
}

void sendTrajectory(float x, float y, float z, float yaw, float wait) 
{
	float tTravel = sqrt(pow(x - quad_state.x,2) + pow(y - quad_state.y,2) + pow((z - quad_state.z),2)) / maxV;
	tTravel = limit(tTravel, 10, 1);

	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = x;
	cmd.y[0] = y;
	cmd.z[0] = z;
	cmd.yaw[0] = yaw;
	cmd.wait_time[0] = wait;
	cmd.duration[0] = tTravel;

	cmd.points = 1;
	traj_pub.publish(cmd);
	isDone = false;
}

void sendLandTrajectory() 
{
	float tTravel = sqrt(pow(quad_state.x,2) + pow(quad_state.y,2) + pow((1 - quad_state.z),2)) / maxV;
	tTravel = limit(tTravel, 10, 1);

	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0.0;
	cmd.y[0] = 0.0;
	cmd.z[0] = 1.0;
	cmd.yaw[0] = 0.0;
	cmd.wait_time[0] = 0.5;
	cmd.duration[0] = tTravel;

	cmd.x[1] = 0.0;
	cmd.y[1] = 0.0;
	cmd.z[1] = 0.0;
	cmd.yaw[1] = 0.0;
	cmd.wait_time[1] = 0.0;
	cmd.duration[1] = 3;

	cmd.points = 2;
	traj_pub.publish(cmd);

	ROS_INFO("Landing...");
	isDone = false;
	reset = false;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Acc Tester");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;
	ros::Timer timer = nh.createTimer(ros::Duration(tTest), timerCallback);
	timer.setPeriod(ros::Duration(tTest), false);
	timer.stop();

	ros::param::get("~q_name", quad_name);

	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);
	xmode_pub = nh.advertise<std_msgs::Bool>(quad_name + "/xcmd_mode", 10);
	xy_pub = nh.advertise<geometry_msgs::Twist>(quad_name + "/xy_cmds", 10);
	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	state_sub = nh.subscribe(quad_name + "/state", 10, stateCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);

	tf::TransformListener listener;	
	tf::StampedTransform transform;

	ROS_INFO("Running: Acc tester");

	while(ros::ok()) {
		ros::spinOnce();

		switch(state) {
			case 1:
				//Gamepad start, and move to 0 0 1
				if(reset && isDone) {
					std_msgs::Bool start;
					start.data = true;
					start_pub.publish(start);

					sendTrajectory(0.0, 0.0, 1.0, 0.0, 1.0);
					ROS_INFO("Taking off to 0.0, 0.0, 1.0");
					state = 2;		
				}
				break;

			case 2:
				//Once completed, move to 0 -1.25 1
				if(isDone) {
					if(reset) {
						sendTrajectory(0,-1.25,1,0,0);
						state = 3;

					}else if(!reset) {
						sendLandTrajectory();
						state = 1;
					}
				}
				break;

			case 3:
				//Once completed, free y axis control and start timer
				if(isDone) {
					if(reset) {
						ROS_INFO("Sending test angle: %f", testA);
						setXmode(true);
						timer.setPeriod(ros::Duration(tTest), true);
						timer.start();
						isTiming = true;
						state = 4;
					}else {
						sendLandTrajectory();
						state = 1;
					}
				}
				break;
		
			case 4:
				//Send test angle until timer expires
				if(isTiming) {
					sendXaxis(testA);
					avg_acc += double(quad_state.ay);
					samples++;

				}else {
					avg_acc = avg_acc / double(samples);
					ROS_INFO("Test angle stopped, samples: %i, acc: %f", samples, avg_acc);
					avg_acc = 0;
					samples = 0;
					setXmode(false);
					timer.stop();
					state = 5;
				}
				break;	

			case 5:
				//Send braking command to slow down
				if(isDone) {
					if(reset) {
						sendBraking();
						state = 6;

					}else if(!reset) {
						sendLandTrajectory();
						state = 1;
					}	
				}
				break;	
				//Check exit conditions

			case 6:
				//Return to 0 -1 1 and rerun test OR
				//if test runs are completed, land
				if(isDone) {
					if(!reset) {
						sendLandTrajectory();
						state = 1;

					}else if(testN == tests) {
						ROS_INFO("All tests completed");
						sendLandTrajectory();
						state = 1;

					}else {
						ROS_INFO("Test %i completed", testN);
						sendTrajectory(0,-1.25,1,0,3);
						testA -= incA;
						testN++;
						state = 3;
					}
				}
				break;
		}
		loop_rate.sleep();
	}
}
