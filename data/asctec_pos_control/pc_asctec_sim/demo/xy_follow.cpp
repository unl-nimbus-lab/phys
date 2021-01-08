#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <string.h>

#define freq 15
#define maxV 0.2
#define XBOUND_H 1.0
#define XBOUND_L -1.0
#define YBOUND_H 1.5
#define YBOUND_L -1.5
#define BORDER_NUMB 5
#define BORDER_TOP 1.0

#define k_ang 1.0

using namespace std;

int state = 0;
string count = "a";
bool freeXY = false;
bool flying = false;
bool reCenter = false;
bool isDone = true;

float robot_x, robot_y, robot_yaw;
string world, ugv, quad_name, quad_frame;
pc_asctec_sim::pc_state quad_state;
geometry_msgs::Twist xyCmd;

ros::Publisher traj_pub, border_pub, pos_pub, start_pub, mode_pub, xy_pub;
ros::Subscriber traj_sub, joy_sub, state_sub, xy_sub;

void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg)
{
	quad_state = *msg;
}

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void xyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	xyCmd = *msg;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(isDone) {
		if(msg->buttons[2] && flying) {
			freeXY = !freeXY;

		}else if(msg->buttons[3] && state == 0) {
			flying = true;

		}else if(msg->buttons[3] && state == 2) {
			flying = false;
		
		}else if(msg->buttons[1] && state == 2) {
			reCenter = true;
		}

		if(freeXY && msg->buttons[4]) {
			xyCmd.angular.x = k_ang * M_PI/2 * msg->axes[2];
			xyCmd.angular.y = k_ang * M_PI/2 * msg->axes[3];

		}else if(freeXY && !msg->buttons[4]) {
			xyCmd.angular.x = 0;
			xyCmd.angular.y = 0;
		}
	}
}

void showBorder(bool outOf, float z)
{
	visualization_msgs::Marker border;
	geometry_msgs::Point corner;

	border.header.frame_id = world;
	border.header.stamp = ros::Time::now();
	border.id = 2;
	border.action = visualization_msgs::Marker::ADD;
	border.type = visualization_msgs::Marker::LINE_LIST;

	if(outOf) {
		border.color.a = 1.0;
		border.color.r = 1.0;	
	}else {
		border.color.a = 1.0;
		border.color.g = 1.0;	
	}
			
	border.scale.x = 0.05;
	border.scale.y = 1.0;
	border.scale.z = 0.5;

		corner.x = XBOUND_L;
		corner.y = YBOUND_L;	
		corner.z = 0;
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_L;	
		corner.z = z;
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_H;	
		corner.z = 0;
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_H;	
		corner.z = z;
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_H;	
		corner.z = 0;
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_H;	
		corner.z = z;
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_L;	
		corner.z = 0;
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_L;	
		corner.z = z;
		border.points.push_back(corner);

	for(int i=0; i<=BORDER_NUMB; i++) {
		corner.x = XBOUND_L;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_H;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = XBOUND_L;
		corner.y = YBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);	
	}
	border_pub.publish(border);
}

void sendTrajectory(float time, float wait, float x, float y, float z, float yaw) 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = x;
	cmd.y[0] = y;
	cmd.z[0] = z;
	cmd.yaw[0] = yaw;
	cmd.wait_time[0] = wait;
	cmd.duration[0] = time;

	cmd.points = 1;
	traj_pub.publish(cmd);
}

void sendLandTrajectory(float time) 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0.0;
	cmd.y[0] = 0.0;
	cmd.z[0] = 1.0;
	cmd.yaw[0] = 0.0;
	cmd.wait_time[0] = 0.5;
	cmd.duration[0] = time;

	cmd.x[1] = 0.0;
	cmd.y[1] = 0.0;
	cmd.z[1] = 0.0;
	cmd.yaw[1] = 0.0;
	cmd.wait_time[1] = 0.0;
	cmd.duration[1] = 3;

	cmd.points = 2;
	traj_pub.publish(cmd);
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

bool outBorder(void)
{
	bool temp = false;
	if(quad_state.x > XBOUND_H || quad_state.x < XBOUND_L || quad_state.y > YBOUND_H || quad_state.y < YBOUND_L) {
		temp = true;
	}
	return temp;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Joy Tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;

	ros::param::get("~q_name", quad_name);
	ros::param::get("~w_frame", world);

	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);
	mode_pub = nh.advertise<std_msgs::Bool>(quad_name + "/xycmd_mode", 10);
	xy_pub = nh.advertise<geometry_msgs::Twist>(quad_name + "/xy_cmds", 10);
	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);
	pos_pub = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);
	state_sub = nh.subscribe(quad_name + "/state", 10, stateCallback);
	xy_sub = nh.subscribe(quad_name + "/mbp_accel", 10, xyCallback);

	ROS_INFO("Running: XY Listener");

	while(ros::ok()) {
		ros::spinOnce();
		showBorder(outBorder(), BORDER_TOP);

		switch(state) {
			case 0:
				//Check exit conditions
				if(flying && isDone) {
					std_msgs::Bool start;
					start.data = true;
					start_pub.publish(start);
					ROS_INFO("Started!!");
					state = 1;			
				}
				break;

			case 1:
				//Check exit conditions
				if(isDone) {
					float tTravel = sqrt(pow(quad_state.x,2) + pow(quad_state.y,2) + pow((1 - quad_state.z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 2.0, 0.0, 0.0, 1.0, 0.0);
					isDone = false;

					ROS_INFO("Taking off to 0.0, 0.0, 1.0, time of travel: %f", tTravel);
					state = 2;
				}
				break;

			case 2:
				//Check exit conditions
				if(isDone && !flying) {
					float tTravel = sqrt(pow(quad_state.x,2) + pow(quad_state.y,2) + pow(quad_state.z,2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendLandTrajectory(tTravel);
					isDone = false;

					ROS_INFO("Landing at 0.0, 0.0, 0.0, time of travel: %f", tTravel);
					state = 0;

				}else if(isDone && reCenter) {
					float tTravel = sqrt(pow(quad_state.x,2) + pow(quad_state.y,2) + pow(quad_state.z-1,2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel,0.0,0.0,0.0,1.0,0.0);
					isDone = false;
					reCenter = false;

					ROS_INFO("Recentering at 0.0, 0.0, 0.0, time of travel: %f", tTravel);

				}else if(isDone && freeXY) {
					std_msgs::Bool free;
					free.data = true;
					mode_pub.publish(free);
					ROS_INFO("Released control of XY axes");
					state = 3;
				}
				break;

			case 3:
				//State behavior
				xy_pub.publish(xyCmd);

				//Check exit conditions
				if(outBorder() || !freeXY) {
					std_msgs::Bool free;
					free.data = false;
					mode_pub.publish(free);

					if(freeXY) {
						freeXY = false;
						ROS_INFO("Border breached!");
					}else {
						ROS_INFO("Regaining control of XY axes");
					}
					state = 2;
				}
				break;	
		}
		loop_rate.sleep();
	}
}
