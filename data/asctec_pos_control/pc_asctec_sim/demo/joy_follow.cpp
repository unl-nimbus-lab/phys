#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <string.h>

#define freq 5
#define maxV 0.2
#define XBOUND_H 1.0
#define XBOUND_L -1.0
#define YBOUND_H 1.5
#define YBOUND_L -1.5

#define YW_GAIN 0.3
#define X_GAIN 0.2
#define Y_GAIN 0.2
#define Z_GAIN 0.1

using namespace std;

int state = 0;
string count = "a";
bool flying = false;
bool tracking = false;
bool isDone = true;

pc_asctec_sim::pc_state quad_state;
float joy_x, joy_y, joy_z, joy_yaw;
string world, quad_name;

ros::Publisher traj_pub, border_pub, pos_pub, start_pub;
ros::Subscriber traj_sub, joy_sub, state_sub;

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
	if(msg->buttons[2] && (state == 2 || state == 3)) {
		tracking = !tracking;

	}else if(msg->buttons[3] && (state == 0 || state == 2)) {
		flying = !flying;

	}else if(msg->buttons[5] && state == 3) {
		joy_yaw = 0;
		joy_x = 0;
		joy_y = 0;
		if(abs(msg->axes[1]) >= 0.02) {
			joy_z = Z_GAIN * msg->axes[1];
		}else {
			joy_z = 0.0;
		}

	}else if(msg->buttons[4] && state == 3) {
                joy_z = 0.0;
                if(msg->axes[4] >= 0.1 || msg->axes[4] <= -0.1) {
                        joy_x = X_GAIN * msg->axes[4];
                }else {
                        joy_x = 0.0;
                }
                if(msg->axes[3] >= 0.1 || msg->axes[3] <= -0.1) {
                        joy_y = Y_GAIN * msg->axes[3];  
                }else {
                        joy_y = 0.0;
                }
                if(msg->axes[0] >= 0.1 || msg->axes[0] <= -0.1) {
                        joy_yaw = YW_GAIN * msg->axes[0];
                }else {
                        joy_yaw = 0.0;
                }
	}else if(state == 3) {
                joy_x = 0.0;
                joy_y = 0.0;
                joy_z = 0.0;
                joy_yaw = 0.0;
	}
}

void show_border(bool outOf)
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
	border.scale.z = 1.0;

	corner.x = XBOUND_L;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

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

void sendPoint(float x, float y, float z, float yaw)
{
	pc_asctec_sim::pc_goal_cmd next;
	next.x = x;
	next.y = y;
	next.z = z;
	next.yaw = yaw;
	pos_pub.publish(next);
}

void sendLandTrajectory(float time) 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0.0;
	cmd.y[0] = 0.0;
	cmd.z[0] = 1.0;
	cmd.yaw[0] = 0.0;
	cmd.wait_time[0] = 1.5;
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

	ros::param::get("~w_frame", world);
	ros::param::get("~q_name", quad_name);

	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);
	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);
	pos_pub = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);

	state_sub = nh.subscribe(quad_name + "/state", 10, stateCallback);
	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);

	ROS_INFO("Running: Joy Tracker");

	while(ros::ok()) {
		ros::spinOnce();
		show_border(outBorder());

		switch(state) {
			case 0:
				/* Basic waiting state, waits for start signal from controller
				*  Input: bool flying
				*/

				//Check exit conditions
				if(flying && isDone) {
					std_msgs::Bool start;
					start.data = true;
					start_pub.publish(start);
					state = 1;
				}
				break;

			case 1:
				/* Fly to Hovering at 0, 0, 1
				*  Input: state change
				*/

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
				/* Hold state, hovers in place
				*  Input: state change + trajectory completion
				*  Output: stop tracking
				*/

				//Check exit conditions
				if(isDone && !flying) {
					float tTravel = sqrt(pow(quad_state.x,2) + pow(quad_state.y,2) + pow(quad_state.z,2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendLandTrajectory(tTravel);
					isDone = false;

					ROS_INFO("Landing at 0.0, 0.0, 0.0, time of travel: %f", tTravel);
					state = 0;

				}else if(tracking && isDone) {
					ROS_INFO("Joy Control Started!");
					state = 3;
				}
				break;

			case 3:
				/* Tracking state, follows joystick
				*  Input: state change
				*/

				if(isDone) {
					if(abs(joy_x) > 0.1 || abs(joy_y) > 0.1 || abs(joy_z) > 0.01 || abs(joy_yaw) > 0.1) {
						float xNew = quad_state.x + joy_x;
						float yNew = quad_state.y + joy_y;
						float zNew = quad_state.z + joy_z;
						float yawNew = quad_state.yaw + joy_yaw;

						xNew = limit(xNew, XBOUND_H, XBOUND_L);
						yNew = limit(yNew, YBOUND_H, YBOUND_L);
						sendPoint(xNew, yNew, zNew, yawNew);
					}
				}

				//Check exit conditions
				if(!tracking) {
					ROS_INFO("Halting Joy Control");		
					state = 2;
				}
				break;		
		}
		loop_rate.sleep();
	}
}
