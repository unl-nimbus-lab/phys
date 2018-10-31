#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <string.h>

#define freq 40
#define maxV 0.3
#define XBOUND_H 1.1
#define XBOUND_L -2.0
#define YBOUND_H 2.4
#define YBOUND_L -2.0
#define BORDER_NUMB 5
#define BORDER_TOP 1.0

#define XBIAS 0.0  //Based on global axes Last:0.4
#define YBIAS 0.0  //-0.3

#define k_ang 1.0

using namespace std;

geometry_msgs::Twist odom;

int state = 0;
bool freeXY = false;
bool flying = false;
bool reStart = false;
bool isDone = true;

float box_x, box_y;
string world, quad_name, quad_frame, box_frame;
pc_asctec_sim::pc_goal_cmd pos_cmd;
pc_asctec_sim::pc_state quad_state;
geometry_msgs::Twist xyCmd;

ros::Publisher traj_pub, border_pub, pos_pub, start_pub, mode_pub, xy_pub, odom_pub, box_pub, debug_pub;
ros::Subscriber traj_sub, joy_sub, state_sub, xy_sub, esen_sub, desSt_sub;

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
	pos_cmd.ax = msg->linear.x;
	pos_cmd.ay = msg->linear.y;
}

void desStCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	pos_cmd.x = msg->linear.x;
	pos_cmd.y = msg->linear.y;
	pos_cmd.vx = msg->angular.x;
	pos_cmd.vy = msg->angular.y;
}

void esenCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(isDone && (state == 2 || state == 3)) {
		freeXY = msg->data;
	}
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(isDone) {
		if(msg->buttons[2] && flying) {
			freeXY = !freeXY;

		}else if(msg->buttons[0] && state == 0) {
			flying = true;

		}else if(msg->buttons[0] && state == 2) {
			flying = false;
		
		}else if(msg->buttons[1] && state == 2) {
			reStart = true;
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

	ros::init(argc, argv, "Esen Experiment");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;

	ros::param::get("~q_name", quad_name);
	ros::param::get("~q_frame", quad_frame);
	ros::param::get("~box_frame", box_frame);
	ros::param::get("~w_frame", world);

	tf::StampedTransform transform;
	tf::TransformListener listener;

	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);
	mode_pub = nh.advertise<std_msgs::Bool>(quad_name + "/xcmd_mode", 10);
	xy_pub = nh.advertise<geometry_msgs::Twist>(quad_name + "/xy_cmds", 10);
	odom_pub = nh.advertise<geometry_msgs::Twist>(quad_name + "/odom", 10);
	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);
	pos_pub = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);
	box_pub = nh.advertise<geometry_msgs::Twist>(quad_name + "/box", 10);
	debug_pub = nh.advertise<geometry_msgs::Twist>(quad_name + "/debug", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);
	state_sub = nh.subscribe(quad_name + "/state", 10, stateCallback);
	xy_sub = nh.subscribe(quad_name + "/accel", 1, xyCallback);
	esen_sub = nh.subscribe(quad_name + "/esen", 10, esenCallback);
	desSt_sub = nh.subscribe(quad_name + "/des_state", 1, desStCallback);

	listener.waitForTransform(world, box_frame, ros::Time(0), ros::Duration(3.0));
	listener.waitForTransform(world, quad_frame, ros::Time(0), ros::Duration(3.0));
	ros::Time past = ros::Time(0);

	ROS_INFO("Running: Esen_exp");

	while(ros::ok()) {
		ros::spinOnce();
		showBorder(outBorder(), BORDER_TOP);

		listener.lookupTransform(world, box_frame, ros::Time(0), transform);
		geometry_msgs::Twist boxy;
		boxy.linear.x = transform.getOrigin().x();
		boxy.linear.y = transform.getOrigin().y();
		box_pub.publish(boxy);

		listener.lookupTransform(world, quad_frame, ros::Time(0), transform);

		double dt = transform.stamp_.toSec() - past.toSec();
		past = transform.stamp_;
		odom.angular.x = (transform.getOrigin().x() - odom.linear.x)/dt;
		odom.angular.y = (transform.getOrigin().y() - odom.linear.y)/dt;

		odom.linear.x = transform.getOrigin().x();
		odom.linear.y = transform.getOrigin().y();
		odom_pub.publish(odom);

		quad_state.x = odom.linear.x;
		quad_state.y = odom.linear.y;

		tf::Quaternion q = transform.getRotation();
		tf::Matrix3x3 m(q);

		double wYaw, wRoll, wPitch;
		m.getRPY(wRoll, wPitch, wYaw);
		geometry_msgs::Twist dbug;
		dbug.angular.x = wRoll;
		dbug.angular.y = wPitch;
		dbug.angular.z = wYaw;
		debug_pub.publish(dbug);

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

				}else if(isDone && reStart) {
					float tTravel = sqrt(pow(0.4 - quad_state.x,2) + pow(-1.5-quad_state.y,2) + pow(quad_state.z-1,2)) / maxV;
					tTravel = limit(tTravel, 15, 1);
					sendTrajectory(tTravel,0.0,0.4,-1.5,1.0,0.0);
					isDone = false;
					reStart = false;

					ROS_INFO("Recentering at 0.4, -1.5, 0.0, time of travel: %f", tTravel);

				}else if(isDone && freeXY) {
					ROS_INFO("Listening to Esen commands...");
					state = 3;
				}
				break;

			case 3:
				//State behavior
				if(isDone) {
					pc_asctec_sim::pc_traj_cmd cmd;
					cmd.x[0] = boxy.linear.x-1;
					cmd.x[1] = boxy.linear.x-1;
					cmd.x[2] = 0.4;

					cmd.y[0] = boxy.linear.y-0.5;
					cmd.y[1] = boxy.linear.y+0.5;
					cmd.y[2] = 1.75;

					cmd.z[0] = 1.0;
					cmd.z[1] = 1.0;
					cmd.z[2] = 1.0;

					cmd.duration[0] = sqrt(pow(quad_state.x-cmd.x[0],2)+pow(quad_state.y-cmd.y[0],2))/maxV;
					cmd.duration[1] = sqrt(pow(cmd.x[0]-cmd.x[1],2)+pow(cmd.y[0]-cmd.y[1],2))/maxV;
					cmd.duration[2] = sqrt(pow(cmd.x[1]-cmd.x[2],2)+pow(cmd.y[1]-cmd.y[2],2))/maxV;

					cmd.points = 3;
					traj_pub.publish(cmd);
					isDone = false;
					state = 4;
				}
				break;
		
			case 4:
					//Stall and do nothing
				break;	
		}
		loop_rate.sleep();
	}
}
