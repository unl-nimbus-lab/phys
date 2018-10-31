#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

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

#define sXBOUND_H 0.75 * XBOUND_H
#define sXBOUND_L 0.75 * XBOUND_L
#define sYBOUND_H 0.75 * YBOUND_H
#define sYBOUND_L 0.75 * YBOUND_L

#define BORDER_NUMB 5
#define BORDER_TOP 1.0
#define XBIAS  0.0
#define YBIAS  0.0

#define k_ang 0.9
#define ANG_LIMIT M_PI/8

using namespace std;

int state = 0;
bool isDone = true;

float robot_x, robot_y, robot_yaw;
string world, quad_name, wand_frame;
pc_asctec_sim::pc_state quad_state;
geometry_msgs::Twist xyCmd;

ros::Publisher traj_pub, sborder_pub, border_pub, start_pub, mode_pub, xy_pub;
ros::Subscriber traj_sub, state_sub;

void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg)
{
	quad_state = *msg;
}

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
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

void limitXY(void)
{
	if(quad_state.x > sXBOUND_H) {
		xyCmd.angular.y = limit(xyCmd.angular.y, ANG_LIMIT, 0) + YBIAS;
		if(quad_state.y > sYBOUND_H) {
			xyCmd.angular.x = limit(xyCmd.angular.x, ANG_LIMIT, 0) + XBIAS;
			
		}else if(quad_state.y < sYBOUND_L){
			xyCmd.angular.x = limit(xyCmd.angular.x, 0, -ANG_LIMIT) + XBIAS;
		}

	}else if(quad_state.x < sXBOUND_L) {
		xyCmd.angular.y = limit(xyCmd.angular.y, 0, -ANG_LIMIT) + YBIAS;
		if(quad_state.y > sYBOUND_H) {
			xyCmd.angular.x = limit(xyCmd.angular.x, ANG_LIMIT, 0) + XBIAS;

		}else if(quad_state.y < sYBOUND_L){
			xyCmd.angular.x = limit(xyCmd.angular.x, 0, -ANG_LIMIT) + XBIAS;
		}
	}
}

void showSBorder(bool outOf, float z)
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
		border.color.g = 1.0;
	}else {
		border.color.a = 1.0;
		border.color.g = 1.0;	
	}
			
	border.scale.x = 0.05;
	border.scale.y = 1.0;
	border.scale.z = 0.5;

        /// Soft Border
	corner.x = sXBOUND_L;
	corner.y = sYBOUND_L;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_L;
	corner.y = sYBOUND_L;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = sXBOUND_L;
	corner.y = sYBOUND_H;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_L;
	corner.y = sYBOUND_H;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_H;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_H;	
	corner.z = z;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_L;	
	corner.z = 0;
	border.points.push_back(corner);

	corner.x = sXBOUND_H;
	corner.y = sYBOUND_L;	
	corner.z = z;
	border.points.push_back(corner);

	for(int i=0; i<=BORDER_NUMB; i++) {

		//Soft Border	
		corner.x = sXBOUND_L;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_L;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_L;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_H;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_H;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);

		corner.x = sXBOUND_L;
		corner.y = sYBOUND_L;	
		corner.z = i*(z/BORDER_NUMB);
		border.points.push_back(corner);
	}
	sborder_pub.publish(border);
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

void sendBorderTrajectory()
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = quad_state.x;
	cmd.y[0] = quad_state.y;
	cmd.z[0] = 1.0;
	cmd.duration[0] = 1.0;
	cmd.points = 1;

	if(quad_state.x > XBOUND_H) {
		cmd.x[0] = sXBOUND_H;

	}else if(quad_state.x < XBOUND_L) {
		cmd.x[0] = sXBOUND_L;
	}

	if(quad_state.y > YBOUND_H) {
		cmd.y[0] = sYBOUND_H;

	}else if(quad_state.y < YBOUND_L) {
		cmd.y[0] = sYBOUND_L;
	}
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
	cmd.wait_time[0] = 1.5;
	cmd.duration[0] = tTravel;

	cmd.x[1] = 0.0;
	cmd.y[1] = 0.0;
	cmd.z[1] = 0.0;
	cmd.yaw[1] = 0.0;
	cmd.wait_time[1] = 0.0;
	cmd.duration[1] = 3;

	cmd.points = 2;
	traj_pub.publish(cmd);
	isDone = false;
}

bool outBorder(void)
{
	bool temp = false;
	if(quad_state.x > XBOUND_H || quad_state.x < XBOUND_L || quad_state.y > YBOUND_H || quad_state.y < YBOUND_L) {
		temp = true;
	}
	return temp;
}

bool outSBorder(void)
{
	bool temp = false;
	if(quad_state.x > sXBOUND_H || quad_state.x < sXBOUND_L || quad_state.y > sYBOUND_H || quad_state.y < sYBOUND_L) {
		temp = true;
	}
	return temp;
}

void setHRIBehavior(tf::StampedTransform * transform)
{
	tf::Quaternion q = transform->getRotation();
	tf::Matrix3x3 m(q);

	double wYaw, wRoll, wPitch;
	m.getRPY(wRoll, wPitch, wYaw);

	xyCmd.angular.x = (limit(k_ang*wRoll, ANG_LIMIT, -ANG_LIMIT) + XBIAS) * cos(wYaw) + (-limit(k_ang*wPitch, ANG_LIMIT, -ANG_LIMIT) + YBIAS) * sin(wYaw);
	xyCmd.angular.y = (-limit(k_ang*wRoll, ANG_LIMIT, -ANG_LIMIT) + XBIAS) * sin(wYaw) + (-limit(k_ang*wPitch, ANG_LIMIT, -ANG_LIMIT) + YBIAS) * cos(wYaw);
}

bool isWandDown(tf::StampedTransform * transform)
{
	if(transform->getOrigin().z() < 0.1) {
		return true;
	}
	return false;
}

bool isWandUp(tf::StampedTransform * transform)
{
	if(transform->getOrigin().z() > 1.7) {
		return true;
	}
	return false;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Wand Tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;

	ros::param::get("~q_name", quad_name);
	ros::param::get("~wand_frame", wand_frame);
	ros::param::get("~w_frame", world);

	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);
	mode_pub = nh.advertise<std_msgs::Bool>(quad_name + "/xycmd_mode", 10);
	xy_pub = nh.advertise<geometry_msgs::Twist>(quad_name + "/xy_cmds", 10);
	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);
	sborder_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/soft_border", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	state_sub = nh.subscribe(quad_name + "/state", 10, stateCallback);

	tf::TransformListener listener;	
	tf::StampedTransform transform;
	listener.waitForTransform(world, wand_frame, ros::Time(0), ros::Duration(3.0));

	ROS_INFO("Running: Wand Listener");

	while(ros::ok()) {
		ros::spinOnce();
		showBorder(outBorder(), BORDER_TOP);
		showSBorder(outSBorder(), BORDER_TOP);

		listener.lookupTransform(world, wand_frame, ros::Time(0), transform);

		switch(state) {
			case 0:
				//Check exit conditions
				if(isWandUp(&transform) && isDone) {
					std_msgs::Bool start;
					start.data = true;
					start_pub.publish(start);
					state = 1;			
				}
				break;

			case 1:
				//Check exit conditions
				if(isDone) {
					sendTrajectory(0.0, 0.0, 1.0, 0.0, 1.0);

					ROS_INFO("Taking off to 0.0, 0.0, 1.0");
					state = 2;
				}
				break;

			case 2:
				if(isDone) {
					std_msgs::Bool free;
					free.data = true;
					mode_pub.publish(free);
					state = 3;
				}
				break;

			case 3:
				//Check exit conditions
				if(isWandDown(&transform) && isDone) {
					ROS_INFO("Landing...");
					std_msgs::Bool free;
					free.data = false;
					mode_pub.publish(free);

					sendLandTrajectory();
					state = 0;
				}

				if(outBorder() && isDone) {
					ROS_INFO("Border breached!");
					std_msgs::Bool free;
					free.data = false;
					mode_pub.publish(free);
					ros::Duration(0.5).sleep();
					state = 4;
				}

				//State behavior
				setHRIBehavior(&transform);
				limitXY();
				xy_pub.publish(xyCmd);

				break;	
		
			case 4:
				if(isDone) {
					sendBorderTrajectory();
					state = 5;
				}
				break;
			case 5:
				//Check exit conditions
				if(isWandDown(&transform) && isDone) {
					ROS_INFO("Landing...");
					std_msgs::Bool free;
					free.data = false;
					mode_pub.publish(free);

					sendLandTrajectory();
					state = 0;
				}

				if(!outBorder() && isDone) {
					std_msgs::Bool free;
					free.data = true;
					mode_pub.publish(free);
					state = 3;
				}

		}
		loop_rate.sleep();
	}
}
