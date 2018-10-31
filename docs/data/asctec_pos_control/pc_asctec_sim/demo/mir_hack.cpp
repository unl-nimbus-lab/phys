#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pc_asctec_sim/pc_traj_cmd.h>

#include <sensor_msgs/Joy.h>

#include <math.h>
#include <string.h>

#define freq 5
#define maxV 0.2
#define maxV_T 0.5
#define minT 1
#define TRIGGER_RANGE 1.0
#define F_HEIGHT 1.25
#define L_HEIGHT 0.75
#define REST 0.5
#define MOUNT 0.45

using namespace std;

int state = -1;
string count = "a";
bool flying = false;
bool over_box = false;
bool lower_box = false;
bool isDone = true;
bool starting = false;

float quad_x, quad_y, quad_z, quad_yaw;
float box_x, box_y;
float ugv_x, ugv_y;

string world, quad_name, quad_frame, box_frame, ugv_frame;

ros::Publisher traj_pub, start_pub;
ros::Subscriber traj_sub, joy_sub;

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0] && state == 0) {
		flying = true;

	}else if(msg->buttons[0] && state == 1) {
		flying = false;

	}else if(msg->buttons[0] && state == 4) {
		flying = false;

	}else if(msg->buttons[1] && state == 1) {
		over_box = true;

	}else if(msg->buttons[1] && state == 2) {
		over_box = false;

	}else if(msg->buttons[2] && state == 2) {
		lower_box = true;

	}else if(msg->buttons[2] && state == 3) {
		lower_box = false;

	}else if(msg->buttons[3] && state == -1) {
		starting = true;
	}
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
	cmd.wait_time[0] = 1.5;
	cmd.duration[0] = time;

	cmd.x[1] = 0.0;
	cmd.y[1] = 0.0;
	cmd.z[1] = MOUNT;
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


int main(int argc, char** argv) {

	ros::init(argc, argv, "Joy Tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;
   
	tf::StampedTransform transform;
	tf::TransformListener listener;

	ros::param::get("~w_frame", world);
	ros::param::get("~q_name", quad_name);
	ros::param::get("~ugv_frame", ugv_frame);
	ros::param::get("~q_frame", quad_frame);
	ros::param::get("~box_frame", box_frame);

	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);
	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);
	
	listener.waitForTransform(world, quad_frame, ros::Time(0), ros::Duration(3.0));
	listener.waitForTransform(world, box_frame, ros::Time(0), ros::Duration(3.0));
	listener.waitForTransform(world, ugv_frame, ros::Time(0), ros::Duration(3.0));


	listener.lookupTransform(world, box_frame, ros::Time(0), transform);
	box_x = transform.getOrigin().x();
	box_y = transform.getOrigin().y();
	
	ROS_INFO("Running: Mirror Hack");

	while(ros::ok()) {
		ros::spinOnce();
		
		listener.lookupTransform(world, quad_frame, ros::Time(0), transform);
		quad_x = transform.getOrigin().x();
		quad_y = transform.getOrigin().y();
		quad_z = transform.getOrigin().z();

		listener.lookupTransform(world, ugv_frame, ros::Time(0), transform);
		ugv_x = transform.getOrigin().x();
		ugv_y = transform.getOrigin().y();
		
		switch(state) {
			case -1:
				// Transition fwd - Waits for joy signal "starting" = true, begins holding quad in place
				if(isDone && starting) {
					ROS_INFO("Floating in place...");
					std_msgs::Bool start;
					start.data = true;
					start_pub.publish(start);
					state = 0;
				}
				break;
			case 0:
				// Transition fwd - Waits for joy signal "flying" = true, moves quad + mirror to start height
				if(isDone && flying) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, minT);
					sendTrajectory(tTravel, 0.0, 0.0, 0.0, F_HEIGHT, 0.0);

					ROS_INFO("Taking off to 0.0, 0.0, %f, time of travel: %f s", F_HEIGHT, tTravel);
					state = 1;
				}
				break;

			case 1:
				// Transition fwd - Waits for joy signal "over_box" = true, moves quad + mirror to box
				if(isDone && over_box) {
					listener.lookupTransform(world, box_frame, ros::Time(0), transform);
					box_x = transform.getOrigin().x();
					box_y = transform.getOrigin().y();

					float tTravel = sqrt(pow(box_x-quad_x,2) + pow(box_y-quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, minT);
					sendTrajectory(tTravel, 0.0, box_x, box_y, F_HEIGHT, 0.0);

					ROS_INFO("Hovering over target, time of travel: %f s", tTravel);
					state = 2;

				// Transition bwd - Waits for joy signal "flying" = false, moves quad + mirror to resting height
				}else if(isDone && !flying) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((REST - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, minT);
					sendTrajectory(tTravel, 0.0, 0.0, 0.0, REST, 0.0);

					ROS_INFO("Returning to rest, time of travel: %f s", tTravel);
					state = 0;
				}
				break;

			case 2:
				// Transition fwd - Waits for joy signal "lower_box" = true, moves quad + mirror down to hide box
				if(isDone && lower_box) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((L_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, minT);
					sendTrajectory(tTravel, 0.0, box_x, box_y, L_HEIGHT, 0.0);

					ROS_INFO("Setting up mirror demo...");
					state = 3;

				// Transition bwd - Waits for joy signal "over_box" = false, moves quad + mirror back to center
				}else if(isDone && !over_box) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, minT);
					sendTrajectory(tTravel, 0.0, 0, 0, F_HEIGHT, 0.0);
				
					ROS_INFO("Returning to center...");
					state = 1;
				}
				break;

			case 3:
				// Transition fwd - Waits for trigger signal and reveals obstacle
				if(isDone && lower_box) {
					float trigger = sqrt(pow((ugv_x - quad_x),2) + pow((ugv_y - quad_y),2));
					if(trigger < TRIGGER_RANGE) {
						float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV_T;
						tTravel = limit(tTravel, 10, 0.25);
						sendTrajectory(tTravel, 0.0, box_x, box_y, F_HEIGHT, 0.0);

						ROS_INFO("Triggered, revealing obstacle!");
						state = 4;
					}

				// Transition bwd - Waits for joy signal "lower_box" = false, moves back up
				}else if(isDone && !lower_box) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, minT);
					sendTrajectory(tTravel, 0.0, box_x, box_y, F_HEIGHT, 0.0);

					ROS_INFO("Joy reset, revealing obstacle...");
					state = 2;
				}
				break;	
	
			case 4:
				// Transition bwd - Waits for reset signal "flying" = false, move back to center
				if(isDone && !flying) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((MOUNT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, minT);
					sendLandTrajectory(tTravel);

					ROS_INFO("Landing at center...");
					state = 0;
					lower_box = false;
					over_box = false;
				}
		}
		loop_rate.sleep();
	}
}
