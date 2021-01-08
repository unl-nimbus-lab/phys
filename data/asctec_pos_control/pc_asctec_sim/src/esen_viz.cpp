#include <ros/ros.h>
#include<atraj.h>
#include "atraj.cpp"

#include <std_msgs/Bool.h>

#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_state.h>
#include <pc_asctec_sim/CMatrix.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <string.h>

#define freq 40

using namespace std;

bool esen = false;
bool recalc = false;
bool first = true;

float x,y,ox,oy;

string world, quad_name, quad_frame;
visualization_msgs::Marker points, avoid, wp;
pc_asctec_sim::pc_state q_st;

ros::Publisher calc_pub, avoid_pub, wp_pub;

NEW_PATH temp;
pc_asctec_sim::CMatrix C;
Trajectory_Accel * path_gen = new Trajectory_Accel(1.0);

void recalcCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) {
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		p.z = 1.0;

		points.action = visualization_msgs::Marker::ADD;
		points.points.push_back(p);
		calc_pub.publish(points);
	};
}

void esenCallback(const std_msgs::Bool::ConstPtr& msg)
{
	esen = msg->data;
}

void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg)
{
	q_st = *msg;
}

void setAvoidPath()
{
	int counter = 0;
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = 1.0;

	for(int i=0; i<temp.cmd.points; i++) {
		float t = 0.0;
		while(t <= temp.cmd.duration[i]) {
			t+=0.1;
			avoid.points.push_back(p);
			p.x = C.c5x[i] * pow(t,5) + C.c4x[i] * pow(t,4) + C.c3x[i] * pow(t,3) + C.c2x[i] * pow(t,2) + C.c1x[i] * t + C.c0x[i];
			p.y = C.c5y[i] * pow(t,5) + C.c4y[i] * pow(t,4) + C.c3y[i] * pow(t,3) + C.c2y[i] * pow(t,2) + C.c1y[i] * t + C.c0y[i];
			counter++;
			avoid.points.push_back(p);
		}
	}
	if(counter % 2 == 0) {
		avoid.points.push_back(p);
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Esen_Viz");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;

	tf::StampedTransform transform;
	tf::TransformListener listener;

	calc_pub = nh.advertise<visualization_msgs::Marker>("/hummingbird_1/calc_points", 10);
	avoid_pub = nh.advertise<visualization_msgs::Marker>("/hummingbird_1/avoid", 10);
	wp_pub = nh.advertise<visualization_msgs::Marker>("/hummingbird_1/wp", 10);

	ros::Subscriber recalc_sub = nh.subscribe("/hummingbird_1/recalc", 10, recalcCallback);
	ros::Subscriber esen_sub = nh.subscribe("/hummingbird_1/esen", 10, esenCallback);
	ros::Subscriber state_sub = nh.subscribe("/hummingbird_1/state", 10, stateCallback);

	listener.waitForTransform("/odom", "/vicon/hummingbird_1/hummingbird_1", ros::Time(0), ros::Duration(3.0));

	points.header.frame_id = "/odom";
	points.header.stamp = ros::Time::now();
	points.ns = "Calc points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::SPHERE_LIST;

	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.a = 1.0;
	points.color.r = 1.0;
	points.color.g = 0.5;

	wp.header.frame_id = "/odom";
	wp.header.stamp = ros::Time::now();
	wp.ns = "wp";
	wp.action = visualization_msgs::Marker::ADD;
	wp.pose.orientation.w = 1.0;
	wp.id = 0;
	wp.type = visualization_msgs::Marker::SPHERE_LIST;

	wp.scale.x = 0.2;
	wp.scale.y = 0.2;
	wp.color.a = 1.0;
	wp.color.r = 0.6;
	wp.color.b = 1.0;

	avoid.header.frame_id = "/odom";
	avoid.header.stamp = ros::Time::now();
	avoid.id = 0;
	avoid.action = visualization_msgs::Marker::ADD;
	avoid.type = visualization_msgs::Marker::LINE_LIST;
	avoid.color.a = 1.0;				
	avoid.color.b = 1.0;
	avoid.color.g = 0.8;

	avoid.scale.x = 0.05;
	avoid.scale.y = 0.05;

	listener.waitForTransform("/odom", "/vicon/tiki/tiki", ros::Time(0), ros::Duration(3.0));

	ROS_INFO("Running: Esen_viz");

	while(ros::ok()) {
		if(esen) {
			if(first) {	
				first = false;
				temp.state = q_st;
				temp.cmd.x[0] = temp.cmd.x[1] = ox-1;
				temp.cmd.x[2] = 0.4;
				temp.cmd.y[0] = oy - 0.5;
				temp.cmd.y[1] = oy + 0.5;
				temp.cmd.y[2] = 1.75;
				temp.cmd.z[0] = temp.cmd.z[1] = temp.cmd.z[2] = 1.0;
				temp.cmd.duration[0] = sqrt(pow(q_st.x - temp.cmd.x[0],2) + pow(q_st.y - temp.cmd.y[0],2))/0.6;
				temp.cmd.duration[1] = sqrt(pow(temp.cmd.x[0] - temp.cmd.x[1],2) + pow(temp.cmd.y[0] - temp.cmd.y[1],2))/0.6;
				temp.cmd.duration[2] = sqrt(pow(temp.cmd.x[1] - temp.cmd.x[2],2) + pow(temp.cmd.y[1] - temp.cmd.y[2],2))/0.6;
				temp.cmd.points = 3;
				C = *path_gen->getPathConstants(&temp, &C);
				setAvoidPath();

				geometry_msgs::Point p;

				p.x = x;
				p.y = y;
				p.z = 1.0;
				wp.points.push_back(p);

				p.x = temp.cmd.x[0];
				p.y = temp.cmd.y[0];
				p.z = temp.cmd.z[0];
				wp.points.push_back(p);

				p.x = temp.cmd.x[1];
				p.y = temp.cmd.y[1];
				p.z = temp.cmd.z[1];
				wp.points.push_back(p);

				p.x = temp.cmd.x[2];
				p.y = temp.cmd.y[2];
				p.z = temp.cmd.z[2];
				wp.points.push_back(p);
			}
			avoid_pub.publish(avoid);
			wp_pub.publish(wp);	
		}

		listener.lookupTransform("/odom", "/vicon/hummingbird_1/hummingbird_1", ros::Time(0), transform);
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();

		listener.lookupTransform("/odom", "/vicon/tiki/tiki", ros::Time(0), transform);
		ox = transform.getOrigin().x();
		oy = transform.getOrigin().y();

		ros::spinOnce();
		loop_rate.sleep();
	}
}

