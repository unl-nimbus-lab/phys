/*************************************************************************
 *
 * FADA-CATEC
 * __________________
 *
 *  [2013] FADA-CATEC
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of FADA-CATEC and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to FADA-CATEC
 * and its suppliers and may be covered by Europe and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from FADA-CATEC.
 *
 * Created on: 23-Oct-2012
 * Engineer: Jonathan Ruiz Páez
 * Email: jruiz@catec.aero
 */

#include "simulation_functions.cpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace std;

extern LandActionClass *land_action;
extern TakeOffActionClass *take_off_action;

ros::Subscriber sub_stage;
ros::Publisher stagevel_pub, state_pub;

void link_loop(const ros::TimerEvent& te)
{
	Twist res;
	Update(&res);

	Vector3 vtemp;

	double rotation_angle =2*PI - tf::getYaw(lastPose.orientation);
	vtemp.x = cos(rotation_angle)*res.linear.x - sin(rotation_angle)*res.linear.y ;
	vtemp.y = cos(rotation_angle)*res.linear.y + sin(rotation_angle)*res.linear.x ;


	res.linear.x = vtemp.x;
	res.linear.y= vtemp.y;

	//Actualizamos el estado en las actions
	land_action->uavUpdateState(actual_state);
	take_off_action->uavUpdateState(actual_state);

	//Intercambiamos linear x por y, para que se corresponda con el testbed

	res.linear.x = res.linear.y;
	res.linear.y = res.linear.x*(-1);

	stagevel_pub.publish(res);
}

void StageCallback(const nav_msgs::Odometry::ConstPtr& fp) {

	//Intercambiamos linear x por y, para que se corresponda con el testbed
	lastPose.position.x = fp->pose.pose.position.y*(-1);
	lastPose.position.y = fp->pose.pose.position.x;
	lastPose.position.z = fp->pose.pose.position.z;

	lastPose.orientation.x = fp->pose.pose.orientation.x;
	lastPose.orientation.y = fp->pose.pose.orientation.y;
	lastPose.orientation.z = fp->pose.pose.orientation.z;
	lastPose.orientation.w = fp->pose.pose.orientation.w;

	lastVel.angular.x = fp->twist.twist.angular.x;
	lastVel.angular.y = fp->twist.twist.angular.y;
	lastVel.angular.z = fp->twist.twist.angular.z;


	lastVel.linear.x = fp->twist.twist.linear.y*(-1);
	lastVel.linear.y = fp->twist.twist.linear.x;
	lastVel.linear.z = fp->twist.twist.linear.z;

	catec_msgs::UALStateStamped res;

	res.header.frame_id = uavID;
	res.header.stamp = ros::Time::now();

	res.ual_state.cpu_usage = 50;
	res.ual_state.memory_usage = 50;
	res.ual_state.remaining_battery = 50;

	res.ual_state.dynamic_state.position.valid = 1;
	res.ual_state.dynamic_state.position.x = lastPose.position.x;
	res.ual_state.dynamic_state.position.y = lastPose.position.y;
	res.ual_state.dynamic_state.position.z = lastPose.position.z;

	res.ual_state.dynamic_state.orientation.valid = 1;
	res.ual_state.dynamic_state.orientation.x = lastPose.orientation.x;
	res.ual_state.dynamic_state.orientation.y = lastPose.orientation.y;
	res.ual_state.dynamic_state.orientation.z = lastPose.orientation.z;

	res.ual_state.dynamic_state.velocity.valid = 1;
	res.ual_state.dynamic_state.velocity.x = lastVel.linear.x;
	res.ual_state.dynamic_state.velocity.y = lastVel.linear.y;
	res.ual_state.dynamic_state.velocity.z = lastVel.linear.z;

	state_pub.publish(res);


	return ;
}


void initialize_link(string id, ros::NodeHandle n_)
{
	string topicname;
	topicname = id;
	topicname.append("/base_pose_ground_truth");
	sub_stage = n_.subscribe(topicname.c_str(), 0, StageCallback);

	topicname = id;
	topicname.append(string("/cmd_vel"));
	stagevel_pub = n_.advertise<geometry_msgs::Twist> (topicname.c_str(), 0);

	topicname =uavID;
	topicname.append("/ual_state");
	state_pub = n_.advertise<catec_msgs::UALStateStamped> (topicname.c_str(), 0);

}







