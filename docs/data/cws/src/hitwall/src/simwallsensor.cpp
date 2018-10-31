#include <iostream>
#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <tinyxml.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

bool KeepGoing = true;
bool ShouldMeasureInitialPosition = true;
float InitialPosition = 0;
float xpos;
float ypos;
float theta;

void subFunc(nav_msgs::Odometry eventMsg)
{
	
	//raw data
	xpos = eventMsg.pose.pose.position.x;
	ypos = eventMsg.pose.pose.position.y;
	theta = eventMsg.twist.twist.angular.z;
	
	//std::cout << " \t x:" << xpos << "\t y: " << ypos << "\t theta: " << theta << "\n";

}

/////////////////////////////////////////////////
// Function is called everytime a message is received.
/*
void cb(ConstWorldStatisticsPtr &_msg)
{
  // Dump the message contents to stdout.
  std::cout << _msg->DebugString();
}
*/

int main(int argc, char **argv)
{

	ros::init(argc, argv, "simwallsensor");
	ros::NodeHandle SimwallsensorNode;
	ros::Rate loop_rate(10);
	
	
	//GAZEBO TOPIC SUBSCRIBER
/*
  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/hokuyo/link/laser/scan", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::transport::fini();
*/
	
	//SETUP CLIENT FOR MODEL STATE
	ros::service::waitForService("/gazebo/get_model_state");
	ros::ServiceClient getModelStateClient =
       SimwallsensorNode.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	gazebo_msgs::GetModelState getModelState;
	getModelState.request.model_name = "mobile_base";

	//SPAWN CUBES
	ros::service::waitForService("/gazebo/spawn_sdf_model");
	ros::ServiceClient spawnCubesClient =
       SimwallsensorNode.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	//~/gazebo_models/nist_maze_wall_240/
	gazebo_msgs::SpawnModel spawnModelSrv;
	spawnModelSrv.request.model_name = "nist_maze_wall_240";
	spawnModelSrv.request.model_xml = "nist_maze_wall_240";
	spawnModelSrv.request.initial_pose.position.x = 1;
	spawnModelSrv.request.initial_pose.position.y = 1;
	spawnModelSrv.request.initial_pose.position.z = 0.5;
	spawnCubesClient.call(spawnModelSrv);
	
	/*
	//FROM ROS spawnbox.cpp start
	
	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient spawn_model_client = SimwallsensorNode.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel spawn_model;
	spawn_model.request.model_name = "box1";

	// load sdf file
	std::string urdf_filename = std::string("~/gazebo_models/nist_maze_wall_240/model.sdf");
	ROS_DEBUG("loading file: %s",urdf_filename.c_str());
	// read sdf / gazebo model xml from file
	TiXmlDocument xml_in(urdf_filename);
	xml_in.LoadFile();
	std::ostringstream stream;
	stream << xml_in;
	spawn_model.request.model_xml = stream.str(); // load xml file
	ROS_DEBUG("XML string: %s",stream.str().c_str());

	spawn_model.request.robot_namespace = "";
	geometry_msgs::Pose pose;
	pose.position.x = pose.position.y = 0; pose.position.z = 1;
	pose.orientation.w = 1.0; pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
	spawn_model.request.initial_pose = pose;
	spawn_model.request.reference_frame = "";
	spawn_model_client.call(spawn_model);
	
	//FROM ROS spawnbox.cpp end
	
	*/
	
	
	
	std::cout << "model out..\n";
	
	
	ros::Publisher pubName = SimwallsensorNode.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	
		ros::Subscriber subName = SimwallsensorNode.subscribe("/odom", 100, subFunc);
		
	while (ros::ok())
  	{
  	
  		getModelStateClient.call(getModelState);
		std::cout << getModelState.response.pose.position.x << "\t";
		std::cout << getModelState.response.pose.position.y << "\t";
		
		 tf::Pose pose;
  		 tf::poseMsgToTF(getModelState.response.pose, pose);
  		 double yaw_angle = tf::getYaw(pose.getRotation());

		//getModelState.response.pose.orientation.z
		std::cout << yaw_angle << "\n";
	
  		if(KeepGoing == true){
  		geometry_msgs::Twist twistMsg;
		twistMsg.linear.x=0.0;
		twistMsg.linear.y=0.0;
		twistMsg.linear.z=0.0;
		twistMsg.angular.x=0.0;
		twistMsg.angular.y=0.0;
		twistMsg.angular.z=0.0;
	  	
	  	pubName.publish(twistMsg);
	  	} else {
	  	std::cout << " Really Stopped!!!";
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

}
