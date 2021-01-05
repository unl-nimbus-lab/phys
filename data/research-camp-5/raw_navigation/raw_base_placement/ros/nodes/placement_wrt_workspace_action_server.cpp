#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>


#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "placement_wrt_workspace/LaserScanLinearRegression.h"
#include <raw_base_placement/OrientToBaseAction.h>
#include <raw_base_placement/BaseScanLinearRegression.h>
#include <iostream>

#include <angles/angles.h>

using namespace raw_base_placement;

#define TARGET_DISTANCE 0.1
#define MAX_VELOCITY 0.1
#define KP_APPROACH 0.5

class OrientToLaserReadingAction {

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<OrientToBaseAction> as_;
	std::string action_name_;

	std::string service_name;
	std::string cmd_vel_topic;

	float target_distance;
	float max_velocity;

        ros::Publisher cmd_pub;
        ros::ServiceClient client;

public:


	OrientToLaserReadingAction(ros::NodeHandle nh, std::string name, std::string cmd_vel_topic, std::string linreg_service_name) :
		as_(nh,	name, boost::bind(&OrientToLaserReadingAction::executeActionCB, this, _1), false)
	{
		//as_.registerGoalCallback();
		this->action_name_ = name;
		this->service_name = linreg_service_name;
		this->cmd_vel_topic = cmd_vel_topic;

		nh_ = nh;

		target_distance = TARGET_DISTANCE;
		max_velocity = MAX_VELOCITY;

		
		ROS_INFO("Register publisher");

		cmd_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000); // 1000-element buffer for publisher

		ROS_INFO("Create service clinet");

		client = nh_.serviceClient<BaseScanLinearRegression>(service_name);

		as_.start();

	}


	
	geometry_msgs::Twist calculateVelocityCommand(double center, double a, double b,bool &oriented,int &iterator) {
		geometry_msgs::Twist cmd;

		std::cout << "HHHHH: " << fabs(b) << std::endl;
		
        if ( (fabs(b) > 0.1) && (fabs(center) > 0.0005) &&(!oriented) )
		{

		      cmd.angular.z = -b/2;

			//cmd_pub.publish(cmd);
			//std::cout << "cmd.angular.z:  " << cmd.angular.z << std::endl;

		}
		/*
		else if (fabs(center) > 0.005) 
		{
			cmd.linear.y = center / 3;
			//cmd_pub.publish(cmd);
			//std::cout << "cmd.linear.y:  " << cmd.linear.y << std::endl;
		}*/	
		
     
		else if (a > target_distance) 
		{
            
            oriented = true; 
			//cmd.linear.x = a / 3;
			cmd.linear.x = KP_APPROACH * (a - target_distance);
            cmd.angular.z = 0;
			std::cout << "cmd.linear.x:  " << cmd.linear.x << std::endl;
			//cmd_pub.publish(cmd);

		}
        else if(a < target_distance)
        {
      
           cmd.linear.x = -KP_APPROACH * (target_distance-a);
           cmd.angular.z = 0;
           std::cout << "cmd.linear.x:  " << cmd.linear.x << std::endl; 
           if(iterator<6)
           {
              iterator++;   
           } 
           else
           {
              oriented = true;
           }
        }

		// saturation
		if (cmd.linear.x > max_velocity) 
			cmd.linear.x = max_velocity;
		else if (cmd.linear.x < -max_velocity) 
			cmd.linear.x = -max_velocity;

		if (cmd.linear.y > max_velocity) 
			cmd.linear.y = max_velocity;
		else if (cmd.linear.y < -max_velocity) 
			cmd.linear.y = -max_velocity;

		if (cmd.angular.z > max_velocity)
			cmd.angular.z = max_velocity;
		else if (cmd.angular.z < -max_velocity)
			cmd.angular.z = -max_velocity;

		return cmd;
	}


	void executeActionCB(const OrientToBaseGoalConstPtr& goal) {

		BaseScanLinearRegression srv;

		double min_angle = -M_PI/8.0, max_angle=M_PI/8.0, min_dist=0.02, max_dist=0.8;
		nh_.getParamCached("/laser_linear_regression/min_angle", min_angle);
		nh_.getParamCached("/laser_linear_regression/max_angle", max_angle);
		nh_.getParamCached("/laser_linear_regression/min_dist", min_dist);
		nh_.getParamCached("/laser_linear_regression/max_dist", max_dist);

		srv.request.filter_minAngle = angles::from_degrees(min_angle);
		srv.request.filter_maxAngle = angles::from_degrees(max_angle);
		srv.request.filter_minDistance = min_dist;
		srv.request.filter_maxDistance = max_dist;
		std::cout << "min_dist " << min_dist << ", max_dist " << max_dist << std::endl;
		//srv.request.filter_minAngle = -M_PI / 8.0;
		//srv.request.filter_maxAngle = M_PI / 8.0;
		//srv.request.filter_minDistance = 0.02;
		//srv.request.filter_maxDistance = 0.80;

		target_distance = goal->distance;

		ros::Duration max_time(50.0);
		ros::Time stamp = ros::Time::now();
		OrientToBaseResult result;
        bool oriented = false;
        int  iterator = 0;

		while (ros::ok()) {
			ROS_INFO("Call service Client");

			if(client.call(srv)) {
				ROS_INFO("Called service LaserScanLinearRegressionService");
				//std::cout << "result: " << srv.response.center << ", " << srv.response.a << ", " << srv.response.b << std::endl;


				geometry_msgs::Twist cmd = calculateVelocityCommand(srv.response.center, srv.response.a, srv.response.b,oriented,iterator);
				cmd_pub.publish(cmd);

				std::cout << "cmd x:" << cmd.linear.x << ", y: "  << cmd.linear.y << ", z: " << cmd.angular.z << std::endl;

				if ((fabs(cmd.angular.z)  + fabs(cmd.linear.x) ) < 0.001) {
			           
			        ROS_INFO("Point reached");
					result.succeed = true;
					as_.setSucceeded(result);
                    geometry_msgs::Twist zero_vel;
                    cmd_pub.publish(zero_vel);
					break;

				}

				if  (stamp + max_time < ros::Time::now()) {
					result.succeed = false;
				        as_.setAborted(result);
                        geometry_msgs::Twist zero_vel;
                        cmd_pub.publish(zero_vel);
				    	break;
				}

			} else {
				ROS_ERROR("Failed to call service LaserScanLinearRegressionService");

				if  (stamp + max_time < ros::Time::now()) {
					result.succeed = false;
					as_.setAborted(result);
					break;
				}
			}
		}

	}
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "raw_base_placement_wrt_workspace_as");
  ros::NodeHandle n;

  std::string cmd_vel_name = "/cmd_vel";
  std::string service_name = "scan_front_linearregression";

  OrientToLaserReadingAction orientAction(n, "/raw_base_placement/adjust_to_workspace", cmd_vel_name, service_name);

  ROS_INFO("Action Service is ready");


  ros::Rate loop_rate(15);
  while (ros::ok()) {    
     ros::spinOnce();
     loop_rate.sleep();
  }
  return 0;
}

