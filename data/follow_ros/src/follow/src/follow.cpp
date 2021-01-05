/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//#include "OniSampleUtilities.h"
#include </opt/OpenNI-Linux-ARM-2.2/Include/OpenNI.h>
//#include "../Common/OniSampleUtilities.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace openni;
using namespace std;

typedef struct point
{
	float x;
	float y;
	float z;
}point;

inline int sign(double x)
{
	if(x>0)
		return 1;
	if(x==0)
		return 0;
	if(x<0)
		return -1;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "follow");
	ros::NodeHandle node;
	//ros::Publisher cmdpub_ = node.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
	ros::Publisher cmdpub_ = node.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/navi", 1);
	double min_y_(-0.35), max_y_(0.35),
			min_x_(0.2), max_x_(0.5),
			max_z_(1.5), goal_z_(0.6),
			z_scale_(2), x_scale_(6);
	double accel_lim_v(0.6);
	double accel_lim_w(5.4);
	double speed_lim_v(1);
	double speed_lim_w(5.5);
	double decel_factor(1);
	bool enabled_(true);
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	VideoStream depth;

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}
	VideoMode modeDepth;
	modeDepth.setResolution( 640, 480 );
	modeDepth.setFps( 30 );
	modeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
	depth.setVideoMode(modeDepth);

	double RealWorldXtoZ=2*tan(1.0226/2);
	double RealWorldYtoZ=2*tan(0.796616/2);
	double real_factor_x[307200];
	double real_factor_y[307200];
	int i,j;
	for( i=0;i<480;i++)
	{
		for(j=0;j<640;j++)
		{
			real_factor_x[i*640+j]= ((float)j / 640.0 - .5f)*RealWorldXtoZ;
			real_factor_y[i*640+j] = (.5f - (float)i / 480.0)*RealWorldYtoZ;
		}
	}

	if( device.isImageRegistrationModeSupported(
			IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
	{
		device.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
	}

	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}
	VideoFrameRef frame;
	geometry_msgs::Twist last_cmd;
	last_cmd.linear.x=0;last_cmd.angular.z=0;
	geometry_msgs::Twist current_cmd = last_cmd;
	double period(0.0);
	//while (!wasKeyboardHit()&&node.ok())
	while(1)
	{
		double timestart = ros::Time::now().toSec();
		int changedStreamDummy;
		VideoStream* pStream = &depth;
		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != STATUS_OK)
		{
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}

		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			printf("Read failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}

		vector<point> points;
		point depth_point;
        float x(0.0),y(0.0),z(0.0),xx(0.0);
		DepthPixel *pDepth = (DepthPixel*)frame.getData();
		for(int i=0;i<frame.getHeight();i++)
		{
			for(int j=0;j<frame.getWidth();j++)
			{
				xx = pDepth[i*frame.getWidth()+j];
				//CoordinateConverter::convertDepthToWorld (depth,i,j,xx,&x,&y,&z);
				x=real_factor_x[i*640+j]*xx;
				y=real_factor_y[i*640+j]*xx;
				//cout<<real_factor_x[i*640+j]<<"  "<<real_factor_y[i*640+j]<<" ***   x="<<x<<"y="<<y<<endl;
				depth_point.y=x*0.001;
				depth_point.x=y*0.001;
				depth_point.z=xx*0.001;
				points.push_back(depth_point);
				//cout<<xx<<endl;
			}
		}
                
		//X,Y,Z of the centroid
		x = 0.0;
		y = 0.0;
		z = 1e6;
		//Number of points observed
		unsigned int n = 0;
		//Iterate through all the points in the region and find the average of the position
		for (vector<point>::iterator iter = points.begin(); iter != points.end(); iter++)
		{
			//First, ensure that the point's position is valid. This must be done in a seperate
			//if because we do not want to perform comparison on a nan value.
			if (!isnan(x) && !isnan(y) && !isnan(z))
			{
				//Test to ensure the point is within the aceptable box.
				if (-iter->y > min_y_ && -iter->y < max_y_ && iter->x < max_x_ && iter->x > min_x_ && iter->z < max_z_ && iter->z > 0.03)
				{
					//Add the point to the totals
					x += iter->x;
					y += iter->y;
					z = std::min(z, iter->z);
					n++;
				}
			}
		}
		//If there are points, find the centroid and calculate the command goal.
		//If there are no points, simply publish a stop goal.

		if (n>100)
		{
			x /= n;
			y /= n;
			if(z > max_z_){
				ROS_DEBUG("No valid points detected, stopping the robot");
				if (enabled_)
				{
					current_cmd.linear.x = 0;
					current_cmd.angular.z = 0;
					//cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
				}
			}
			else
			{
				current_cmd.linear.x = (z - goal_z_) * z_scale_;
				if(current_cmd.linear.x<0)
					current_cmd.linear.x=0;
				current_cmd.angular.z = y * x_scale_;
				//cmdpub_.publish(cmd);
			}

		}
		else
		{
			current_cmd.linear.x = 0;
			current_cmd.angular.z = 0;
		}
		period = ros::Time::now().toSec() -timestart;
		current_cmd.linear.x  =
				current_cmd.linear.x  > 0.0 ? std::min(current_cmd.linear.x,  speed_lim_v) : std::max(current_cmd.linear.x,  -speed_lim_v);
		current_cmd.angular.z =
				current_cmd.angular.z > 0.0 ? std::min(current_cmd.angular.z, speed_lim_w) : std::max(current_cmd.angular.z, -speed_lim_w);
        if(1)
		{
			double v_inc, w_inc, max_v_inc, max_w_inc;

			v_inc = current_cmd.linear.x - last_cmd.linear.x;
			if (last_cmd.linear.x*current_cmd.linear.x < 0.0)
			{
				// countermarch (on robots with significant inertia; requires odometry feedback to be detected)
				max_v_inc = decel_factor*accel_lim_v*period;
			}
			else
			{
				max_v_inc = ((v_inc*current_cmd.linear.x > 0.0)?accel_lim_v:decel_factor*accel_lim_v)*period;
			}

			w_inc = current_cmd.angular.z - last_cmd.angular.z;
			if (last_cmd.angular.z*current_cmd.angular.z < 0.0)
			{
				// countermarch (on robots with significant inertia; requires odometry feedback to be detected)
				max_w_inc = decel_factor*accel_lim_w*period;
			}
			else
			{
				max_w_inc = ((w_inc*current_cmd.angular.z > 0.0)?accel_lim_w:decel_factor*accel_lim_w)*period;
			}
			double MA = sqrt(    v_inc *     v_inc +     w_inc *     w_inc);
			double MB = sqrt(max_v_inc * max_v_inc + max_w_inc * max_w_inc);

			double Av = std::abs(v_inc) / MA;
			double Aw = std::abs(w_inc) / MA;
			double Bv = max_v_inc / MB;
			double Bw = max_w_inc / MB;
			double theta = atan2(Bw, Bv) - atan2(Aw, Av);
			//
			if (theta < 0)
			{
				// overconstrain linear velocity
				max_v_inc = (max_w_inc*std::abs(v_inc))/std::abs(w_inc);
			}
			else
			{
				// overconstrain angular velocity
				max_w_inc = (max_v_inc*std::abs(w_inc))/std::abs(v_inc);
			}

			if (std::abs(v_inc) > max_v_inc)
			{
				// we must limit linear velocity
				current_cmd.linear.x  = last_cmd.linear.x  + sign(v_inc)*max_v_inc;
			}

			if (std::abs(w_inc) > max_w_inc)
			{
				// we must limit angular velocity
				current_cmd.angular.z = last_cmd.angular.z + sign(w_inc)*max_w_inc;
			}
			//cout<<"current_cmd->linear.x = "<<current_cmd.linear.x<<endl;
		}
		cmdpub_.publish(current_cmd);

		last_cmd = current_cmd;
	}

	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}
