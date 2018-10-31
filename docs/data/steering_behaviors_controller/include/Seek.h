/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _SEEK_H
#define _SEEK_H

#include "ros/ros.h"
#include "ros/message.h"
#include "tf/tf.h"

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <sstream>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#define PI 3.14159265

#include <libconfig.h++>
using namespace libconfig;

class Seek: public SteeringBehavior {
public:

	/**
	 * @param objective
	 * @param id
	 * @param weight
	 */
	Seek(unsigned int id, std::string pre, Setting* configurationPtr);

	~Seek();

	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	virtual int update();
	virtual void updateState() ;
	virtual void setGoal(float, float);

private:
	geometry_msgs::Pose target;

	//Variables para suscribirse a un topic
	ros::NodeHandle* rosNode;
	ros::Subscriber* odomSubscriber;
	std::vector<ros::Subscriber*> sensorSubscriber;

	//Funciones de Callback para las suscripciones a los topic de la posicion)
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

	//distancia hasta el objetivo
	float errorx;
	float errory;
	float errorw;

	float toleranceToTarget;
	float standardVel;

	void oIdeal();
	int vIdeal();
};

#endif //_SEEK_H
