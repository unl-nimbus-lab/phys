/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _AGENT_H
#define _AGENT_H

#include "ros/ros.h"
#include "ros/message.h"
#include "tf/tf.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "SteeringBehavior.h"
#include "Factory.h"

#include <string>
#include <vector>
#include <sstream>

#include <math.h>

#include <iostream>
#include <fstream>
#include <errno.h>

#define PI 3.14159265

struct qlearningStats
{
	float distanciaRecorrida;
	float tiempo;
	std::vector<float> mins;
	std::vector<float> maxs;
	float qvalTotal;
};

class Agent {
public:

	Agent(unsigned int id, string type, Factory* factoryPtr);

	~Agent();

	virtual int update();

	void setNewObjective(std::pair<float, float>);
	qlearningStats getStats();
	std::vector< std::pair< std::string , int > > getRefsAcumulados();

protected:
	string myType;
	std::vector<float> pesos;
	std::vector<SteeringBehavior*> behaviors;
	Setting* configurationPtr;
	std::vector<geometry_msgs::Twist> twists;
	std::vector< std::pair< std::string , int > > refuerzosAcumulados;
	float qvalAcumulado;
	std::vector<float> getOneVectorState();
	std::vector< std::vector<float> > getIndividualVectorState();
	int tOut;

	void blend();

private:
	//Id del robot del controlador
	unsigned int robotId;
	std::vector< float > actualState;
	std::vector< float > ansState;
	int roundCounter;

	//Variables para publicar por un topic
	ros::NodeHandle* rosNode;
	ros::Subscriber* odomSubscriber;
	ros::Publisher* ctrlPublisher;
	//ros::Subscriber* ctrlSubscriber;
	std::stringstream* pretopicname;

	//Funciones de Callback para las suscripciones al odometro
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

	//variables para almacenar los datos del odometro
	float x;
	float y;
	float tita;

	// void printState();

	virtual void updateWeights(std::vector<float> estado);

	//Fcs y Vbles para stats
	float stiempo;
	float disty;
	float distx;

	float minObs;

	std::vector< float > minState;
	std::vector< float > maxState;
	void minMaxStats();

	Factory* myfactory;
};

#endif //_AGENT_H
