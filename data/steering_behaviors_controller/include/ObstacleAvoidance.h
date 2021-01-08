/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _OBSTACLEAVOIDANCE_H
#define _OBSTACLEAVOIDANCE_H

#include "ros/ros.h"
#include "ros/message.h"
#include "tf/tf.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include "SteeringBehavior.h"

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <libconfig.h++>
using namespace libconfig;

#define PI 3.14159265

class ObstacleAvoidance : public SteeringBehavior {

	public:

		ObstacleAvoidance(unsigned int id, std::string pre, Setting* configurationPtr);

		~ObstacleAvoidance();

		/**
		 * gets the last data and actualizes the desiredTwist
		 */
		virtual int update();
		virtual void updateState() ;

	private:

		//Variables para suscribirse a un topic
		ros::NodeHandle* rosNode;

		//Funcion de Callback y variables para la suscripcion al topic del laser
		ros::Subscriber* sensorSubscriber;
		void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure);
		//Array para almacenar los valores recibidos de las funciones de callback para el posterior calculo con las mismas
		float* laser;
		//Variables inherentes al sensor laser
		int haz;					//cantidad de haces del laser
		int divisiones;					//cantidad de haces por sector
		float prescicion;			//separacion entre mediciones del laser, en angulos, es decir, la prescicion del sensor
		float abanico;				//angulo total barrido por el sensor

		//Funcion de Callback y variables para la suscripcion al topic del odometro (posicion y twist)
		ros::Subscriber* odomSubscriber;
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

		//distancia máxima a la que actua el comportamiento
		float distMax;
		float distMin;
		float minLaser;
		int minLaserIndex;

		//Funciones privadas
		void oIdeal();
		int vIdeal();
};

#endif //_OBSTACLEAVOIDANCE_H
