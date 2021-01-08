/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _STEERINGBEHAVIOR_H
#define _STEERINGBEHAVIOR_H

#include "ros/ros.h"

#include <libconfig.h++>
using namespace libconfig;

#include <string>
#include <vector>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;
using std::string;

#define PI 3.14159265

class SteeringBehavior {
public:

	SteeringBehavior(unsigned int id, std::string pre, Setting* configurationPtr);

	~SteeringBehavior();

	std::vector< std::vector<float> > getPosibleValues();

	virtual int update() ; // función virtual pura
	virtual std::vector<float> getState() ;
	virtual void updateState() ;

	virtual string getName();
	virtual string getType();
	virtual float getDesiredV();
	virtual float getDesiredO();
	virtual void setGoal(float, float); //only for seek

protected:

	string myName;
	std::string myType;
	unsigned int robotId;
	int nbVar;
	float vmax;

	std::vector<float> stateDiscrete;
	std::vector<float> stateContinuous;
	std::vector<float> valoresEstado;


	//para estadisticas
	std::vector<float> minStateContinuous;
	std::vector<float> maxStateContinuous;

	std::string pretopicname;

	float desiredV;
	float desiredO;

	//Clase para el ingreso de parametros de configuración
	Setting* config;

	//variables para almacenar los datos del odometro
	float x;
	float y;
	float tita;

	void setDesiredV(float y);
	void setDesiredO(float z);

	void discretizarEstado ();
};

#endif //_STEERINGBEHAVIOR_H
