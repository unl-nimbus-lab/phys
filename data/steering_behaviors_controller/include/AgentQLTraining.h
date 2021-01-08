/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _AGENTQLTRAINING_H
#define _AGENTQLTRAINING_H

#include "ros/ros.h"
#include "ros/message.h"
#include "tf/tf.h"

#include "Agent.h"

#include <math.h>
#include <sstream>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#define PI 3.14159265

#include <libconfig.h++>
using namespace libconfig;

struct qTableOutput
{
	int visits;
	float qValue;
};

struct reinforcement
{
	int behaviorNb;
	float reinforcementState;
	float reinforcementValue;
	std::string message;
};

class AgentQLTraining: public Agent {
public:

	AgentQLTraining(unsigned int id, string type, Factory* factoryPtr);

	~AgentQLTraining();

	int update();

	std::vector< std::vector<float> >* wCombinacionesPosibles;
	std::vector<reinforcement> critic;
	std::map<std::vector<float> , qTableOutput, std::less< std::vector<float> >, std::allocator< std::pair<std::vector<float> , qTableOutput> > > qTable;

//private:

	virtual void updateWeights(std::vector<float>);

	//Fcs auxiliares al constructor
	int newQTable();
	int loadQTable(std::string, int);
	//Fcs auxiliares para newQtable y loadQTable
	void instanciarWcombinaciones(int wCantDiscretizacion, int size);
	void wPermutaciones(std::vector<float> valores, std::vector<float> individuo , int longitud, std::vector< std::vector<float> >* contenedor);
	void sPermutaciones(std::vector< std::vector<float> > valores, std::vector<float> individuo , int longitud, std::vector< std::vector<float> >* contenedor);
	int writeQTableToFile();
	//void printPerm(std::vector< std::vector<float> >);

	//variables para la qtable
	std::string file;

	int allocateNb;
	std::pair< std::vector<float> , qTableOutput>* allocP;


	std::vector<int> refuerzosAplicados;
	float gamma;
	int maxVisitasDif;
	float dtCastigo;
	int lastMemoriaSize;

	int wCantDiscretizacion;
	int weightSize;

	//Fcs auxiliares a getWeights
	std::vector<float> getBestWfromQTable(std::vector<float>);

	std::vector<float> getRandomWfromQTable(std::vector<float>);
	float chooseTime;

	int checkVisits(std::vector<float>);
	void criticCheck();
	void actualizarQTable(int refuerzo);
	void dtPunish();


	std::vector< std::map<std::vector<float> , qTableOutput>::iterator > memoria;
	std::vector<float> ultimaColision;

	void printPerm(std::vector< std::vector<float> > perm );

};

#endif //_AGENTQLTRAINING_H
