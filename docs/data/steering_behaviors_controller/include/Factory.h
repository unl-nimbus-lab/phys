/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _FACTORY_H
#define _FACTORY_H

#include "SteeringBehavior.h"
#include "ObstacleAvoidance.h"
#include "Seek.h"

#include <string>
#include <vector>
#include <sstream>

#include <libconfig.h++>
using namespace libconfig;

#define CONFIGFILE "./src/steering_behaviors_controller/simulation.cfg"

class Factory {

public:

	Factory();

	~Factory();

	int instanciateBehaviors(unsigned int id, std::string pre,
							std::vector<SteeringBehavior*>* behaviors,
							std::string type);

	int getAgents();

	Setting* getTypeSetting(std::string);

	Setting* getExperimentSetting();

	std::string getCommand();

	int deleteBehaviors(std::vector<SteeringBehavior*>* behaviors);

private:

	Config* cfg;
	int nbAgents;

	SteeringBehavior* pickBehavior(std::string behaviorName, int id, std::string pre);

};

#endif //_FACTORY_H
