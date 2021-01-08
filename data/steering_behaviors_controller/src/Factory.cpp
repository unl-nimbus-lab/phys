/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#include "../include/Factory.h"

Factory::Factory()
{
	//Almaceno el espacio para la clase Config de libconfig++
	cfg = new Config;
	//Cargo los valores en el archivo de configuracion
	cfg->readFile(CONFIGFILE);
}

Factory::~Factory()
{
}

int Factory::instanciateBehaviors(	unsigned int id, std::string pre,
									std::vector<SteeringBehavior*>* behaviors,
									std::string type)
{
	//busco el tipo de agente
	std::stringstream agentType;
	agentType << "agents." << type;

	Setting& behaviorsToCreate = cfg ->lookup(agentType.str())["behaviors"];
	int nbBehaviorsType = cfg->lookup(agentType.str())["behaviors"].getLength();


	for (int k = 0; k < nbBehaviorsType; ++k)
	{
		//cargar el vector de comportamiento con los que dice el array behaviorsToCreate
		behaviors->push_back(pickBehavior(behaviorsToCreate[k].c_str() , id, pre));
	}

	return 1;
}

int Factory::deleteBehaviors(std::vector<SteeringBehavior*>* behaviors)
{
	while (!(*behaviors).empty()) {
		SteeringBehavior* auxBhPtr = behaviors->back();
		behaviors->pop_back();
		delete auxBhPtr;
	}
	return 1;
}


SteeringBehavior* Factory::pickBehavior(std::string behaviorName, int id, std::string pre)
{
	std::stringstream bhType;
	bhType << "behaviors." << behaviorName;
	Setting& sets = cfg ->lookup(bhType.str());

	SteeringBehavior* auxBhPtr;

	if (behaviorName == "seek" )
	{
		auxBhPtr = new Seek (id,pre,&sets);
	}
	else if (behaviorName == "obstacleAvoidance")
	{
		auxBhPtr = new ObstacleAvoidance (id,pre,&sets);
	}

	return auxBhPtr;
}

Setting* Factory::getTypeSetting(std::string type)
{
	std::stringstream agType;
	agType << "agents." << type;
	Setting& sets = cfg ->lookup(agType.str());
	return &sets;
}

Setting* Factory::getExperimentSetting()
{
	string aux = cfg ->lookup("experimento");
	std::stringstream expType;
	expType << "experimentos." << aux;
	cout << expType.str() << " cfg returned" << endl;
	Setting& sets = cfg ->lookup(expType.str());
	return &sets;
}

std::string Factory::getCommand(){
	string aux = cfg ->lookup("simulacion");
	return aux;
}
