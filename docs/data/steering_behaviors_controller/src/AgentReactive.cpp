/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "AgentReactive.h"

AgentReactive::AgentReactive(unsigned int id, string type, Factory* factoryPtr) : Agent(id, type, factoryPtr)
{
	//saco los pesos constantes de la conf
	Setting& ws = (*configurationPtr)["weights"];
	int nbWs = (*configurationPtr)["weights"].getLength();
	for (int k = 0; k < nbWs; ++k)
	{
		float aux = ws[k];
		pesos.push_back(aux);
	}
}

AgentReactive::~AgentReactive()
{

}

void AgentReactive::updateWeights(std::vector<float> estado)
{
	/*for (std::vector<float>::iterator itb = pesos.begin(); itb != pesos.end(); ++itb)
	{
		cout << *itb << " ";
	}*/
}

int AgentReactive::update()
{

	int behaviorFlag[behaviors.size()];
	twists.clear();
	bool llegada = false;

	for (int i = 0; i < behaviors.size(); ++i)
	{
		geometry_msgs::Twist tw;
		behaviorFlag[i] = behaviors[i]->update();
		if (behaviors[i]->getType()=="seek")
		{
			if (behaviorFlag[i] == 0) {
				llegada = true;
			}
			tw.angular.z = behaviors[i]->getDesiredO();
			tw.linear.x = behaviors[i]->getDesiredV();
			twists.push_back(tw);
		}
		else if (behaviors[i]->getType()=="avoidObstacles")
		{
			if(behaviorFlag[i] != 0) {
				tw.angular.z = behaviors[i]->getDesiredO();
				tw.linear.x = behaviors[i]->getDesiredV();
				twists.push_back(tw);
			}
		}
	}

	blend();

	if (llegada) {
		return 0;
	}
	return 1;
}
