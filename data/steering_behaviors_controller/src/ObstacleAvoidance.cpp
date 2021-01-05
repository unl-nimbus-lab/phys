/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#include "ObstacleAvoidance.h"

/**
 * ObstacleAvoidance implementation
 *
 * Wall avoidance steers to avoid potential collisions
 * with a wall.
 */

void ObstacleAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::LaserScan tmpLaser = *scan;
	for (int i = 0; i < haz; ++i)
	{
		laser[i]=scan->ranges[i];
	}
}

ObstacleAvoidance::ObstacleAvoidance(unsigned int id, std::string pre, Setting* configurationPtr) : SteeringBehavior(id, pre, configurationPtr)
{
	//Cargar Valores de configuracion
	distMax = (*configurationPtr)["distMax"];
	distMin = (*configurationPtr)["distMin"];
	haz = (*configurationPtr)["haz"];
	prescicion = (*configurationPtr)["prescicion"];
	vmax = (*configurationPtr)["desiredV"];

	divisiones = haz/nbVar;
	abanico = prescicion*haz;

	//generar el nombre del nodo con el robotId, inicializa el nodo
	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	ros::M_string remappingsArgs;
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;
	remappingsArgs.insert(ros::M_string::value_type( "aoBh", nameMaster.str()));
	std::stringstream name;
	name << "obstacleavoidance_" << robotId;
	ros::init(remappingsArgs, name.str());
	rosNode = new ros::NodeHandle;

	//inicializo el puntero con las variables para almacenar los valores de los lasers
	laser = new float[haz];	//almacena base_scan
	for (int i = 0; i < haz; ++i)
	{
		laser[i]=5.000;
	}

	// Subscripcion al topic base_scan, crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	std::stringstream lasertopicname;
	lasertopicname << pretopicname << "base_scan";
	sensorSubscriber = new ros::Subscriber;
	*sensorSubscriber = (*rosNode).subscribe<sensor_msgs::LaserScan>(lasertopicname.str(), 1000, &ObstacleAvoidance::sensorCallback,this);
	}

ObstacleAvoidance::~ObstacleAvoidance()
{
	delete rosNode;
	delete [] laser;
	delete odomSubscriber;
	delete sensorSubscriber;
}

int ObstacleAvoidance::update()
{
	updateState();
	oIdeal();//calcular la orientacion ideal
	int flag = vIdeal();//calcular la velocidad ideal
	//discretizo el valor de estado continuo
	discretizarEstado();
	return flag;
}

void ObstacleAvoidance::updateState()
{
	//Busqueda del minimo global
	int minIndexGlobal = 0;
	for (int i = 0; i < haz; ++i)
	{
		if (laser[i] < laser[minIndexGlobal]) {
			minIndexGlobal = i;
		}
	}
	minLaserIndex = minIndexGlobal;

	//Busqueda del minimo para cada sector, cada valor del arreglo de estado
	int sector = 0;
	int minSector = 0;
	for (int i = 0; i <= haz; ++i)
	{
		if (i == haz/nbVar*(sector+1)) {
			stateContinuous[sector] = laser[minSector];
			sector ++;
			minSector = i;
		}
		if (laser[i] < laser[minSector]) {
			minSector = i;
		}
	}

}

void ObstacleAvoidance::oIdeal()
{
	int minIndex = minLaserIndex;
	if (minIndex==269)
	{
		//si el obstaculo esta en el primer laser tomo como minimo el siguiente así laser[minIndex-1] no da error
		minIndex = 268;
	}
	float distUno = laser[minIndex];
	float distDos = laser[minIndex+1];
	float alphaUno = (minIndex - 135) * PI /180;
	float alphaDos = (minIndex + 1 - 135) * PI /180;
	float distUnoX = distUno * cos(alphaUno);
	float distUnoY = distUno * sin(alphaUno);
	float distDosX = distDos * cos(alphaDos);
	float distDosY = distDos * sin(alphaDos);
	float obsAng;
	if (distUnoX == distDosX) {
		obsAng = PI/2;	//Si el obstaculo esta de frente del agente, las distancias en x son iguales, y la inclinacion se hace infinita...equivalente a una linea vertical
	} else {
		obsAng = atan2((distDosY - distUnoY),(distDosX - distUnoX));
	}
	//calcularl el error respecto a la orientacion actual
	float angRespuesta = (obsAng+(PI/2));
	if (angRespuesta > PI) {
		angRespuesta = angRespuesta - 2 * PI;
	}
	//cout << minIndex << " minimo -> " << laser[minIndex] << " . Respuesta " << angRespuesta*180/PI << " " << angRespuesta << endl;
	setDesiredO(angRespuesta);
}

int ObstacleAvoidance::vIdeal()
{
	if (laser[minLaserIndex]>distMax) {
		setDesiredV(0.0);
		return 0;
	}
	else if (laser[minLaserIndex]<distMin) {
		setDesiredV(vmax);
		return (-1);
	}
	else
	{
		float m =(vmax-0)/(distMin-distMax);
		float b = -m*distMax;
		float desiredV = (m*laser[minLaserIndex])+b;
		setDesiredV(desiredV);
		return 1;
	}
}
