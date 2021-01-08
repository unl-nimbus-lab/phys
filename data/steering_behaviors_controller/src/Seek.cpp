/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "Seek.h"

void Seek::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
	tf::Pose pose;
	tf::poseMsgToTF(odom->pose.pose, pose);
	tita = tf::getYaw(pose.getRotation());	//tita: orientación en radianes para el marco coordenadas 2D, transformado a partir del marco de referencia 3D expresado por el quaternion (x,y,z,w) de la estructura orientation.
	//a partir de la posición inicial (0rad) tiene un rango (-PI/2 ; +PI/2] siendo el giro positivo hacia la izquierda del vehículo
}

Seek::Seek(unsigned int id, std::string pre, Setting* configurationPtr) : SteeringBehavior(id, pre, configurationPtr)
{
	//Cargar Valores de configuracion
	standardVel = (*configurationPtr)["desiredV"];
	toleranceToTarget = (*configurationPtr)["toleranceToTarget"];

	//generar el nombre del nodo con el robotId, inicializa el nodo
	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	ros::M_string remappingsArgs;
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;
	remappingsArgs.insert(ros::M_string::value_type( "seekBh", nameMaster.str()));
	std::stringstream name;
	name << "seek_" << robotId;
	ros::init(remappingsArgs, name.str());
	rosNode = new ros::NodeHandle;

	// Subscripcion al topic odom, crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	std::stringstream topicname;
	topicname << pretopicname << "odom" ;
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(topicname.str(), 1000, &Seek::odomCallback,this);

	setDesiredV(standardVel);
}

Seek::~Seek()
{
	delete rosNode;
	delete odomSubscriber;
}

/**
 * gets the last data and actualizes the desiredTwist
 * @param myPose
 */
int Seek::update()
{
	updateState();//actualizar estado
	oIdeal();//calcular la orientacion ideal
	int flag = vIdeal();//calcular la velocidad ideal
	discretizarEstado();
	return flag;
}

int Seek::vIdeal()
{
	if (toleranceToTarget>stateContinuous[0])
	{
		//si es menor que la tolerancia se detiene
		setDesiredV(0.0);
		return 0;
	}
	else if (toleranceToTarget*3>stateContinuous[0])
	{
		setDesiredV(standardVel/3);
	}
	else
	{
		setDesiredV(standardVel);
	}
	return 1;
}

void Seek::oIdeal()
{
	float objAng = atan2(errory,errorx);	//este es el angulo en el q se encuentra el objetivo relativo a las coords del robot (odom). El angulo de la fuerza del comportamiento es la diferencia con la orientacion actual
	float auxOrientation = objAng - tita;
	if (auxOrientation >= PI ) {
		auxOrientation -= 2*PI;
	}
	setDesiredO(auxOrientation);
}

/* Calcula esl estado en el espacio continuo y*
* lo matchea a las opciones de estado discreto*/
void Seek::updateState()
{
	errorx = target.position.x - x;
	errory = target.position.y - y;
	stateContinuous[0]=sqrt(pow(errorx,2)+pow(errory,2));

	//cout << stateContinuous[0] << " -> " << stateDiscrete[0] << endl;
}

void Seek::setGoal(float xg, float yg)
{
	target.position.x = xg;
	target.position.y = yg;
	updateState();
}
