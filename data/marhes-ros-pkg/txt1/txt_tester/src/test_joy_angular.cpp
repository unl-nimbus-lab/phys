#include "ros/ros.h"
#include "joy/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <cmath>

class Txt1Teleop{
public:
	Txt1Teleop(ros::NodeHandle nh);
	virtual ~Txt1Teleop();
private:
	void joyCB(joy::Joy msg);
	void linVelCB(std_msgs::Float64 msg);
  void cmdVelTmrCB(const ros::TimerEvent& e);

	ros::NodeHandle n;
	ros::Subscriber joySub;
	ros::Subscriber linVelSub;
	ros::Publisher velPub;
	ros::Timer cmdVelTmr;
	joy::Joy joyMsg;

	int linAxis, angAxis;
	double linVel, lin_vel_max, ang_vel_max, radius_min;
	bool joyRxFlag;
};

Txt1Teleop::Txt1Teleop(ros::NodeHandle nh):n(nh)
{
	ros::NodeHandle n_private("~");

	n_private.param("linAxis", linAxis, 1);
	n_private.param("angAxis", angAxis, 0);
	n_private.param("linVelMax", lin_vel_max, 1.0);
	n_private.param("radiusMin", radius_min, 0.555);
	
	joyRxFlag = false;
	ang_vel_max = 0.0;

	velPub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	joySub = n.subscribe<joy::Joy>("joy", 10, &Txt1Teleop::joyCB, this);
	linVelSub = n.subscribe<std_msgs::Float64>("lin_vel", 10, &Txt1Teleop::linVelCB, this);
	cmdVelTmr = n.createTimer(ros::Duration(0.1), &Txt1Teleop::cmdVelTmrCB, this);
}


Txt1Teleop::~Txt1Teleop()
{

}

void Txt1Teleop::cmdVelTmrCB(const ros::TimerEvent& e)
{
  geometry_msgs::Twist vel;
	if (joyRxFlag && joyMsg.buttons[0] == 1)
  {
    if (linVel > lin_vel_max)
      linVel = lin_vel_max;
    else if (linVel < -lin_vel_max)
      linVel = -lin_vel_max;
    vel.linear.x = linVel;
    ang_vel_max = std::abs(linVel) / radius_min;
    vel.angular.z = ang_vel_max * joyMsg.axes[angAxis];
  }
  else
  {
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
  }
  velPub.publish(vel);
  ROS_INFO("Linear: %f; Angular: %f", vel.linear.x, vel.angular.z);
}

void Txt1Teleop::joyCB(joy::Joy msg)
{
	joyMsg = msg;
	joyRxFlag = true;
}

void Txt1Teleop::linVelCB(std_msgs::Float64 msg)
{
  linVel = msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Txt1Teleop");
	ros::NodeHandle n;

	Txt1Teleop *p = new Txt1Teleop(n);

  ros::spin();

	return 0;
}
