// code based on the code available at:
// https://code.ros.org/svn/ros-pkg/stacks/joystick_drivers_tutorials/trunk/turtle_teleop/src/teleop_turtle_joy.cpp
// for WritinTeleopNode tutorial:
// http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

#include <ros/ros.h>
#include <labbot/msgToLabbot.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class LabbotTeleoperation
{
	public:
		LabbotTeleoperation();

	private:
		void CallbackTwist(const geometry_msgs::Twist::ConstPtr& twist);

		ros::NodeHandle nh;
		ros::Publisher msgToLabbotPublisher;
		ros::Subscriber twistSubscriber;
		
		float motorRightScale, motorLeftScale;
		float motorScale, angularScale;
};

LabbotTeleoperation::LabbotTeleoperation()
{
	this->motorRightScale = 40.0F;
	this->motorLeftScale = 40.0F;
	
	this->motorScale = 40.0F;
	this->angularScale = 1.0F;

	this->msgToLabbotPublisher = this->nh.advertise<labbot::msgToLabbot>("toLabbot", 10);
	this->twistSubscriber = this->nh.subscribe<geometry_msgs::Twist>("Twist", 10, &LabbotTeleoperation::CallbackTwist, this);
}

void LabbotTeleoperation::CallbackTwist(const geometry_msgs::Twist::ConstPtr& twist)
{
	// get the data from the gamepad
	float x = twist->linear.x;
	float y = twist->angular.z;

	// set the speeds
	
	// first option:
	// float motorRightSpeed = x - y;
	// float motorLeftSpeed = x + y;
	// scale the output
	// motorRightSpeed = motorRightSpeed * motorRightScale;
	// motorLeftSpeed = motorLeftSpeed * motorLeftScale;
	
	// second option:
	// calculate angle and size of vector
	float angle = atan2(y, x);
	float power = sqrt(x*x + y*y);
	// ??
	
	// third option:
	float motorRightSpeed = x*motorScale + y*motorScale*angularScale;
	float motorLeftSpeed = x*motorScale - y*motorScale*angularScale;

	// check if over scale
	if(motorRightSpeed > motorRightScale)
	{
		motorRightSpeed = motorRightScale;
	}
	else if(motorRightSpeed < -motorRightScale)
	{
		motorRightSpeed = -motorRightScale;
	}
	if(motorLeftSpeed > motorLeftScale)
	{
		motorLeftSpeed = motorLeftScale;
	}
	else if(motorLeftSpeed < -motorLeftScale)
	{
		motorLeftSpeed = -motorLeftScale;
	}

	// fill the msg
	labbot::msgToLabbot msg;
	msg.motorRightSpeed = motorRightSpeed;
	msg.motorLeftSpeed = motorLeftSpeed;
	// publish the message
	msgToLabbotPublisher.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "labbot_teleopeartor_twist_node");
	LabbotTeleoperation teleoperator;

	ros::spin();
}
