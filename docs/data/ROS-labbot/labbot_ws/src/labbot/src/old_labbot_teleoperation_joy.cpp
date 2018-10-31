// code based on the code available at:
// https://code.ros.org/svn/ros-pkg/stacks/joystick_drivers_tutorials/trunk/turtle_teleop/src/teleop_turtle_joy.cpp
// for WritinTeleopNode tutorial:
// http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

#include <ros/ros.h>
#include <labbot/msgToLabbot.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

class LabbotTeleoperation
{
	public:
		LabbotTeleoperation();

	private:
		void CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh;
		ros::Publisher msgToLabbotPublisher;
		ros::Subscriber joySubscriber;

		// int gamepadStcikLeftY, gamepadStcikRightY;
		int gamepadStickRightX, gamepadStickRightY;
		float motorRightScale, motorLeftScale;
		float motorScale, angularScale;
};

LabbotTeleoperation::LabbotTeleoperation()
{
	//gamepadStcikRightY = 1;
	//gamepadStcikLeftY = 4;
	gamepadStickRightX = 3;
	gamepadStickRightY = 4;

	this->motorRightScale = 40.0F;
	this->motorLeftScale = 40.0F;
	
	this->motorScale = 40.0F;
	this->angularScale = 1.0F;

	//nh.param("axis_linear", linear, linear);
	//nh.param("axis_angular", angular, angular);

	this->msgToLabbotPublisher = this->nh.advertise<labbot::msgToLabbot>("toLabbot", 10);

	this->joySubscriber = this->nh.subscribe<sensor_msgs::Joy>("joy", 10, &LabbotTeleoperation::CallbackJoy, this);
}

void LabbotTeleoperation::CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
	// get the data from gamepad
	float x = joy->axes[gamepadStickRightX];
	float y = joy->axes[gamepadStickRightY];

	// calculate angle and size of vector
	float angle = atan2(y, x);
	float power = sqrt(x*x + y*y);

	// set the speed
	float motorRightSpeed = y - x;
	float motorLeftSpeed = y + x;

	// scale the output
	motorRightSpeed = motorRightSpeed * motorRightScale;
	motorLeftSpeed = motorLeftSpeed * motorLeftScale;

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
	ros::init(argc, argv, "Labbot_teleopeartor_joy");
	LabbotTeleoperation teleoperator;

	ros::spin();
}
