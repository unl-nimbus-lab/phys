// based on https://code.ros.org/svn/ros-pkg/stacks/joystick_drivers_tutorials/trunk/turtle_teleop/src/teleop_turtle_joy.cpp
// from tutorial http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <labbot/msgToLabbot.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

namespace labbot_teleoperation
{

class labbot_teleoperation_nodelet : public nodelet::Nodelet
{
  public:
    labbot_teleoperation_nodelet() {}
    ~labbot_teleoperation_nodelet() {}
    virtual void onInit()
    {
	ros::NodeHandle &nh=getNodeHandle(); 

	motorRightScale = 40;
	motorLeftScale = 40;

	msgToLabbotPublisher = nh.advertise<labbot::msgToLabbot>("toLabbot", 1);
	gamepadSubscriber = nh.subscribe<geometry_msgs::Twist>("Twist", 10, boost::bind(&labbot_teleoperation_nodelet::gamepadCallback, this, _1));
    }

  private:
    void gamepadCallback(const geometry_msgs::Twist::ConstPtr& twist)
    {

	float motorRightSpeed = 0;
	float motorLeftSpeed = 0;

	// get the data from gamepad
	float x = twist->linear.x;
	float y = twist->angular.z;

	// set the speed
	motorRightSpeed = x - y;
	motorLeftSpeed = x + y;

	// scale the output
	motorRightSpeed = motorRightSpeed * motorRightScale;
	motorLeftSpeed = motorLeftSpeed * motorLeftScale;

	//check if over scale
	if(motorRightSpeed > motorRightScale)
	{
	  motorRightSpeed = motorRightScale;
	}

	if(motorLeftSpeed > motorLeftScale)
	{
	  motorLeftSpeed = motorLeftScale;
	}

	// fill the msg
	labbot::msgToLabbot msg;
	msg.motorRightSpeed = motorRightSpeed;
	msg.motorLeftSpeed = motorLeftSpeed;
	// publish the message
	msgToLabbotPublisher.publish(msg);
    }
	
    float motorRightScale, motorLeftScale;
    ros::Publisher msgToLabbotPublisher;
    ros::Subscriber gamepadSubscriber;  
};

}
PLUGINLIB_EXPORT_CLASS(labbot_teleoperation::labbot_teleoperation_nodelet, nodelet::Nodelet);
