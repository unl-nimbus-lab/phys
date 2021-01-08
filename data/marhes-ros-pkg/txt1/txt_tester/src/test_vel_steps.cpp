#include "ros/ros.h"
#include "joy/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <vector>
#include <cmath>

class VelTest
{
public:
  VelTest(ros::NodeHandle nh);
private:
  ros::NodeHandle n_;
  ros::Time prev_time_;
	std::vector<double> lin_vel_array_, ang_vel_array_, duration_array_;
	uint32_t index_, index_max_;
	bool test_;
	
  ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
  ros::Timer tmr_;

  void joyCB(joy::Joy msg);
  void tmrCB(const ros::TimerEvent& e);
};

VelTest::VelTest(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  
  if (n_private.hasParam("lin_vel_array") && 
      n_private.hasParam("ang_vel_array") &&
      n_private.hasParam("duration_array"))
  {
    // Get the array
	  XmlRpc::XmlRpcValue lin_vel_array;
	  n_private.getParam("lin_vel_array", lin_vel_array);
	  if (lin_vel_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR("Error reading ang_vel_array.");
	  
	  // Get the array
	  XmlRpc::XmlRpcValue ang_vel_array;
	  n_private.getParam("ang_vel_array", ang_vel_array);
	  if (ang_vel_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR("Error reading ang_vel_array.");
	    
    // Get the array
    XmlRpc::XmlRpcValue duration_array;
	  n_private.getParam("duration_array", duration_array);
	  if (duration_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR("Error reading duration_array.");
	    
	  int size_lin, size_ang, size_duration;
		try
		{
			size_lin = lin_vel_array.size();
      size_ang = ang_vel_array.size();
			size_duration = duration_array.size();
			
			if ((size_lin != size_ang) && (size_lin != size_duration))
			{
				ROS_ERROR("Size of arrays do not match, exiting");
				exit(0);
			}
		} 
		catch (const XmlRpc::XmlRpcException e)
		{
			ROS_ERROR("No table available, exiting.");
			exit(0);
		}
		
	  for(int i=0; i < size_lin; i++)
		{
		  lin_vel_array_.push_back((double)lin_vel_array[i]);
		}

	  for(int i=0; i < size_ang; i++)
		{
		  ang_vel_array_.push_back((double)ang_vel_array[i]);
		}
		
		for(int i=0; i < size_duration; i++)
		{
		  duration_array_.push_back((double)duration_array[i]);
		}
  }
  else
  {
    ROS_ERROR("All arrays not given, exiting.");
    exit(0);
  }
  
  index_ = 0;
  index_max_ = lin_vel_array_.size();
  test_ = false;
  prev_time_ = ros::Time::now();

	vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel",1);
	joy_sub_ = n_.subscribe<joy::Joy>("joy", 10, &VelTest::joyCB, this);
  tmr_ = n_.createTimer(ros::Duration(0.1), &VelTest::tmrCB, this);
  ROS_INFO("Hello");
}

void VelTest::joyCB(joy::Joy msg)
{
  ROS_INFO("Joystick callback");
  if (msg.buttons[0] == 1)
    test_ = true;
  else
    test_ = false;
}

void VelTest::tmrCB(const ros::TimerEvent& e)
{
  //ROS_INFO("Timer callback");
  geometry_msgs::Twist vel_msg;
  ros::Duration dur = ros::Time::now() - prev_time_;
  //ROS_INFO("Time: %f", dur.toSec());
         
  if (test_)
  {
    if (dur.toSec() > duration_array_[index_])
    {
      index_++;
      prev_time_ = ros::Time::now();
      if (index_ >= index_max_)
        index_ = 0;
    }
    
    vel_msg.linear.x = lin_vel_array_[index_];
    vel_msg.angular.z = ang_vel_array_[index_];
  }
  else
  {
    prev_time_ = ros::Time::now();
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
  }
  
  vel_pub_.publish(vel_msg); 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Txt1VelTest");
	ros::NodeHandle n;
	
	VelTest * v = new VelTest(n); 
  ros::spin();

	return 0;
}
