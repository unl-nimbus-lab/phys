#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv)
{
  ros::NodeHandle nh;
  ros::Publisher pub;
  geometry_msgs::Twist cmdvel;
  pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);


  cmdvel.angular.z = 10;
  printf("sending...[%f, %f]\n", cmdvel.linear.x, cmdvel.angular.z);
  pub.publish(cmdvel);
}
/*
class RobotControl
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  geometory_msgs::Twist cmdvel;

public:
  RobotControl()
  {
    pub = n_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
  }

}
*/
