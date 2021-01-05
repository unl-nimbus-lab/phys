#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "chargingpile_tf_listener");
	ros::NodeHandle node;

	ros::Publisher robot_cmdvel_pub =
		node.advertise<geometry_msgs::Twist>("ctrl_cmd_vel", 10);

	tf::TransformListener listener;


	ros::Rate rate(10.0);
	while (node.ok())
	{
		tf::StampedTransform transform;
		try
		{
			ros::Time now = ros::Time::now();
			listener.waitForTransform("/charging_pile", "/robot",
					now, ros::Duration(3.0));
			listener.lookupTransform("/charging_pile", "/robot",
					now, transform);



			//			listener.lookupTransform("/charging_pile", "/robot", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		if(transform.getOrigin().x() > 0.001)
		{
			ROS_INFO_STREAM("xiworng==>transform.x .y .z .theta = " << transform.getOrigin().x() <<" " << transform.getOrigin().y() );	
			geometry_msgs::Twist vel_msg;
			vel_msg.angular.z = 10.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
			vel_msg.linear.x  = 0.5 * sqrt(pow(transform.getOrigin().x(), 2.0) + pow(transform.getOrigin().y(), 2.0));
			robot_cmdvel_pub.publish(vel_msg);
		}
		rate.sleep();
	}
	return 0;
}
