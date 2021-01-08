#include "ml_follow_pioneer/waypoint_navigator.h"

WaypointNavigator::WaypointNavigator(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  
  if (n_private.hasParam("waypoints"))
  {
    XmlRpc::XmlRpcValue waypoints;
	  n_private.getParam("waypoints", waypoints);
	  if (waypoints.getType() != XmlRpc::XmlRpcValue::TypeArray)
	  {
		  ROS_FATAL("Error reading waypoints: Not an array.");
		  exit(0);
		}
		  
    int size;
		try
		{
			size = waypoints.size();
		} 
		catch (const XmlRpc::XmlRpcException e)
		{
			ROS_INFO("No table available, exiting.");
			exit(0);
		}
    ROS_INFO("Starting array.");
		// create qarray set
		for(int i = 0; i < size; i += 2)
		{
		  geometry_msgs::Point pt;
		  pt.x = waypoints[i]; pt.y = waypoints[i + 1]; pt.z = 0;
			waypoints_.push_back(pt);
			ROS_INFO("X: %f, Y: %f, Z: %f", pt.x, pt.y, pt.z);
		}
  }
  else
  {
    ROS_FATAL("Need to have the waypoints parameter.");
  }  
  
  n_private.param("lin_vel", lin_vel_, 0.5);
  n_private.param("look_ahead", look_ahead_, 1.0);
  
  next_waypoint_ = 1;
  last_waypoint_ = 0;
  
  cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  odom_sub_ = n_.subscribe<nav_msgs::Odometry>("/odom", 1, &WaypointNavigator::cb_odom, this);
  
}

void WaypointNavigator::cb_odom(nav_msgs::Odometry msg)
{
  double a, b, c, x1, x2, xc, y1, y2, yc, theta, exp, u1, u2, theta_err;
  geometry_msgs::Point p1, p2;
  geometry_msgs::Twist vel_msg;
  
  // If distance to next waypoint is less than look ahead switch to next waypoint
  while (distance(msg.pose.pose.position, waypoints_[next_waypoint_]) < look_ahead_)
  {
    // Go to next waypoint. If the the current waypoint is the last one go to the first waypoint
    last_waypoint_ = next_waypoint_;
    next_waypoint_++;
    if (next_waypoint_ >= waypoints_.size())
      next_waypoint_ = 0;
  }
  
  // From http://paulbourke.net/geometry/sphereline/  
  x1 = waypoints_[next_waypoint_].x;
  y1 = waypoints_[next_waypoint_].y;
  x2 = waypoints_[last_waypoint_].x;
  y2 = waypoints_[last_waypoint_].y;
  xc = msg.pose.pose.position.x;
  yc = msg.pose.pose.position.y;
  theta = tf::getYaw(msg.pose.pose.orientation);
  
  a = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  b = 2 * ((x2 - x1) * (x1 - xc) + (y2 - y1) * (y1 - yc));
  c = x1 * x1 + xc * xc + y1 * y1 + yc * yc - look_ahead_ * look_ahead_ - 2 * x1 * xc - 2 * y1 * yc;
  
  exp = b * b - 4 * a * c;
  // Line is outside circle
  if (exp < 0)
  {
    p1.x = x1;
    p1.y = y1;
    ROS_INFO("OUTSIDE AND NO INTERSECTION");
  }
  // Line is tangent to circle
  else if (exp == 0)
  {
    u1 = -b / (2 * a);
    p1.x = x1 + u1 * (x2 - x1);
    p1.y = y1 + u1 * (y2 - y1);
    ROS_INFO("TANGENT");
  }
  else
  {
    u1 = (-b + sqrt(exp)) / (2 * a);
    u2 = (-b - sqrt(exp)) / (2 * a);
    
    // No intersection but inside the circle
    if ((u1 > 1 && u2 < 0) || (u1 < 0 && u2 > 1))
    {
      ROS_INFO("No intersection but inside the circle");
    }
    // One intersection point
    else if (u1 >= 0 && u1 <= 1 && (u2 > 1 || u2 < 0)) 
    {
      p1.x = x1 + u1 * (x2 - x1);
      p1.y = y1 + u1 * (y2 - y1);
    }
    else if (u2 >= 0 && u2 <= 1 && (u1 > 1 || u1 < 0))
    {
      p1.x = x1 + u2 * (x2 - x1);
      p1.y = y1 + u2 * (y2 - y1);
    }
    // Decide between two intersection points
    else
    {
      p1.x = x1 + u1 * (x2 - x1);
      p1.y = y1 + u1 * (y2 - y1);
      p2.x = x1 + u2 * (x2 - x1);
      p2.y = y1 + u2 * (y2 - y1);
      
      if (distance(p1, waypoints_[next_waypoint_]) > distance(p2, waypoints_[next_waypoint_]))
      {
        p1 = p2;
      }
    }
  }
  
  //ROS_INFO("Point X: %f, Y: %f", p1.x, p1.y);
 
  //x_err = (p1.x - xc) * sin(theta) + (p1.y - yc) * cos(theta);
  //ROS_INFO("X_ERR: %f", x_err);
  theta_err = atan2(p1.y - yc, p1.x - xc) - theta;
  while (theta_err > M_PI)
    theta_err -= 2 * M_PI;
  while (theta_err < -M_PI)
    theta_err += 2 * M_PI;
    
  vel_msg.linear.x = lin_vel_;
  vel_msg.angular.z = lin_vel_ * 2 * theta_err / look_ahead_;
  cmd_vel_pub_.publish(vel_msg);
}

double WaypointNavigator::distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_navigator");
  ros::NodeHandle n;
  
  WaypointNavigator * wn = new WaypointNavigator(n);
  ros::spin();

  return 0;
}
