#include "step_navigator.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "step_navigator");
	 ros::NodeHandle n2;
	 n1 = new ros::NodeHandle();	 
     vel_pub = n1->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
     pos_sub = n2.subscribe("/position", 1, positionCallback);

     n1->param("step_navigator/ok_to_drive", ok_to_drive, 1);

/*	//JUST FOR testing:
	position_tracker::Position *tmp_pos = new position_tracker::Position();
	tmp_pos->x = 9;
	tmp_pos->y = 13;
	tmp_pos->theta = 0;
	cur_pos = *tmp_pos;*/

     //ros::spinOnce(); 
     ros::spin();     
}

void positionCallback(const position_tracker::PositionConstPtr& msg)
{
   cur_pos = *msg;

   //query the param server whether you are ready to go
   n1->getParam("step_navigator/ok_to_drive", ok_to_drive);

   //cout << "ok_to_drive = " << ok_to_drive << endl;	

   if(!ok_to_drive)
      return;

   //if no goal position is set, stay in the current position
   position_tracker::Position goal_pos;
   n1->param("step_navigator/goal_pos_x", goal_pos_x, cur_pos.x);
   n1->param("step_navigator/goal_pos_y", goal_pos_y, cur_pos.y);

/*   cout << "CUR_POS.x = " << cur_pos.x << endl;
   cout << "CUR_POS.y = " << cur_pos.y << endl;
   cout << "CUR_POS.theta = " << cur_pos.theta << endl;
   cout << "goal_pos_x = " << goal_pos_x << endl;
   cout << "goal_pos_y = " << goal_pos_y << endl;*/

   double x_delta = goal_pos_x - cur_pos.x;
   double y_delta = goal_pos_y - cur_pos.y; 
   
   //double m = (cur_pos_y - goal_pos_y)/(cur_pos_x - goal_pos_x);
   double goal_theta = atan2(y_delta, x_delta); // use math.h for atan (then make sure it's in [-pi, pi] range

   double theta_delta = goal_theta - cur_pos.theta;
   if(theta_delta > PI)
   {
       theta_delta = -2*PI + theta_delta;
   }
   else if(theta_delta < -PI)
   {
       theta_delta = 2*PI + theta_delta;
   }

   double goal_distance = sqrt(pow(x_delta, 2) + pow(y_delta, 2));

   //if you are so close to the goal, don't rotate at all
   if(goal_distance <= 0.1)
   {
       theta_delta = 0;
   }

   //cout << "goal_distance = " << goal_distance << endl;	   

   geometry_msgs::Twist twist;
   twist.angular.z = 0.0;
   twist.linear.x = 0.0;

   if(fabs(theta_delta) > 0.5)
      twist.angular.z = copysign(0.5, theta_delta);
   else 
      twist.angular.z = theta_delta;

   //if the rotation angle is big, just rotate, don't move forwards
   if(fabs(theta_delta) > 0.1)
   {
      twist.linear.x = 0.0;
   }
   else if(goal_distance > 0.25)
   {
      twist.linear.x = 0.25;  //0.05 for DEBUGGING!!!, 0.5 for max sustainable speed
   }
   else if(goal_distance > 0.1)
   {
      twist.linear.x = goal_distance; //0.05 for DEBUGGING!!!
   }
   else
   {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;

	  //goal position was reached, ask for the next goal position
	  ros::ServiceClient client = n1->serviceClient<path_navigator::getNextWaypoint>("get_next_waypoint");
	  path_navigator::getNextWaypoint srv;
	  if(!client.call(srv))
      {
          cout << "Failed to call service getNextWaypoint" << endl;
      }

   }
 
   /*cout << "Publish twist (" << twist.linear.x << ", " << twist.angular.z << ") from step_navigator" << endl;
   cout << "Goal = (" << goal_pos_x << ", " << goal_pos_y << ") and cur_pos = (" << cur_pos.x << ", " << cur_pos.y << ")" << endl;*/
   vel_pub.publish(twist);
}


