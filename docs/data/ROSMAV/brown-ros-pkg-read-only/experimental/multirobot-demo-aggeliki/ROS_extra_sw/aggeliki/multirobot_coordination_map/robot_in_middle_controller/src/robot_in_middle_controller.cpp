#include "robot_in_middle_controller.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "coordination_server_info_fusion");
     ros::NodeHandle n1, n2, n3, n4, n5;
     vel_pub = n1.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
     pos_sub = n2.subscribe("/position", 1, positionCallback);
     pospub = n3.advertise<position_tracker::Position>("/position", 1); //SET QUEUE SIZE = 1!!! (keep only last msg)
     bump_sub = n4.subscribe("/sensorPacket", 1, bumperCallback);
     wifi_sub = n5.subscribe("/wifiNNs", 1, wifiCallback);       

	 //loadMap();

	//JUST FOR testing:
	position_tracker::Position *tmp_pos = new position_tracker::Position();
	tmp_pos->x = 9;
	tmp_pos->y = 13;
	tmp_pos->theta = 0;
	cur_pos = *tmp_pos;

     ros::spinOnce(); 
     //ros::spin();     
 
     handleRequests();
}

void reach_middle_point(list< boost::shared_ptr<position_tracker::Position> > nn_pos_list)
{
	if(nn_pos_list.size() < 2) //need to have 2 neighbors for the function to work
	{
		cout << "I have " << nn_pos_list.size() << " neighbors. I need 2." << endl;
		return;
	}

	ros::spinOnce(); //to get the most recent position of the robot

	//(1) find the nodes that are closest to the position of the neighbors 	
	loadMap();
	vector<Node *> closest_nodes;
	list< boost::shared_ptr<position_tracker::Position> >::iterator itl;
	int cnt = 0;
    for(itl = nn_pos_list.begin(); itl != nn_pos_list.end(); ++itl)
	{
		position_tracker::Position *pin = new position_tracker::Position();
		pin->x = (*itl)->x;
		pin->y = (*itl)->y;
		pin->theta = (*itl)->theta;
	
		std::cout << "Robot " << cnt << " in position (" << pin->x << ", " << pin->y << ", " << pin->theta << ")" << std::endl;

		//find the closest node np to the input position p
		double min_dist = 10000;
		int min_ind = -1;
		vector<Node *>::iterator itn;
		for(itn = nodes.begin(); itn != nodes.end(); ++itn)
		{
			double dist = sqrt(pow(pin->x - (*itn)->p->x, 2) + pow(pin->y - (*itn)->p->y, 2));			
			if(dist < min_dist)
			{
				min_dist = dist;
				min_ind = itn - nodes.begin();
			} 
		}
		closest_nodes.insert(closest_nodes.end(), nodes[min_ind]);
		
		std::cout << "It's closest position is (" << nodes[min_ind]->p->x << ", " << nodes[min_ind]->p->y << ")" << std::endl;
		cnt++;
	}

    //(2)For now: assume 2 neighbors, find the position in between the 2 neighbors 
	//shortest path between the 2 neighbors
    //vector<Node *> closest_nodes_vec;
	//std::copy(closest_nodes.begin(), closest_nodes.end(), closest_nodes_vec.begin());
	//closest_nodes_vec[0]->distanceFromStart = 0; // set start node
	closest_nodes[0]->distanceFromStart = 0; // set start node
	Dijkstras(); //NEED TO DO THAT ONLY ONCE IN THE BEGINNING, NOT ALL THE TIME!!!!!!!!

	/*vector<Node *>::iterator itw;
	for(itw = waypoints.begin(); itw != waypoints.end(); ++itw)
	{
		cout << "waypint id = " << (*itw)->id << endl;	
	} */

	waypoints.clear();
	PrintLoadShortestRouteTo(closest_nodes[1]); //stores the path in the waypoints vector => 	
	//PrintLoadShortestRouteTo(closest_nodes_vec[1]); //stores the path in the waypoints vector => NEED TO CHANGE THAT (I want to get the path back as a vector and then do whatever I want with that)
	//BUT, for now...

	//get position among 2 neighbors
	Node *middle_init = waypoints[ceil(waypoints.size()/2)];

	std::cout << "The position in the middle of the robots is (" << middle_init->p->x << ", " << middle_init->p->y << ")" << std::endl;

	//(3)Find the closest node nc to my current position
	loadMap(); //!!!!!!!!!!!!!!!
	vector<Node *>::iterator itn2;
	cout << "All the nodes: " << endl;
	for(itn2 = nodes.begin(); itn2 != nodes.end(); ++itn2)
	{
		cout << "node: id = " << (*itn2)->id << ", x = " << (*itn2)->p->x << ", y = " << (*itn2)->p->y << endl;
	}



	//find the middle node using the newly loaded map
	double min_dist = 10000;
	int min_ind = -1;
	vector<Node *>::iterator itn;
	for(itn = nodes.begin(); itn != nodes.end(); ++itn)
	{
		double dist = sqrt(pow(middle_init->p->x - (*itn)->p->x, 2) + pow(middle_init->p->y - (*itn)->p->y, 2));		
		if(dist < min_dist)
		{
			min_dist = dist;
			min_ind = itn - nodes.begin();
		} 
	}
	Node *middle = nodes[min_ind];

	//find closest node to the current position of the robot
	min_dist = 10000;
	min_ind = -1;
	//vector<Node *>::iterator itn;
	for(itn = nodes.begin(); itn != nodes.end(); ++itn)
	{
		double dist = sqrt(pow(cur_pos.x - (*itn)->p->x, 2) + pow(cur_pos.y - (*itn)->p->y, 2));		
		if(dist < min_dist)
		{
			min_dist = dist;
			min_ind = itn - nodes.begin();
		} 
	}
	Node *nc = nodes[min_ind];

	std::cout << "My closest position is (" << nc->p->x << ", " << nc->p->y << ")" << std::endl;

	//(4)Do shortest path between the current position of the robot and the position in between its 2 neighbors 
	nc->distanceFromStart = 0; // set start node
	Dijkstras();
	waypoints.clear();
	PrintLoadShortestRouteTo(middle); //THIS TIME, I want the generated path to be stored in the waypoints list

	//set your goal to be 3 nodes backwards from and follow the way to the goal (make your waypoints contain all the nodes up to the 3 nodes away from the destination)
	/*waypoints.pop_back();
	waypoints.pop_back();
	waypoints.pop_back();
     //loadWaypoints(); //NEED TO CHANGE THAT !!!!*/

	//======================= don't move for now =============================
	//EXECUTE THE PATH (probably need to move it somewhere else)
     vector<Node *>::iterator it;
     for(it = waypoints.begin(); it != waypoints.end(); ++it) //reach every single waypoint generated by the planner
     {
      
        goal_pos_x = (*it)->p->x;
        goal_pos_y = (*it)->p->y;

        //move to the next waypoint
        while(sqrt(pow(goal_pos_x - cur_pos.x, 2) + pow(goal_pos_y - cur_pos.y, 2)) > Tdist)
        {
		   ros::spinOnce();
           drive();
        }//end while(current goal is not reached)
     }//end while(1)*/
}

void drive()
{
   if(!ok_to_drive)
      return;

   cout << "CUR_POS.x = " << cur_pos.x << endl;
   cout << "CUR_POS.y = " << cur_pos.y << endl;
   cout << "CUR_POS.theta = " << cur_pos.theta << endl;
   cout << "goal_pos_x = " << goal_pos_x << endl;
   cout << "goal_pos_y = " << goal_pos_y << endl;

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
   
   geometry_msgs::Twist twist;
   twist.angular.z = 0.0;
   twist.linear.x = 0.0;

   if(fabs(theta_delta) > 0.5)
      twist.angular.z = copysign(0.5, theta_delta);
   else
      twist.angular.z = theta_delta;

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
   }
 
   vel_pub.publish(twist);
}



