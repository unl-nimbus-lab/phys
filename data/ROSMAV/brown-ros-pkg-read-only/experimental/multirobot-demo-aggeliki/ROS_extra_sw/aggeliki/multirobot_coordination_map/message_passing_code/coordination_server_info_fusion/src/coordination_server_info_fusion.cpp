#include "coordination_server_info_fusion.h"

class Node
{M
public:
	Node(int id, double x, double y) 
		: id(id), previous(NULL), distanceFromStart(INT_MAX)
	{
        p = new position_tracker::Position();
        p->x = x; 
        p->y = y;
		nodes.push_back(this);
	}
public:
	int id;
	Node* previous;
	int distanceFromStart;
    position_tracker::Position *p;
};


class Edge
{
public:
	Edge(Node* node1, Node* node2, int distance) 
		: node1(node1), node2(node2), distance(distance)
	{
		edges.push_back(this);
	}
	bool Connects(Node* node1, Node* node2)
	{
		return (
			(node1 == this->node1 &&
			node2 == this->node2) ||
			(node1 == this->node2 && 
			node2 == this->node1));
	}
public:
	Node* node1;
	Node* node2;
	int distance;
};

class Line{
  public:
   double Ax;
   double Ay;
   double Bx;
   double By;
   double theta; //slope in the (x,y) coordinate system

  public:
   Line(double cAx, double cAy, double cBx, double cBy, double ctheta)
   {
       Ax = cAx;
       Ay = cAy;
       Bx = cBx;
       By = cBy;
       theta = ctheta;
   }
};


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

void handleRequests()
{
     //receive message from client
     int sockfd, newsockfd, portno, clilen;
     char buffer[BUFSIZ];
     struct sockaddr_in serv_addr, cli_addr;
     int n;

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = 50060;nn
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
	 int optval = 1;
     setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);
     if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
          error("ERROR on binding");
     listen(sockfd,5); //can accomodate up to 5 requests at the same time
     clilen = sizeof(cli_addr);
     
	 while(1) //receive messages from neighbors and process
	 {
		 newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t *)&clilen);
		 if (newsockfd < 0) 
		      error("ERROR on accept");
		 bzero(buffer, BUFSIZ);

		 n = read(newsockfd,buffer,BUFSIZ);
		 if (n < 0) error("ERROR reading from socket");

		 //get the IP of the client
		 char *client_ip = inet_ntoa(cli_addr.sin_addr);
		 string client_ip_str = char_to_string(client_ip);

	 	 ros::spinOnce(); //to get the most recent position of the robot

		 //check whether the received message is an "ok_to_move" command or a /position message
		 /*ros::Message *rosmsg = new ros::Message();
		 rosmsg->deserialize((uint8_t *)buffer);
         string strmsg = rosmsg->__s_getDataType();*/
		 if(strcmp(buffer, "ok_to_move") == 0)
		 {		
			 cout << "received ok_to_move msg" << endl;
			 ok_to_move = true;
		 }
		 else{
			 //deserialize received position message
			 boost::shared_ptr<position_tracker::Position> newpos(new position_tracker::Position());            
		     newpos->deserialize((uint8_t *)buffer);

			 cout << "received position = (" << newpos->x << "," << newpos->y << "," << newpos->theta << ") ";
			 printf("from %s\n", client_ip);

			 //associate received position message with the corresponding neighbor/sender (just keep the last message received by the neighbor with client_ip)
             cout << "wn.count = " << wn.count << endl;
             cout << "wn.neighbors.size = " << wn.neighbors.size() << endl;

			 if(wn.count != 0)
			 {
				nn_pos.erase(client_ip_str);
	            cout << "CURRENT nn_pos.size() = " << nn_pos.size() << endl;
				nn_pos.insert(pair<string, boost::shared_ptr<position_tracker::Position> >(client_ip_str, newpos));				 
			 }

			 if(ok_to_move)
			 {
				 //get the current positions of your neighbors and determine your new position so that you are in the middle of them
				 list< boost::shared_ptr<position_tracker::Position> > nn_pos_list;
				 vector<batman_mesh_info::WifiNN>::iterator it;
				 for(it = wn.neighbors.begin(); it != wn.neighbors.end(); ++it) 
				 {
					 /*char *tmp_ip = new char[strlen((*it).ip.c_str())];
					 strcpy(tmp_ip, (*it).ip.c_str());
					 cout << "(*it).ip = " << (*it).ip << endl;
					 printf("tmp_ip = %s\n", tmp_ip);*/

					 mapType::iterator iter1;
					 for(iter1 = nn_pos.begin(); iter1 != nn_pos.end(); ++iter1)
					 {
						 cout << "iter1->first = " << iter1->first << endl;
						 cout << "iter1->second->x = " << iter1->second->x << endl;
					 }

					 mapType::iterator iter = nn_pos.find((*it).ip);
					 if(iter != nn_pos.end())
					 {
						 nn_pos_list.push_back(iter->second);
						 cout << "inserting neighbors..." << endl;
					 }
					 //delete []tmp_ip;
				 }
				
				 cout << "nn_pos.size() = " << nn_pos.size() << endl;
				 std::cout << "nn_pos_list.size() = " << nn_pos_list.size() << std::endl;

				 reach_middle_point(nn_pos_list);
				 //ok_to_move = false; //in case I need an ok_to_move message to make me move again
			 }
		 }

		 close(newsockfd);
	 }

     close(sockfd);

}

void wifiCallback(const batman_mesh_info::WifiNNsConstPtr& msg)
{
	 //read all the neighbors in terms of wifi
	 wn = *msg;	 

	cout << "WIFI message read" << endl;
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

void loadMap()
{
    //note: ordering of points when adding edges: from left to right, from bottom to top
	map4.clear();
	nodes.clear();
	edges.clear();

	/*ifstream ifs("./lines_map.txt");
	string line;
	if(!ifs)
	{
		cerr << "Error: file could not be opened" << endl;
		exit(1);
	}
	
	while(!ifs.eof())
	{
		getline(ifs, line);
		cout << "[" << line << "]" << endl;
		//TOKENIZE STRINGS HERE !!!!!!!		
		//...
		//...
		//...	
	}
	ifs.close();*/

    //edge AB
	Line *le1 = new Line(7.45, 13.67, 14.56, 13.67, 0);
    map4.push_back(le1);

    //edge BC
	Line *le2 = new Line(14.56, 13.67, 14.56, 14.38, PI/2);
    map4.push_back(le2);

    //edge DE
	Line *le3 = new Line(6.75, 12.55, 14.56, 12.55, 0);
    map4.push_back(le3);

    //edge FE
	Line *le4 = new Line(14.56, 6.43, 14.56, 12.55, PI/2);
    map4.push_back(le4);

    //edge HG
	Line *le5 = new Line(15.68, 7.54, 15.68, 13.84, PI/2);
    map4.push_back(le5);

    //edge FM
	Line *le6 = new Line(14.56, 6.43, 33.26, 6.43, 0);
    map4.push_back(le6);

    //edge HI
	Line *le7 = new Line(15.68, 7.54, 25.5, 7.54, 0);
    map4.push_back(le7);

    //edge JK
	Line *le8 = new Line(26.5, 7.54, 32.16, 7.54, 0);
    map4.push_back(le8);

    //edge KL
	Line *le9 = new Line(32.16, 7.54, 32.16, 17.24, PI/2);
    map4.push_back(le9);

    //edge MN
	Line *le10 = new Line(33.26, 6.43, 33.26, 18.04, PI/2);
    map4.push_back(le10);
	
   //INPUT THE MAP IN THE FORM OF NODES
    //SOS: id's of nodes should be in the order the nodes are created, they should start from 0
	//--------- corner nodes ----------
	Node* n1 = new Node(0, 7, 13);
	Node* n2 = new Node(1, 15, 13);
	Node* n3 = new Node(2, 15, 7);
	Node* n4 = new Node(3, 33, 7);
	Node* n5 = new Node(4, 32.8, 16);

	//-- intermediate nodes, corridor 1 --
	Node* n6 = new Node(5, 7.5, 13);
	Node* n7 = new Node(6, 8, 13);
	Node* n8 = new Node(7, 8.5, 13);
	Node* n9 = new Node(8, 9, 13);
	Node* n10 = new Node(9, 9.5, 13);
	Node* n11 = new Node(10, 10, 13);
	Node* n12 = new Node(11, 10.5, 13);
	Node* n13 = new Node(12, 11, 13);
	Node* n14 = new Node(13, 11.5, 13);
	Node* n15 = new Node(14, 12, 13);
	Node* n16 = new Node(15, 12.5, 13);
	Node* n17 = new Node(16, 13, 13);
	Node* n18 = new Node(17, 13.5, 13);
	Node* n19 = new Node(18, 14, 13);
	//Node* n20 = new Node(7, 14.5, 13);

	//-- intermediate nodes, corridor 2 --
	Node* n20 = new Node(19, 15, 11.5);
	Node* n21 = new Node(20, 15, 10);
	Node* n22 = new Node(21, 15, 8.5);

	//-- intermediate nodes, corridor 3 (DON'T CARE FOR NOW) --
	Node* n23 = new Node(22, 16.5, 7);
	Node* n24 = new Node(23, 18, 7);
	Node* n25 = new Node(24, 19.5, 7);
	Node* n26 = new Node(25, 21, 7);
	Node* n27 = new Node(26, 22.5, 7);
	Node* n28 = new Node(27, 24, 7);

/*	//Edge* e1 = new Edge(n1, n2, 1); //NEED to reset the distance of the edge
	Edge* e2 = new Edge(n2, n3, 2);
	Edge* e3 = new Edge(n3, n4, 2);
	Edge* e4 = new Edge(n4, n5, 1);
	Edge* e5 = new Edge(n1, n6, 1);
	Edge* e6 = new Edge(n6, n7, 1);
	//Edge* e7 = new Edge(n7, n8, 1);
	Edge* e8 = new Edge(n7, n2, 1);*/

	Edge* e1 = new Edge(n1, n6, 1); //NEED to reset the distance of the edge
	Edge* e2 = new Edge(n6, n7, 1);
	Edge* e3 = new Edge(n7, n8, 1);
	Edge* e4 = new Edge(n8, n9, 1);

	Edge* e18 = new Edge(n9, n10, 1);
	Edge* e19 = new Edge(n10, n11, 1);
	Edge* e20 = new Edge(n11, n12, 1);
	Edge* e21 = new Edge(n12, n13, 1);
	Edge* e22 = new Edge(n13, n14, 1);
	Edge* e23 = new Edge(n14, n15, 1);
	Edge* e24 = new Edge(n15, n16, 1);
	Edge* e25 = new Edge(n16, n17, 1);
	Edge* e26 = new Edge(n17, n18, 1);
	Edge* e27 = new Edge(n18, n19, 1);
    
	Edge* e5 = new Edge(n19, n2, 1);
	Edge* e6 = new Edge(n2, n20, 1);
	Edge* e7 = new Edge(n20, n21, 1);
	Edge* e8 = new Edge(n21, n22, 1);
	Edge* e9 = new Edge(n22, n3, 1);
	Edge* e10 = new Edge(n3, n23, 1);
	Edge* e11 = new Edge(n23, n24, 1);
	Edge* e12 = new Edge(n24, n25, 1);
	Edge* e13 = new Edge(n25, n26, 1);
	Edge* e14 = new Edge(n26, n27, 1);
	Edge* e15 = new Edge(n27, n28, 1);
	Edge* e16 = new Edge(n28, n4, 1);
	Edge* e17 = new Edge(n4, n5, 1);

}

void loadWaypoints()
{
    //HERE: path planning algo will produce path (inputs: map, initial location, end location; outputs: list me waypoints to follow one by one)

   //FOR NOW: I create list me ta waypoints...
   /*position_tracker::Position *p1 = new  position_tracker::Position();
   p1->x = 15;
   p1->y = 13;  
   waypoints.push_back(p1); 
   position_tracker::Position *p2 = new  position_tracker::Position();
   p2->x = 15;
   p2->y = 7;  
   waypoints.push_back(p2);		
   position_tracker::Position *p3 = new  position_tracker::Position();
   p3->x = 33;
   p3->y = 7;  
   waypoints.push_back(p3);
   position_tracker::Position *p4 = new  position_tracker::Position();
   p4->x = 32.8;
   p4->y = 16;  
   waypoints.push_back(p4);
   waypoints.push_back(p3); 
   waypoints.push_back(p2); 
   waypoints.push_back(p1); 
   position_tracker::Position *p5 = new  position_tracker::Position();
   p5->x = 6;
   p5->y = 13;  
   waypoints.push_back(p5); */
 
   //DijkstrasTest();
   
}

void bumperCallback(const irobot_create_2_1::SensorPacketConstPtr& msg)
{
   cur_sensors = *msg;

    //we have recovered from bumping         
    if(!(cur_sensors.bumpLeft || cur_sensors.bumpRight))
    {
        if(!ok_to_drive)
        {
            ros::NodeHandle nn;
            ros::ServiceClient client = nn.serviceClient<position_tracker::SetPosition>("set_position");
            position_tracker::SetPosition srv;            
            srv.request.x = latestBumpPos.x;
            srv.request.y = latestBumpPos.y;
            srv.request.theta = latestBumpPos.theta; //cur_pos.theta + 0.5; //pos.theta + 0.15; //0.15: concluded from experiments

            if(!client.call(srv))
            {
                ROS_ERROR("Failed to call service setPosition");
            }

            ok_to_drive = 1;
            prevBump = 0;
        }
        
        return;
    }


    ok_to_drive = 0;
   geometry_msgs::Twist twist;
   twist.angular.z = 0.0;
   twist.linear.x = 0.0;

   //we've hit something hard or head-on.  Back away slowly.
   if(cur_sensors.bumpLeft && cur_sensors.bumpRight)
   {
      twist.linear.x = -0.1; //-0.1;
      prevBump = 1;
   }
   else if(cur_sensors.bumpLeft)
   {   
      twist.angular.z = -0.1; //-0.1;
      if(!prevBump)
      {
            latestBumpPos = getClosestLeftLineProjection();
            //pospub.publish(pos);
            cout << "BUMPED LEFT:" << endl;
            cout << "latestBumpPos.x = " << latestBumpPos.x << endl;
            cout << "latestBumpPos.y = " << latestBumpPos.y << endl;
            cout << "latestBumpPos.theta = " << latestBumpPos.theta << endl;
      }
      prevBump = 1;
   }
   else if(cur_sensors.bumpRight)
   {
      twist.angular.z = 0.1;
      if(!prevBump)
      {
            latestBumpPos = getClosestRightLineProjection();
            //pospub.publish(pos);
            cout << "BUMPED RIGHT:" << endl;
            cout << "latestBumpPos.x = " << latestBumpPos.x << endl;
            cout << "latestBumpPos.y = " << latestBumpPos.y << endl;
            cout << "latestBumpPos.theta = " << latestBumpPos.theta << endl;

      }
      prevBump = 1;
   }

   vel_pub.publish(twist);
}

//in positionhandler function
void positionCallback(const position_tracker::PositionConstPtr& msg)
{
   if(!ok_to_drive)
      return;

   cur_pos = *msg;
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

position_tracker::Position getClosestLeftLineProjection()
{

   //WHAT IF THERE IS NO LINE TO THE LEFT OF THE ROBOT???

   double min_dist = 100000000;
   position_tracker::Position *tmp_pos = new position_tracker::Position();
   tmp_pos->x = -1;
   tmp_pos->y = -1;
   tmp_pos->theta = -1;

   list<Line *>::iterator i;   
   for(i = map4.begin(); i != map4.end(); ++i) //for each edge in the map
   {
      Line *e = *i;
      double D[2][2] = {{(e->Bx - e->Ax), (e->By - e->Ay)}, {(cur_pos.x - e->Ax), (cur_pos.y - e->Ay)}};

      /*cout << "D[0][0] = " << D[0][0] << endl;
      cout << "D[0][1] = " << D[0][1] << endl;
      cout << "D[1][0] = " << D[1][0] << endl;
      cout << "D[1][1] = " << D[1][1] << endl;*/

      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e->Ax = " << e->Ax << endl;
      cout << "e->Ay = " << e->Ay << endl;
      cout << "e->Bx = " << e->Bx << endl;
      cout << "e->By = " << e->By << endl;
      cout << "e->theta = " << e->theta << endl;
      cout << "det(D) = " << det(D) << endl;


      //keep only the lines to the left of the current robot pos
      if((det(D) < 0 && fabs(det(D)) > 0.0001 && (cur_pos.theta > e->theta - PI/2 && cur_pos.theta < e->theta + PI/2)) || (det(D) > 0  && fabs(det(D)) > 0.0001 && (cur_pos.theta < e->theta - PI/2 || cur_pos.theta > e->theta + PI/2)))
      {
          //find the projection of the current point to the last identified left line
          double A[2][2] = {{(e->Bx - e->Ax), (e->By - e->Ay)}, {(e->Ay - e->By), (e->Bx - e->Ax)}};
          double X[2] = {0, 0};
          double B[2] = {cur_pos.x*(e->Bx - e->Ax) + cur_pos.y*(e->By - e->Ay), e->Ay*(e->Bx - e->Ax) - e->Ax*(e->By - e->Ay)};
          solve(A, B, X);

            
          //discard line segment if the projection falls outside its boundaries
          if(e->theta < 0.1 ) //horizontal line
          {
              if(X[0] > e->Bx || X[0] < e->Ax)
                 continue;
          }
		  else{ //vertical line
              if(X[1] > e->By || X[1] < e->Ay)
                 continue; 

          }
          

      cout << "line to the LEFT of the robot" << endl;
/*      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e->Ax = " << e->Ax << endl;
      cout << "e->Ay = " << e->Ay << endl;
      cout << "e->Bx = " << e->Bx << endl;
      cout << "e->By = " << e->By << endl;
      cout << "e->theta = " << e->theta << endl;*/

          //calculate the distance between the projection and the current robot location
          double sdist = sqrt(pow(cur_pos.x - X[0], 2) + pow(cur_pos.y - X[1], 2));

          //keep the projection with the min distance from the current pos
          if(sdist < min_dist)
          {
              min_dist = sdist;
              tmp_pos->x = X[0] + (0.05/sdist)*(cur_pos.x - X[0]); //X[0]; //15 cm away from the wall may seem hard, but it handles the doors in the building
              tmp_pos->y = X[1] + (0.05/sdist)*(cur_pos.y - X[1]); //X[1];


			  double angle1 = e->theta;
			  double angle2 = e->theta + PI;
              if(angle1 > PI)
                   angle1 = -2*PI + angle1;
              else if(angle1 < -PI)
                   angle1 = 2*PI + angle1;

              if(angle2 > PI)
                   angle2 = -2*PI + angle2;
              else if(angle2 < -PI)
                   angle2 = 2*PI + angle2;

              double diff1 = angle1 - cur_pos.theta;
              double diff2 = angle2 - cur_pos.theta;

              if(fabs(diff1) < fabs(diff2))
                   tmp_pos->theta = angle1; 
              else
                   tmp_pos->theta = angle2; 

      cout << "angle1 = " << angle1 << endl;
      cout << "angle2 = " << angle2 << endl;
      cout << "tmp_pos->theta = " << tmp_pos->theta << endl;


      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e->Ax = " << e->Ax << endl;
      cout << "e->Ay = " << e->Ay << endl;
      cout << "e->Bx = " << e->Bx << endl;
      cout << "e->By = " << e->By << endl;
      cout << "e->theta = " << e->theta << endl;
      cout << "det(D) = " << det(D) << endl;

          }
      }
   }
  
   return *tmp_pos;

}

position_tracker::Position getClosestRightLineProjection()
{

   //WHAT IF THERE IS NO LINE TO THE RIGHT OF THE ROBOT???

   double min_dist = 100000000;
   position_tracker::Position *tmp_pos = new position_tracker::Position();
   tmp_pos->x = -1;
   tmp_pos->y = -1;
   tmp_pos->theta = -1;

   list<Line *>::iterator i;   
   for(i = map4.begin(); i != map4.end(); ++i) //for each edge in the map
   {
      Line *e = *i;
      double D[2][2] = {{(e->Bx - e->Ax), (e->By - e->Ay)}, {(cur_pos.x - e->Ax), (cur_pos.y - e->Ay)}};

/*      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e->Ax = " << e->Ax << endl;
      cout << "e->Ay = " << e->Ay << endl;
      cout << "e->Bx = " << e->Bx << endl;
      cout << "e->By = " << e->By << endl;
      cout << "e->theta = " << e->theta << endl;
      cout << "det(D) = " << det(D) << endl;*/

      //keep only the lines to the right of the current robot pos
      if((det(D) > 0  && fabs(det(D)) > 0.0001 && (cur_pos.theta > e->theta - PI/2 && cur_pos.theta < e->theta + PI/2)) || (det(D) < 0  && fabs(det(D)) > 0.0001 && (cur_pos.theta < e->theta - PI/2 || cur_pos.theta > e->theta + PI/2)))
      {
          //find the projection of the current point to the last identified left line
          double A[2][2] = {{(e->Bx - e->Ax), (e->By - e->Ay)}, {(e->Ay - e->By), (e->Bx - e->Ax)}};
          double X[2] = {0, 0};
          double B[2] = {cur_pos.x*(e->Bx - e->Ax) + cur_pos.y*(e->By - e->Ay), e->Ay*(e->Bx - e->Ax) - e->Ax*(e->By - e->Ay)};
          solve(A, B, X);

          //discard line segment if the projection falls outside its boundaries
          if(e->theta < 0.1 ) //horizontal line
          {
              if(X[0] > e->Bx || X[0] < e->Ax)
                 continue;
          }
		  else{ //vertical line
              if(X[1] > e->By || X[1] < e->Ay)
                 continue; 
          }

          cout << "line to the RIGHT of the robot" << endl;

          //calculate the distance between the projection and the current robot location
          double sdist = pow(cur_pos.x - X[0], 2) + pow(cur_pos.y - X[1], 2);

          //keep the projection with the min distance from the current pos
          if(sdist < min_dist)
          {
              min_dist = sdist;
              tmp_pos->x = X[0] + (0.05/sdist)*(cur_pos.x - X[0]); //X[0];
              tmp_pos->y = X[1] + (0.05/sdist)*(cur_pos.y - X[1]); //X[1];


			  double angle1 = e->theta;
			  double angle2 = e->theta + PI;
              if(angle1 > PI)
                   angle1 = -2*PI + angle1;
              else if(angle1 < -PI)
                   angle1 = 2*PI + angle1;

              if(angle2 > PI)
                   angle2 = -2*PI + angle2;
              else if(angle2 < -PI)
                   angle2 = 2*PI + angle2;

              double diff1 = angle1 - cur_pos.theta;
              double diff2 = angle2 - cur_pos.theta;

              if(fabs(diff1) < fabs(diff2))
                   tmp_pos->theta = angle1; 
              else
                   tmp_pos->theta = angle2; 

      cout << "angle1 = " << angle1 << endl;
      cout << "angle2 = " << angle2 << endl;
      cout << "tmp_pos->theta = " << tmp_pos->theta << endl;

      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e->Ax = " << e->Ax << endl;
      cout << "e->Ay = " << e->Ay << endl;
      cout << "e->Bx = " << e->Bx << endl;
      cout << "e->By = " << e->By << endl;
      cout << "e->theta = " << e->theta << endl;
      cout << "det(D) = " << det(D) << endl;

          }
      }
   }

   return *tmp_pos;

}

// det = ad - bc
double det(double A[2][2])
{
   return(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
}

// D = 1/det(A)
//
// .............. | d -b | . | D*d -D*b |
// inv = D * |-c a | = |-D*c D*a |
void inv(double A[2][2], double IA[2][2])
{
   double D = 1/det(A);
   IA[0][0] = +D*A[1][1];
   IA[0][1] = -D*A[0][1];
   IA[1][0] = -D*A[1][0];
   IA[1][1] = +D*A[0][0];
}

// |a b| |x| . |e|
// |c d| |y| = |f|
//
// |x| . |a b|-1 |e|
// |y| = |c d| .. |f|
void solve(double A[2][2], double C[2], double S[2])
{
 double IA[2][2];
 inv(A, IA);
 S[0] = IA[0][0]*C[0] + IA[0][1]*C[1];
 S[1] = IA[1][0]*C[0] + IA[1][1]*C[1];
}

void substring(const char* text, int start, int stop, char *new_string)
{
    sprintf(new_string, "%.*s", stop - start, &text[start]);
}

string char_to_string(char *input_p)
{
    string str(input_p);
    return str;
}

/*void Tokenize(const string& str, vector<string>& tokens, const string& delimiters=" ")
{
	//skip delimiters at beginning
	string::size_type lastPos = str.find_first_not_of(delimiters, 0);

	//find first "non-delimiter"
	string::size_type pos = str.find_first_of(delimiters, lastPos);

	while(string::npos != pos || string::npos != lastPos)
	{
		//found a token, add it to the vector
		tokens.push_back(str.substr(lastPos, pos - lastPos));

		//skip delimiters
		lastPos = str.find_first_not_of(delimiters, pos);

		//find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);					
	}
}*/


void error(char *msg)
{
    perror(msg);
    exit(1);
}

//------------- Dijkstra related code ----------------

void DijkstrasTest()
{
	Node* n1 = new Node('1', 15, 13);
	Node* n2 = new Node('2', 15, 7);
	Node* n3 = new Node('3', 33, 7);
	Node* n4 = new Node('4', 32.8, 16);
	Node* n5 = new Node('5', 6, 13);
	//Node* f = new Node('6', 0, 0);
	//Node* g = new Node('7', 0, 0);

	Edge* e1 = new Edge(n1, n2, 1); //NEED to reset the distance of the edge
	Edge* e2 = new Edge(n2, n3, 2);
	Edge* e3 = new Edge(n3, n4, 2);
	Edge* e4 = new Edge(n4, n5, 1);
	/*Edge* e5 = new Edge(b, f, 3);
	Edge* e6 = new Edge(c, e, 3);
	Edge* e7 = new Edge(e, f, 2);
	Edge* e8 = new Edge(d, g, 1);
	Edge* e9 = new Edge(g, f, 1);*/

	n1->distanceFromStart = 0; // set start node
	Dijkstras();
	PrintLoadShortestRouteTo(n4);

	// TODO: Node / Edge memory cleanup not included
}

void Dijkstras()
{
	while (nodes.size() > 0)
	{
		Node* smallest = ExtractSmallest(nodes);
		vector<Node*>* adjacentNodes = 
			AdjacentRemainingNodes(smallest);

		const int size = adjacentNodes->size();
		for (int i=0; i<size; ++i)
		{
			Node* adjacent = adjacentNodes->at(i);
			int distance = Distance(smallest, adjacent) +
				smallest->distanceFromStart;
			
			if (distance < adjacent->distanceFromStart)
			{
				adjacent->distanceFromStart = distance;
				adjacent->previous = smallest;
			}
		}
		delete adjacentNodes;
	}
}

// Find the node with the smallest distance,
// remove it, and return it.
Node* ExtractSmallest(vector<Node*>& nodes)
{
	int size = nodes.size();
	if (size == 0) return NULL;
	int smallestPosition = 0;
	Node* smallest = nodes.at(0);
	for (int i=1; i<size; ++i)
	{
		Node* current = nodes.at(i);
		if (current->distanceFromStart <
			smallest->distanceFromStart)
		{
			smallest = current;
			smallestPosition = i;
		}
	}
	nodes.erase(nodes.begin() + smallestPosition);
	return smallest;
}

// Return all nodes adjacent to 'node' which are still
// in the 'nodes' collection.
vector<Node*>* AdjacentRemainingNodes(Node* node)
{
	vector<Node*>* adjacentNodes = new vector<Node*>();
	const int size = edges.size();
	for(int i=0; i<size; ++i)
	{
		Edge* edge = edges.at(i);
		Node* adjacent = NULL;
		if (edge->node1 == node)
		{
			adjacent = edge->node2;
		}
		else if (edge->node2 == node)
		{
			adjacent = edge->node1;
		}
		if (adjacent && Contains(nodes, adjacent))
		{
			adjacentNodes->push_back(adjacent);
		}
	}
	return adjacentNodes;
}

// Return distance between two connected nodes
int Distance(Node* node1, Node* node2)
{
	const int size = edges.size();
	for(int i=0; i<size; ++i)
	{
		Edge* edge = edges.at(i);
		if (edge->Connects(node1, node2))
		{
			return edge->distance;
		}
	}
	return -1; // should never happen
}

// Does the 'nodes' vector contain 'node'
bool Contains(vector<Node*>& nodes, Node* node)
{
	const int size = nodes.size();
	for(int i=0; i<size; ++i)
	{
		if (node == nodes.at(i))
		{
			return true;
		}
	}
	return false;
}

///////////////////

void PrintLoadShortestRouteTo(Node* destination)
{
	Node* previous = destination;

	cout << "Distance from start: " 
		<< destination->distanceFromStart << endl;
	cout << "destination->id = " << destination->id << endl;
	while (previous)
	{
		cout << previous->id << " ";
        waypoints.insert(waypoints.begin(), previous);
		previous = previous->previous;
	}
	cout << endl;
}

vector<Edge*>* AdjacentEdges(vector<Edge*>& edges, Node* node)
{
	vector<Edge*>* adjacentEdges = new vector<Edge*>();

	const int size = edges.size();
	for(int i=0; i<size; ++i)
	{
		Edge* edge = edges.at(i);
		if (edge->node1 == node)
		{
			cout << "adjacent: " << edge->node2->id << endl;
			adjacentEdges->push_back(edge);
		}
		else if (edge->node2 == node)
		{
			cout << "adjacent: " << edge->node1->id << endl;
			adjacentEdges->push_back(edge);
		}
	}
	return adjacentEdges;
}

void RemoveEdge(vector<Edge*>& edges, Edge* edge)
{
	vector<Edge*>::iterator it;
	for (it=edges.begin(); it<edges.end(); ++it)
	{
		if (*it == edge)
		{
			edges.erase(it);
			return;
		}
	}
}

