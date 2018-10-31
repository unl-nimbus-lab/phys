#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <ohm_igvc/waypoint.h>
#include <ohm_igvc/get_successors.h>
#include <ohm_igvc/cell_to_real.h>
#include <ohm_igvc/real_to_cell.h>
#include <ohm_igvc/coordinate_convert.h>
#include <ohm_igvc/robot_position.h>

#include <boost/heap/priority_queue>
#include <vector>
#include <array>
#include <cmath>

struct Node {
    Node *parent;
    int x, y, which_child;
    float f, g, h, t;

	bool operator<(const Node &lhs, const Node &rhs) { return lhs.f < rhs.f; };
	bool operator>(const Node &lhs, const Node &rhs) { return rhs < lhs; };
	bool operator<=(const Node &lhs, const Node &rhs) { return !(lhs > rhs); };
	bool operator>=(const Node &lhs, const Node &rhs) { return !(lhs < rhs); };

	bool operator==(const Node &lhs, const Node &rhs) { return (lhs.x == rhs.x && lhs.y == rhs.y); };

	Node() {
		parent = nullptr;
		x = y = which_child = 0;
		f = g = h = 0.0;
	};
}

class Planner {
    public:
        Planner(); // 
        std::vector<geometry_msgs::Pose2D> plan(int x, int y); // 
		void get_next_waypoint(int i); // 
		int get_robot_x() { return robot_position.x; };
		int get_robot_y() { return robot_position.y; };
		int get_goal_x() { return goal.x; };
		int get_goal_y() { return goal.y; };
		void get_robot_position(); //
		double distance_to_goal() { return distance(robot_position, goal); };
		double distance_to_last() { return distance(robot_position, last_in_path); };
		geometry_msgs::Point get_real_robot_position() { return cell_to_world(robot_position.x, robot_position.y); };
		void replan_path(const ros::TimerEvent &e);
			
    private:
		std::array<Node, 8> get_successors(int x, int y, Node &parent); // 
		geometry_msgs::Point cell_to_world(int x, int y);  //  

		double distance(Node first, Node second) { return std::hypot((second.x - first.x), (second.y - first.y)); }; // 
		double angular_distance(double a1, double a2) { return 180.0 - std::fabs(std::fabs(a1 - a2) - 180.0); }; // 

        double permissible_distance;
		double robot_theta;
		int robot_x, robot_y;

		double child_angle[8];

		Node goal;
		Node last_in_path;
		Node robot_position;

        ros::NodeHandle node;
        ros::ServiceClient map_get_successors, map_cell_to_world, map_world_to_cell, map_robot_position, coord_convert, waypoint_service;
};

Planner::Planner() {
	permissible_distance = 25;
	node.param("plan_distance", permissible_distance, permissible_distance);
	
	map_get_successors = node.serviceClient<ohm_igvc::get_successors>("get_successors", true);
	map_cell_to_world = node.serviceClient<ohm_igvc::cell_to_real>("cell_to_real", true);
	map_world_to_cell = node.serviceClient<ohm_igvc::real_to_cell>("real_to_cell", true);
	map_robot_position = node.serviceClient<ohm_igvc::robot_position>("get_robot_position", true);
	coord_convert = node.serviceClient<ohm_igvc::coordinate_convert>("coordinate_convert", true);
	waypoint_service = node.serviceClient<ohm_igvc::waypoint>("waypoint");
}

std::vector<geometry_msgs::Pose2D> Planner::plan(int x = robot_position.x, int y = robot_position.y) {
    boost:heap::priority_queue<Node> open, closed;
	std::vector<Node> path;
	std::vector<geometry_msgs::Pose2D> final_path;	

	path.reserve(100);
	final_path.reserve(40);
	
	Node start;
	start.parent = nullptr;
	start.f = 0;
	start.x = x;
	start.y = y;

	open.push_back(start);

	Node last_q = start;

	bool searching = true;

	while(!open.is_empty() && searching) {
		Node q = open.top();
		open.pop();
		
		std::array<Node, 8> successors = get_successors(q.x, q.y, Node &parent);

		for(auto child = successors.begin(); it != successors.end(); ++it) {
			if(child->parent == nullptr) continue;
			
			if(child == goal) {
				goal.parent = child.parent;
				last_q = goal;                               
				searching = false;
				break;
			}

			child->g = q.g + distance(q, *child);
			child->h = distance(*child, goal);
			child->f = child->g + child->h + child->t;

			if(child->g > permissible_distance) continue;

			// sorry for this.

			bool skip = false;

			for(auto it = open.begin(); it != open.end(); ++it) { if(child == *it && child > *it) skip = true; }
			if(skip) continue;

			skip = false;

			for(auto it = closed.begin(); it != closed.end(); ++it) { if(child == *it && child > *it) skip = true; }
			if(skip) continue;

			// end sorry

			open.push(*child);
		}

		closed.push(q);

		last_q = q;                                                                                                                          
		
	}

	for(Node n = last_q; n.parent != nullptr; n = (*n).parent) { path.push_back(n); }
	
	for(auto node = path.rbegin(), auto next = path.rbegin() + 1; next != path.rend(); ++node, ++next) {
		if(angular_distance(child_angle[node->which_child], child_angle[next->which_child]) >= 90.0) {
			geometry_msgs::Pose2D waypoint;
			geometry_msgs::Point real_coord = cell_to_world(node->x, node->y);

			waypoint.x = real_coord.x;
			waypoint.y = real_coord.y;
			waypoint.theta = child_angle[node->which_child];

			final_path.push_back(waypoint);
		}
	}

	last_in_path = path.front();

	return final_path;
}

void get_next_waypoint(int i) {
	ohm_igvc::waypoint req_wp;
	ohm_igvc::coordinate_convert req_conv;
	ohm_igvc::real_to_cell req_cell;

	req_wp.request.ID = i;
	
	waypoint_service.call(req_wp);

	req_conv.request.coordinate.latitiude = req_wp.response.waypoint.latitude;
	req_conv.request.coordinate.longitude = req_wp.response.waypoint.longitude;


	coord_convert.call(req_conv);

	real_to_cell.request.coordinate.x = req_conv.response.coordinate.x;
	real_to_cell.request.coordinate.y = req_conv.response.coordinate.y;

	map_real_to_cell.call(req_cell);

	goal.x = req_cell.response.x;
	goal.y = req_cell.response.y;
}

std::array<Node, 8> Planner::get_successors(int x, int y, Node &parent) {
	std::array<Node, 8> successors;

	if(map_get_successors) {
		ohm_igvc::get_successors req;
		req.x = x;
		req.y = y;
		
		map_get_successors.call(req);
		
		for(int i = 0; i < 8; i++) {
			if(req.nodes[i].t_cost >= 0) {
				successors[i].parent = &parent;
			}
			
			successors[i].x = req.nodes[i].x;
			successors[i].y = req.nodes[i].y;	
			successors[i].which_child = i;		
			successors[i].t = req.nodes[i].t;
		}
	}

	return successors;
}	

geometry_msgs::Point Planner::cell_to_world(int x, int y){
	/*** TODO: Implement reconnection logic for service calls ***/
	// if(map_cell_to_world) {
	ohm_igvc::cell_to_real req;

	req.request.x = x;
	req.request.y = y;

	map_cell_to_world.call(req);

	return req.response.real_coordinate;
	// }
}

void Planner::get_robot_position() {
	ohm_igvc::robot_position req;
	
	map_robot_position.call(req);

	robot_position.x = req.response.x;
	robot_position.y = req.response.y;
}

