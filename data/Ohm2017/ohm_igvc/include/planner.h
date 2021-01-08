#ifndef _PLANNER_H_
#define _PLANNER_H_

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

#endif
