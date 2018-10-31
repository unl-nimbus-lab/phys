#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ohm_igvc/waypoint.h>
#include <ohm_igvc/get_successors.h>
#include <ohm_igvc/cell_to_real.h>
#include <ohm_igvc/real_to_cell.h>
#include <ohm_igvc/coordinate_convert.h>
#include <ohm_igvc/position_update.h>
#include <ohm_igvc/pid_feedback.h>
#include <ohm_igvc/planned_path.h>
#include <ohm_igvc/drive_mode.h>
#include <ohm_igvc/path_debug.h>

#include <boost/heap/priority_queue.hpp>
#include <vector>
#include <array>
#include <cmath>

struct Node {
    Node *parent;
    int x, y, which_child;
    float f, g, h, t;
	Node() {
		parent = nullptr;
		x = y = which_child = 0;
		f = g = h = 0.0;
	};
};

bool operator<(const Node &lhs, const Node &rhs) { return lhs.f < rhs.f; };
bool operator>(const Node &lhs, const Node &rhs) { return rhs < lhs; };
bool operator<=(const Node &lhs, const Node &rhs) { return !(lhs > rhs); };
bool operator>=(const Node &lhs, const Node &rhs) { return !(lhs < rhs); };
bool operator==(const Node &lhs, const Node &rhs) { return (lhs.x == rhs.x && lhs.y == rhs.y); };

class Planner {
    public:
        Planner(); // 
        void plan(int x, int y); //
		void run(); 
		bool is_finished() { return finished; };

		bool get_next_waypoint(); // 
		void get_robot_position(const ohm_igvc::position_update::ConstPtr &pos); // change to message

		void request_speed(double speed);
		void set_first_target();
		void set_next_target();

		double distance_to_goal() { return real_distance(real_position, real_goal); };
		double distance_to_last() { return real_distance(real_position, last_in_path); };
		geometry_msgs::Point get_robot_real_position() { return real_position; };
		
		void path_debug(const ros::TimerEvent &e);

		void pid_feedback_callback(const ohm_igvc::pid_feedback::ConstPtr &fb);
		void drive_mode_callback(const ohm_igvc::drive_mode::ConstPtr &mode);
			
    private:
		std::array<Node, 8> get_successors(int x, int y, Node &parent); // 
		geometry_msgs::Point cell_to_world(int x, int y);  //  

		double real_distance(geometry_msgs::Point x1, geometry_msgs::Point x2) { return std::hypot((x2.x - x1.x), (x2.y - x1.y)); };
		double distance(Node first, Node second) { return std::hypot((second.x - first.x), (second.y - first.y)); }; // 
		double angular_distance(double a1, double a2) { return 180.0 - std::fabs(std::fabs(a1 - a2) - 180.0); }; // 

		double child_angle[8] = {315.0, 0.0, 45.0, 90.0, 135.0, 180.0, 215.0, 270.0};

        double permissible_distance;
		double planning_threshold = 5.0, hit_threshold = 2.0;
		double planning_rate = 0.5;
		double OP_SPEED = 0.25, PLAN_SPEED = 0.1, STOP = 0.0, SPEEDY_GONZALES = 0.5;

		geometry_msgs::Point real_position, real_goal, last_in_path;
		Node robot_position, goal;

		ohm_igvc::pid_feedback feedback;

		bool debug, auto_mode, finished, first_run;
		int waypoint_id;
		std::vector<geometry_msgs::Pose2D> current_path;
		ohm_igvc::planned_path pid_path;
		ros::Time last_path_update;
		
        ros::NodeHandle node;
		ros::Subscriber pid_feedback, drive_mode, robot_position_update;
        ros::ServiceClient map_get_successors, map_cell_to_world, map_world_to_cell, coord_convert, waypoint_service;
		ros::Publisher path_debug_pub, target, desired_speed;
		ros::Timer debug_publisher;
};

Planner::Planner() {
	auto_mode = false;
	finished = false;
	waypoint_id = 0;
	pid_path.dir = 1;

	current_path.reserve(100);

	permissible_distance = 25;
	planning_threshold = 5.0, hit_threshold = 2.0; // planning threshold should be percentage?
	planning_rate = 0.5;
	OP_SPEED = 0.25, PLAN_SPEED = 0.1, STOP = 0.0, SPEEDY_GONZALES = 0.5;

	debug = false;

	node.param("plan_distance", permissible_distance, permissible_distance);
	node.param("planner_debug", debug, debug);
	node.param("planning_threshold", planning_threshold, planning_threshold);
	node.param("hit_threshold", hit_threshold, hit_threshold);
	node.param("planning_rate", planning_rate, planning_rate);
	node.param("operating_speed", OP_SPEED, OP_SPEED);
	node.param("planning_speed", PLAN_SPEED, PLAN_SPEED);
	node.param("top_speed", SPEEDY_GONZALES, SPEEDY_GONZALES);

	ros::Rate plan_rate(planning_rate);

	if(debug) {
		path_debug_pub = node.advertise<ohm_igvc::path_debug>("/ohm/path_planner_debug", 1);
		debug_publisher = node.createTimer(ros::Duration(1.0), &Planner::path_debug, this);
	}

	if(planning_threshold > 0) pid_feedback = node.subscribe<ohm_igvc::pid_feedback>("ohmPidFeedback", 1, &Planner::pid_feedback_callback, this);

	drive_mode = node.subscribe<ohm_igvc::drive_mode>("drive_mode", 1, &Planner::drive_mode_callback, this);
	robot_position_update = node.subscribe<ohm_igvc::position_update>("/ohm/robot_cell_position", 1, &Planner::get_robot_position, this);
	desired_speed = node.advertise<geometry_msgs::Twist>("/ohm/pathPlannerSpeed", 5);
	target = node.advertise<ohm_igvc::planned_path>("/pathPlanning", 5);
	
	map_get_successors = node.serviceClient<ohm_igvc::get_successors>("get_successors");
	map_cell_to_world = node.serviceClient<ohm_igvc::cell_to_real>("cell_to_real", true);
	map_world_to_cell = node.serviceClient<ohm_igvc::real_to_cell>("real_to_cell", true);
	coord_convert = node.serviceClient<ohm_igvc::coordinate_convert>("coordinate_convert", true);
	waypoint_service = node.serviceClient<ohm_igvc::waypoint>("waypoint");

	if(!map_get_successors.exists()) {
		map_get_successors.waitForExistence();
		ROS_INFO("%s", map_get_successors.getService().c_str());
	}

	get_next_waypoint();

	first_run = true;

	last_path_update = ros::Time::now();
}

void Planner::run() {
	if(auto_mode) {
		if(first_run) {
			plan(robot_position.x, robot_position.y);
			set_first_target();
			request_speed(OP_SPEED);
			first_run = false;
		}

		double d_goal = distance_to_goal(), d_last = distance_to_last();

		if(d_goal < hit_threshold) {
			ROS_INFO("REACHED GOAL %d", waypoint_id);
			request_speed(STOP);
			if(!get_next_waypoint()) finished = true;
			plan(robot_position.x, robot_position.y);
			set_first_target();
			request_speed(OP_SPEED);
		} else if(d_last < planning_threshold) {
			request_speed(PLAN_SPEED);
			ROS_INFO("REACHED PLANNING THRESHOLD");
			plan(robot_position.x, robot_position.y);
			set_first_target();
			request_speed(OP_SPEED);
		} else if(feedback.targetDist < hit_threshold) {
			ROS_INFO("HIT PATH TARGET");
			set_next_target();
		}

		if(current_path.size() > 0) {
			target.publish(pid_path);
		}
	}
}

void Planner::plan(int x, int y) {
	//ROS_INFO("BEGIN_PLANNING");
    boost::heap::priority_queue<Node> open, closed;
	std::vector<Node> path;
	//std::vector<geometry_msgs::Pose2D> final_path;	

	path.reserve(100);
	//final_path.reserve(40);
	current_path.clear();
	
	Node start;
	start.parent = nullptr;
	start.f = 0;
	start.x = x;
	start.y = y;

	open.push(start);

	Node last_q = start;

	bool searching = true;

	//ROS_INFO("A* REACHED");

	while(!open.empty() && searching) {
		Node q = open.top();
		open.pop();
		
		ROS_INFO("Check 0");

		std::array<Node, 8> successors = get_successors(q.x, q.y, q);

		for(auto child = successors.begin(); child != successors.end(); ++child) {
			if(child->parent == nullptr) continue;

			ROS_INFO("Check 1");
			
			if(*child == goal) {
				ROS_INFO("Child: (%d, %d), Goal: (%d, %d)", child->x, child->y, goal.x, goal.y);

				goal.parent = child->parent;
				last_q = goal;                               
				searching = false;
				break;
			}

			child->g = q.g + distance(q, *child);
			child->h = distance(*child, goal);
			child->f = child->g + child->h + child->t;

			if(child->g > permissible_distance) continue;

			ROS_INFO("Check 2");

			// sorry for this.

			bool skip = false;

			for(auto it = open.begin(); it != open.end(); ++it) { if(*child == *it && *child > *it) skip = true; }
			if(skip) continue;

			skip = false;

			for(auto it = closed.begin(); it != closed.end(); ++it) { if(*child == *it && *child > *it) skip = true; }
			if(skip) continue;

			// end sorry

			open.push(*child);
		}

		closed.push(q);

		last_q = q;                                                                                                                          
		
	}

	for(Node *n = &last_q; n->parent != nullptr; n = n->parent) { path.push_back(*n); }
	
	if(path.size() > 0) {
		double angle_avg = 0.0;
		int n = 0;

		ROS_INFO("Check 3");

		for(auto current = path.rbegin(); current != path.rend();) {
			angle_avg = (child_angle[current->which_child] + (n * angle_avg)) / (++n);
			
			geometry_msgs::Pose2D waypoint;
			geometry_msgs::Point real_coord = cell_to_world(current->x, current->y);
			
			waypoint.x = real_coord.x;
			waypoint.y = real_coord.y;

			ROS_INFO("Check 4");			

			for(auto next = current + 1; next != path.rend(); ++next, ++current) {
				if(angular_distance(child_angle[current->which_child], child_angle[next->which_child]) >= 90.0) {
					waypoint.theta = angle_avg;

					angle_avg = 0.0;
					n = 0;

					ROS_INFO("Check 5");
					
					current = next;
					break;
				}
				
				ROS_INFO("Check 6");

				angle_avg += (child_angle[next->which_child] + (n * angle_avg)) / (++n);
			}

			current_path.push_back(waypoint);

			if((current + 1) == path.rend()) {

				ROS_INFO("Check 7");				

				waypoint.theta = angle_avg;
				current_path.push_back(waypoint);

				real_coord = cell_to_world(current->x, current->y);
				
				waypoint.x = real_coord.x;
				waypoint.y = real_coord.y;
				
				waypoint.theta = angle_avg;

				current++;
			}
		}
	}

	//ROS_INFO("FINAL PATH COMPILED");

	last_in_path = cell_to_world(path.front().x, path.front().y);

	if(debug) last_path_update = ros::Time::now();

	ROS_INFO("Check 8");

	//current_path = final_path;
}

bool Planner::get_next_waypoint() {
	ohm_igvc::waypoint req_wp;
	ohm_igvc::coordinate_convert req_conv;
	ohm_igvc::real_to_cell req_cell;

	req_wp.request.ID = waypoint_id;
	
	if(!waypoint_service.call(req_wp)) return false;

	req_conv.request.coordinate.latitude = req_wp.response.waypoint.latitude;
	req_conv.request.coordinate.longitude = req_wp.response.waypoint.longitude;


	if(!coord_convert.call(req_conv)) return false;

	req_cell.request.real_coordinate.x = req_conv.response.coordinate.x;
	req_cell.request.real_coordinate.y = req_conv.response.coordinate.y;

	real_goal.x = req_conv.response.coordinate.x;
	real_goal.y = req_conv.response.coordinate.y;

	if(!map_world_to_cell.call(req_cell)) return false;

	goal.x = req_cell.response.x;
	goal.y = req_cell.response.y;

	waypoint_id++;

	return true;
}

std::array<Node, 8> Planner::get_successors(int x, int y, Node &parent) {
	std::array<Node, 8> successors;

	if(map_get_successors) {
		ohm_igvc::get_successors req;
		req.request.x = x;
		req.request.y = y;
		
		//if(map_get_successors.call(req)) {
		
		for(int i = 0; i < 8; i++) {
			if(req.response.nodes[i].t_cost >= 0) {
				successors[i].parent = &parent;
			}
				
			successors[i].x = req.response.nodes[i].x;
			successors[i].y = req.response.nodes[i].y;	
			successors[i].which_child = i;		
			successors[i].t = req.response.nodes[i].t_cost;
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

void Planner::get_robot_position(const ohm_igvc::position_update::ConstPtr &pos) {
	robot_position.x = pos->x;
	robot_position.y = pos->y;
	real_position = pos->real;
}

void Planner::request_speed(double speed) {
	geometry_msgs::Twist t;
	t.linear.x = speed;
	desired_speed.publish(t);
}

void Planner::set_first_target() {
	pid_path.lastTarget.latitude = real_position.x;
	pid_path.lastTarget.longitude = real_position.y;
	pid_path.currentTarget.latitude = current_path.front().x;
	pid_path.currentTarget.longitude = current_path.front().y;
}

void Planner::set_next_target() {
	pid_path.lastTarget = pid_path.currentTarget;
	current_path.erase(current_path.begin());
	pid_path.currentTarget.latitude = current_path.front().x;
	pid_path.currentTarget.longitude = current_path.front().y;
}

void Planner::path_debug(const ros::TimerEvent &e) {
	ohm_igvc::path_debug d;
	d.target_waypoint = waypoint_id;
	d.where_am_i.x = robot_position.x;
	d.where_am_i.y = robot_position.y;
	d.distance_to_goal = distance_to_goal();
	d.distance_to_last = distance_to_last();
	d.path = current_path;
	//if(current_path.size() == 0) ROS_INFO("NO PATH AAAAAAH!");
	d.last_update.data = last_path_update;
	path_debug_pub.publish(d);
}

void Planner::pid_feedback_callback(const ohm_igvc::pid_feedback::ConstPtr &fb) {
	feedback = *fb;
};

void Planner::drive_mode_callback(const ohm_igvc::drive_mode::ConstPtr &mode) {
	if(mode->mode == "auto") auto_mode = true;
	else auto_mode = false;
}

/* ------------------- // MAIN STUFF // -------------------- */

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_planner");

	Planner planner;
	
	while(ros::ok() && !planner.is_finished()) {
		ros::spinOnce();
		planner.run();
	}

	return 0;
}
