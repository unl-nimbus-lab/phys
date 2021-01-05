#include <atraj.h>
#include "atraj.cpp"

#define freq 20.0
#define dt 1/freq

ros::Timer timer_event;

Trajectory_Accel *accTraj;
pc_asctec_sim::pc_state q_state;

string q_name, world;
bool isTiming = false;

ros::Publisher trail_pub, viz_pub;
visualization_msgs::Marker trail;

void pathCallback(const pc_asctec_sim::pc_traj_cmd::ConstPtr& msg) 
{
	if(msg->points != 0) {
		NEW_PATH path;
		path.cmd = *msg;
		path.state = q_state;
		path.type = buffer;
	
		accTraj->setBMatrix(&path);
	}else {
		ROS_INFO("Set number of points; path ignored");
	}
}
void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg) 
{
	q_state = *msg;
}

void init_trail(void)
{
	trail.header.frame_id = "/odom";
	trail.header.stamp = ros::Time::now();
	trail.id = 2;
	trail.action = visualization_msgs::Marker::ADD;
	trail.type = visualization_msgs::Marker::LINE_LIST;
	trail.color.a = 1.0;
	trail.color.g = 1.0;				
	trail.color.b = 1.0;

	trail.scale.x = 0.05;
	trail.scale.y = 0.05;

	geometry_msgs::Point vis_trail;
	vis_trail.x = q_state.x;
	vis_trail.y = q_state.y;
	vis_trail.z = q_state.z;

	trail.points.push_back(vis_trail);
}

void publish_trail(float x, float y, float z)
{
	geometry_msgs::Point vis_trail;
	vis_trail.x = x;
	vis_trail.y = y;
	vis_trail.z = z;

	trail.points.push_back(vis_trail);
	trail_pub.publish(trail);
	trail.points.push_back(vis_trail);

	geometry_msgs::PointStamped viz;
	viz.header.stamp = ros::Time::now();
	viz.header.frame_id = "/odom";
	viz.point.x = x;
	viz.point.y = y;
	viz.point.z = z;
	viz_pub.publish(viz);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "min_accel_node");
	ros::NodeHandle nh;
	ros::Rate rate(freq);

	ros::param::get("~q_name", q_name);
	ros::param::get("~w_frame", world);

	ros::Subscriber path_sub = nh.subscribe(q_name + "/traj_points", 1, pathCallback);
	ros::Subscriber state_sub = nh.subscribe(q_name + "/state", 10, stateCallback);

	ros::Publisher end_pub = nh.advertise<std_msgs::Bool>(q_name + "/traj_end",10);
	ros::Publisher goal_pub = nh.advertise<pc_asctec_sim::pc_goal_cmd>(q_name + "/pos_goals", 10);
	trail_pub = nh.advertise<visualization_msgs::Marker>(q_name + "/trajectory_trail",10);
	viz_pub = nh.advertise<geometry_msgs::PointStamped>(q_name + "/viz_goals",10);

	init_trail();

	accTraj = new Trajectory_Accel(dt);
	WAYPOINT W, * W_ptr;
	W_ptr = &W;

	ROS_INFO("Min Accel Node Running!");

	while(ros::ok()) {
		ros::spinOnce();
		if(!accTraj->getComplete()) {
			W_ptr = accTraj->updateWaypoint(W_ptr);
			if(W_ptr->isValid) {
				goal_pub.publish(W_ptr->goal);
			}
		}

		std_msgs::Bool end;
		end.data = accTraj->getComplete();
		end_pub.publish(end);

		publish_trail(W_ptr->goal.x, W_ptr->goal.y, W_ptr->goal.z);
		rate.sleep();
	}
	return 0;
}
