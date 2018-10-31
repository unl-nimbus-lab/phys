#include <asctec_control.h>
#include "asctec_control.cpp"

PUB_DATA pub;
PUB_DATA * runtime = &pub;

GOAL_DATA g_data;
GOAL_DATA * goal = &g_data;

geometry_msgs::Twist xy;

bool fb_prev = false;
bool fb_flag = false;

ros::Publisher fback_pub, cmd_pub, state_pub, trail_pub;
visualization_msgs::Marker trail;

void configCallback(const pc_asctec_sim::ascTunerConfig &config, uint32_t level) {

	runtime->k_val.kpx = config.kpx;
	runtime->k_val.kix = config.kix;
	runtime->k_val.kvx = config.kvx;
	runtime->k_val.kax = config.kax;

	runtime->k_val.kpy = config.kpy;
	runtime->k_val.kiy = config.kiy;
	runtime->k_val.kvy = config.kvy;
	runtime->k_val.kay = config.kay;

	runtime->k_val.kpz = config.kpz;
	runtime->k_val.kiz = config.kiz;
	runtime->k_val.kvz = config.kvz;
	runtime->k_val.kaz = config.kaz;

	runtime->k_val.kpyaw = config.kpyaw;
	runtime->k_val.kiyaw = config.kiyaw;
	runtime->k_val.kvyaw = config.kvyaw;
	runtime->k_val.kayaw = config.kayaw;
}

void xyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	xy = *msg;
}

void modeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	runtime->xyFree = msg->data;
}

void startCallback(const std_msgs::Bool::ConstPtr& msg)
{
	runtime->running = msg->data;
	runtime->xyFree = false;
	ROS_INFO("Start signal!");
}

void goalCallback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg)
{
	goal->goal = *msg;
	goal->isNew = true;
}

void llCallback(const pc_asctec_sim::LLStatus::ConstPtr& msg)
{
	runtime->battery = msg->battery_voltage;
}

void initTrail(string str_frame)
{
	trail.header.frame_id = str_frame;
	trail.header.stamp = ros::Time::now();
	trail.id = 2;
	trail.action = visualization_msgs::Marker::ADD;
	trail.type = visualization_msgs::Marker::LINE_LIST;
	trail.color.a = 1.0;				
	trail.color.b = 1.0;
	trail.color.g = 0.7;

	trail.scale.x = 0.05;
	trail.scale.y = 0.05;

	geometry_msgs::Point vis_trail;
	vis_trail.x = runtime->state.x;
	vis_trail.y = runtime->state.y;
	vis_trail.z = runtime->state.z;
	trail.points.push_back(vis_trail);
}

float limitOutput(float input, float ceiling, float floor) 
{
	if(input > ceiling) {
		return ceiling;
	}else if(input < floor) {
		return floor;
	}else {
		return input;
	}
}

void publishData()
{
	if(runtime->xyFree) {
		runtime->TRPYcmd.roll = limitOutput(xy.angular.x,ROLL_MAX,ROLL_MIN)/XY_LIMIT;
		runtime->TRPYcmd.pitch = limitOutput(xy.angular.y,PITCH_MAX,PITCH_MIN)/XY_LIMIT;
	}
	cmd_pub.publish(runtime->TRPYcmd);
	state_pub.publish(runtime->state);
	if(fb_flag) {
		fback_pub.publish(runtime->g_feedback);
		fb_flag = false;
	}

	geometry_msgs::Point vis_trail;
	vis_trail.x = runtime->state.x;
	vis_trail.y = runtime->state.y;
	vis_trail.z = runtime->state.z;

	trail.points.push_back(vis_trail);
	trail_pub.publish(trail);
	trail.points.push_back(vis_trail);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_controller");
	ros::NodeHandle nh;
	ros::Rate rate(CONTROL_RATE);

	dynamic_reconfigure::Server<pc_asctec_sim::ascTunerConfig> server;
	dynamic_reconfigure::Server<pc_asctec_sim::ascTunerConfig>::CallbackType f;
	f = boost::bind(&configCallback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Dynamic Reconfigure Server Started");

	K_DATA k_vals;
	string q_name, w_frame, q_frame;

	ros::param::get("~w_frame", w_frame);
	ros::param::get("~q_name", q_name);
	ros::param::get("~q_frame", q_frame);

	ros::param::get("~kpx", k_vals.kpx); 
	ros::param::get("~kix", k_vals.kix); 
	ros::param::get("~kvx", k_vals.kvx);
	ros::param::get("~kax", k_vals.kax);

	ros::param::get("~kpy", k_vals.kpy); 
	ros::param::get("~kiy", k_vals.kiy); 
	ros::param::get("~kvy", k_vals.kvy);
	ros::param::get("~kay", k_vals.kay);

	ros::param::get("~kpz", k_vals.kpz); 
	ros::param::get("~kiz", k_vals.kiz); 
	ros::param::get("~kvz", k_vals.kvz);	
	ros::param::get("~kaz", k_vals.kaz);
 
	ros::param::get("~kpyaw", k_vals.kpyaw); 
	ros::param::get("~kiyaw", k_vals.kiyaw); 
	ros::param::get("~kvyaw", k_vals.kvyaw);
	ros::param::get("~kayaw", k_vals.kayaw);

	ros::Subscriber goal_sub = nh.subscribe(q_name + "/pos_goals", 1, goalCallback);
	ros::Subscriber ll_sub = nh.subscribe(q_name + "/ll_status", 1, llCallback);
	ros::Subscriber run_sub = nh.subscribe(q_name + "/start", 1, startCallback);
	ros::Subscriber mode_sub = nh.subscribe(q_name + "/cmd_mode", 1, modeCallback);
	ros::Subscriber xy_sub = nh.subscribe(q_name + "/xy_cmds", 5, xyCallback);

	fback_pub = nh.advertise<pc_asctec_sim::pc_feedback>(q_name + "/goal_feedback", 10);
	cmd_pub = nh.advertise<pc_asctec_sim::SICmd>(q_name + "/cmd_si", 10);
	state_pub = nh.advertise<pc_asctec_sim::pc_state>(q_name + "/state", 10);
	trail_pub = nh.advertise<visualization_msgs::Marker>(q_name + "/quad_trail",10);

	AscTec_Controller asc(q_frame, w_frame, &k_vals);
	tf::StampedTransform transform;
	tf::TransformListener listener;	
	listener.waitForTransform(w_frame, q_frame, ros::Time(0), ros::Duration(3.0));

	initTrail(w_frame);

	listener.lookupTransform(w_frame, q_frame, ros::Time(0), transform);
	goal->goal.x = transform.getOrigin().x();
	goal->goal.y = transform.getOrigin().y();
	goal->goal.z = transform.getOrigin().z();
	goal->goal.yaw = tf::getYaw(transform.getRotation());
	goal->isNew = true;

	ROS_INFO("Asctec Controller Running!");

	while(ros::ok()) {
		ros::spinOnce();

		listener.lookupTransform(w_frame, q_frame, ros::Time(0), transform);
		runtime = asc.runAsctec(runtime, goal, &transform);

		if((fb_prev != runtime->g_feedback.arrived) && !fb_prev) { fb_flag = true; }
		if(goal->isNew) { goal->isNew = false; }
		fb_prev = runtime->g_feedback.arrived; 

		publishData();

		rate.sleep();
	}
	return 0;
}
