#include <asctec_quad.h>
#include "asctec_control.cpp"
#include "atraj.cpp"
#include "asctec_quad.cpp"

QUAD_CMD cmd;
QUAD_OUT *feedback = new QUAD_OUT;
tf::StampedTransform qtransform;

visualization_msgs::Marker tTrail, qTrail;
ros::Publisher viz_pub, tTrail_pub, qTrail_pub;

/* -------------------- callbacks -------------------- */
void configCallback(const pc_asctec_sim::ascTunerConfig &config, uint32_t level) 
{
	cmd.kvals.kpx = config.kpx;
	cmd.kvals.kix = config.kix;
	cmd.kvals.kvx = config.kvx;
	cmd.kvals.kax = config.kax;

	cmd.kvals.kpy = config.kpy;
	cmd.kvals.kiy = config.kiy;
	cmd.kvals.kvy = config.kvy;
	cmd.kvals.kay = config.kay;

	cmd.kvals.kpz = config.kpz;
	cmd.kvals.kiz = config.kiz;
	cmd.kvals.kvz = config.kvz;
	cmd.kvals.kaz = config.kaz;

	cmd.kvals.kpyaw = config.kpyaw;
	cmd.kvals.kiyaw = config.kiyaw;
	cmd.kvals.kvyaw = config.kvyaw;
	cmd.kvals.kayaw = config.kayaw;
}

void startCallback(const std_msgs::Bool::ConstPtr& msg)
{
	cmd.start = msg->data;
	cmd.xyFree = false;
}

void pathCallback(const pc_asctec_sim::pc_traj_cmd::ConstPtr& msg) 
{
	if(msg->points != 0) {
		cmd.f_path = *msg;
		cmd.newPath = true;
	}else {
		ROS_INFO("Set number of points; path ignored");
	}
}

void llCallback(const pc_asctec_sim::LLStatus::ConstPtr& msg)
{
	cmd.battery = msg->battery_voltage;
}

void posCallback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg)
{
	cmd.gNew.goal = *msg;
	cmd.gNew.isNew = true;
}

void xymodeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) {
		cmd.xyFree = 1;
	}else {
		cmd.xyFree = 0;
	}
}

void xmodeCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) {
		cmd.xyFree = 2;
	}else {
		cmd.xyFree = 0;
	}
}

void xyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	cmd.xyCmd = *msg;

}

void estStCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	cmd.x = msg->linear.x;
	cmd.y = msg->linear.y;
	cmd.vx = msg->angular.x;
	cmd.vy = msg->angular.y;
}

void clearCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) {
		tTrail.points.clear();
		qTrail.points.clear();

		geometry_msgs::Point vis_trail;
		vis_trail.x = feedback->state.x;
		vis_trail.y = feedback->state.y;
		vis_trail.z = feedback->state.z;

		tTrail.points.push_back(vis_trail);
		qTrail.points.push_back(vis_trail);
	}
}

/* -------------------- callbacks end -------------------- */

void resetFlags()
{
	cmd.newPath = false;
	cmd.gNew.isNew = false;
}

void initTrails(string frame)
{
	tTrail.header.frame_id = frame;
	tTrail.header.stamp = ros::Time::now();
	tTrail.id = 2;
	tTrail.action = visualization_msgs::Marker::ADD;
	tTrail.type = visualization_msgs::Marker::LINE_LIST;
	tTrail.color.a = 1.0;
	tTrail.color.g = 1.0;				
	tTrail.color.b = 1.0;
	tTrail.scale.x = 0.05;
	tTrail.scale.y = 0.05;

	qTrail.header.frame_id = frame;
	qTrail.header.stamp = ros::Time::now();
	qTrail.id = 2;
	qTrail.action = visualization_msgs::Marker::ADD;
	qTrail.type = visualization_msgs::Marker::LINE_LIST;
	qTrail.color.a = 1.0;				
	qTrail.color.b = 1.0;
	qTrail.color.g = 0.7;
	qTrail.scale.x = 0.05;
	qTrail.scale.y = 0.05;

	geometry_msgs::Point vis_trail;
	vis_trail.x = qtransform.getOrigin().x();
	vis_trail.y = qtransform.getOrigin().y();
	vis_trail.z = qtransform.getOrigin().z();

	tTrail.points.push_back(vis_trail);
	qTrail.points.push_back(vis_trail);
}

void pubTrails(struct TRAIL * g, pc_asctec_sim::pc_state * st, string wframe_)
{
	geometry_msgs::Point vis_trail;
		
	//Publish full quad path
	vis_trail.x = st->x;
	vis_trail.y = st->y;
	vis_trail.z = st->z;
	qTrail.action = visualization_msgs::Marker::ADD;
	qTrail.points.push_back(vis_trail);
	qTrail_pub.publish(qTrail);
	qTrail.points.push_back(vis_trail);

	//Publish full trajectory path
	vis_trail.x = g->x;
	vis_trail.y = g->y;
	vis_trail.z = g->z;
	tTrail.action = visualization_msgs::Marker::ADD;
	tTrail.points.push_back(vis_trail);
	tTrail_pub.publish(tTrail);
	tTrail.points.push_back(vis_trail);

	//Publish immediate goal point
	geometry_msgs::PointStamped viz;
	viz.header.stamp = ros::Time::now();
	viz.header.frame_id = wframe_;
	viz.point.x = g->x;
	viz.point.y = g->y;
	viz.point.z = g->z;
	viz_pub.publish(viz);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_");
	ros::NodeHandle nh;
	ros::Rate rate(CONTROL_RATE);

	/* -------------------- Dynamic Reconfigure -------------------- */
	dynamic_reconfigure::Server<pc_asctec_sim::ascTunerConfig> server;
	dynamic_reconfigure::Server<pc_asctec_sim::ascTunerConfig>::CallbackType f;
	f = boost::bind(&configCallback, _1, _2);
	server.setCallback(f);

	/* -------------------- roslaunch parameter values -------------------- */
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

	/* -------------------- Publishers, global Publishers, and Subscribers -------------------- */
	viz_pub = nh.advertise<geometry_msgs::PointStamped>(q_name + "/viz_goals",10);			//rviz - current goal - pink
	tTrail_pub = nh.advertise<visualization_msgs::Marker>(q_name + "/trajectory_trail",10);		//rviz - trajectory - L blue
	qTrail_pub = nh.advertise<visualization_msgs::Marker>(q_name + "/quad_trail",10);		//rviz - quad path - D blue

	ros::Publisher cmd_pub = nh.advertise<pc_asctec_sim::SICmd>(q_name + "/cmd_si", 10);		//TRPY commands to ll_node
	ros::Publisher end_pub = nh.advertise<std_msgs::Bool>(q_name + "/traj_end",10);			//Traj feedback showing end
	ros::Publisher state_pub = nh.advertise<pc_asctec_sim::pc_state>(q_name + "/state",10);		//Quad state data
	ros::Publisher goal_pub = nh.advertise<pc_asctec_sim::pc_goal_cmd>(q_name + "/cmd_now",10);	//Quad current goal data

	ros::Subscriber start_sub = nh.subscribe(q_name + "/start", 1, startCallback);			//Moves Quad from free state
	ros::Subscriber path_sub = nh.subscribe(q_name + "/traj_points", 1, pathCallback);		//Traj commands heard here
	ros::Subscriber ll_sub = nh.subscribe(q_name + "/ll_status", 1, llCallback);			//Battery voltage level
	ros::Subscriber pos_sub = nh.subscribe(q_name + "/pos_goals", 1, posCallback);			//Manual position setting
	ros::Subscriber xymode_sub = nh.subscribe(q_name + "/xycmd_mode", 1, xymodeCallback);		//Control XY axes mode
	ros::Subscriber xmode_sub = nh.subscribe(q_name + "/xcmd_mode", 1, xmodeCallback);		//Control XY axes mode
	ros::Subscriber xy_sub = nh.subscribe(q_name + "/xy_cmds", 5, xyCallback);			//Roll Pitch commands
	ros::Subscriber estSt_sub = nh.subscribe(q_name + "/est_state", 1, estStCallback);
	ros::Subscriber clear_sub = nh.subscribe(q_name + "/clear_trails", 1, clearCallback);

	/* -------------------- transform listener -------------------- */
	tf::TransformListener listener;	
	listener.waitForTransform(w_frame, q_frame, ros::Time(0), ros::Duration(3.0));

	/* -------------------- AscTec Quad Object and viz trails-------------------- */
	AscTec_Quad asc(q_frame, w_frame, &listener, dT, &k_vals);
	initTrails(w_frame);
	ROS_INFO("Asctec Quad Running!");

	while(ros::ok()) {
		//Handle callbacks
		ros::spinOnce();

		//Run quad with feedback data and updated commands
		feedback = asc.runQuad(&cmd);

		//Publish quad state
		state_pub.publish(feedback->state);

		//Publish quad goal for debug purposes
		goal_pub.publish(*asc.controller.getGoal());

		//Publish trajectory completion signal
		std_msgs::Bool ender;
		ender.data = feedback->isComplete;
		end_pub.publish(ender);

		//Publish TRPY commands to ll node
		cmd_pub.publish(feedback->TRPYcmd);

		//Publish visualization markers
		pubTrails(&feedback->goal,&feedback->state,w_frame);

		//Reset newPath and gNew so path and pos goals are only sent once
		resetFlags();

		//Sleep and maintain desired control rate
		rate.sleep();
	}
	return 0;
}
