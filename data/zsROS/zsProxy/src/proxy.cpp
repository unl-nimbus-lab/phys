// Author: Li Zhensheng
// 两个功能
// 1）接受服务请求，向move_base发送取消
// 2）接收pose消息，重新配置zs_world_frame坐标系的位置
// 3）接受move_base_goal，转发请求
// 4) 订阅rosaria/zs_pose，改造为一个pose2D发出去，回调函数里检查速度值，当都为0时设置布尔值，使得heading消息可以发送，记得延迟。
// 5) 接受/cmdvel2D,捆绑后，转发给rosaria
#include <string>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"
#include <iomanip> // for std :: setprecision and std :: fixed
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/tf.h"
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

class ProxyNode{
    public:
        ProxyNode(ros::NodeHandle n);
    // TODO:
    private:
		// action client
        MoveBaseActionClient* ac_;
		// publisher
		ros::Publisher zs_heading_pub_;
		ros::Publisher zs_cmdvel_pub_;
		ros::Publisher zs_movetozero_pub_;
		ros::Publisher zs_moveto_pub_;
		// ros::Publisher zs_forceheading_pub_;
		// msg subscriber
        ros::Subscriber zs_tf_sub_;
		ros::Subscriber zs_goal_sub_;
		ros::Subscriber zs_forcegoal_sub_;
		ros::Subscriber cancel_goal_sub_;
		ros::Subscriber zs_goal2D_sub_;
		ros::Subscriber zs_forcegoal2D_sub_;
		ros::Subscriber zs_pose2D_sub_;
		ros::Subscriber rosaria_pose_sub_;
		ros::Subscriber zs_cmdvel2D_sub_;
		ros::Subscriber zs_tf2D_sub_;
		ros::Subscriber zs_max_vel_sub_;
		// ros::Subscriber zs_precise_sub_;
		// srv server
        ros::ServiceServer cancel_goal_srv_;
		// functions
        bool cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
        void reconfigParameterCB(const geometry_msgs::Pose& param);
		void sendGoalCB(const geometry_msgs::Pose& goal);
		void cancelGoalMsgCB(const std_msgs::Bool trigger);
		void sendforceGoalCB(const geometry_msgs::Pose& goal);
		void sendGoal2DCB(const geometry_msgs::Pose2D&);
		void sendforceGoal2DCB(const geometry_msgs::Pose2D&);
		void reconfigParameter2DCB(const geometry_msgs::Pose2D& param);
		void rosaria_poseCB(const nav_msgs::Odometry);
		void sendcmdvel2DCB(const geometry_msgs::Pose2D& msg);
		void reconfigmaxvel(const geometry_msgs::Pose2D& msg);
		// void reconfigMovebaseCB(const std_msgs::Bool& msg);
		// another
        ros::NodeHandle nh_;
        double start_pose_x_;
        double start_pose_y_;
        double start_pose_th_;
        std::string start_pose_x_str_;
        std::string start_pose_y_str_;
        std::string start_pose_th_str_;
		move_base_msgs::MoveBaseGoal goal_;
		std_msgs::Float64 heading;
		bool setheading;
		std_msgs::Bool movetozero;
		geometry_msgs::Twist cmdvel;
		std::string max_x_str_;
        std::string max_y_str_;
        std::string max_th_str_;
		geometry_msgs::Pose2D moveto_;
		
};

ProxyNode::ProxyNode(ros::NodeHandle n):
	// TODO:列表初始化
	nh_(n),
	ac_(NULL),
	start_pose_x_(0.0),
	start_pose_y_(0.0),
	start_pose_th_(0.0),
	start_pose_x_str_(""),
	start_pose_y_str_(""),
	start_pose_th_str_(""),
	setheading(false),
	max_x_str_(""),
	max_y_str_(""),
	max_th_str_("")
{
	// ROS_INFO("zs_proxy constructing...");
	ac_ = new MoveBaseActionClient("move_base", false);
	//srv server
	cancel_goal_srv_ = nh_.advertiseService("cancel_goal", &ProxyNode::cancelGoalCB, this);
	// msg subscriber
	zs_tf_sub_ = nh_.subscribe<geometry_msgs::Pose>("zs_tf", 1, (boost::function <void(const geometry_msgs::Pose)>)boost::bind(&ProxyNode::reconfigParameterCB, this, _1 ));
	zs_goal_sub_ = nh_.subscribe<geometry_msgs::Pose>("zs_goal", 1, (boost::function <void(const geometry_msgs::Pose)>)boost::bind(&ProxyNode::sendGoalCB, this, _1 ));
	cancel_goal_sub_ = nh_.subscribe<std_msgs::Bool>("zs_cancel", 1, (boost::function <void(const std_msgs::Bool)>)boost::bind(&ProxyNode::cancelGoalMsgCB, this, _1 ));
	zs_forcegoal_sub_ = nh_.subscribe<geometry_msgs::Pose>("zs_forcegoal", 1, (boost::function <void(const geometry_msgs::Pose)>)boost::bind(&ProxyNode::sendforceGoalCB, this, _1 ));
	zs_goal2D_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("zs_goal2D", 1, (boost::function <void(const geometry_msgs::Pose2D)>)boost::bind(&ProxyNode::sendGoal2DCB, this, _1 ));
	zs_forcegoal2D_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("zs_forcegoal2D", 1, (boost::function <void(const geometry_msgs::Pose2D)>)boost::bind(&ProxyNode::sendforceGoal2DCB, this, _1 ));
	zs_tf2D_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("zs_tf2D", 1, (boost::function <void(const geometry_msgs::Pose2D)>)boost::bind(&ProxyNode::reconfigParameter2DCB, this, _1 ));
	zs_pose2D_sub_ = nh_.subscribe<nav_msgs::Odometry>("/RosAria/pose", 1, (boost::function <void(const nav_msgs::Odometry)>)boost::bind(&ProxyNode::rosaria_poseCB, this, _1 ));
	zs_cmdvel2D_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("zs_cmdvel2D", 1, (boost::function <void(const geometry_msgs::Pose2D)>)boost::bind(&ProxyNode::sendcmdvel2DCB, this, _1 ));
	zs_max_vel_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("zs_maxvel", 1, (boost::function <void(const geometry_msgs::Pose2D)>)boost::bind(&ProxyNode::reconfigmaxvel, this, _1 ));
	// zs_precise_sub_ = nh_.subscribe<std_msgs::Bool>("zs_precise", 1, (boost::function <void(const std_msgs::Bool)>)boost::bind(&ProxyNode::reconfigMovebaseCB, this, _1 ));
	// msg publisher
	zs_heading_pub_ = nh_.advertise<std_msgs::Float64>("/RosAria/zs_heading",1);
	zs_cmdvel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1000);
	zs_movetozero_pub_ = nh_.advertise<std_msgs::Bool>("/RosAria/zs_movetozero", 1);
	// zs_forceheading_pub_ = nh_.advertise<std_msgs::Float64>("zs_forceheading",1);
	zs_moveto_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/RosAria/zs_moveto",1);
}
void ProxyNode::sendcmdvel2DCB(const geometry_msgs::Pose2D& msg)
{
	cmdvel.linear.x=msg.x;
	cmdvel.angular.z=msg.theta;
	ROS_INFO("send cmdvel to aria ( %f, %f)", msg.x, msg.theta);
	zs_cmdvel_pub_.publish(cmdvel);
}
bool ProxyNode::cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
	ROS_INFO("cancelGoalCB");
	ac_->cancelGoal();
	return true;
}
void ProxyNode::cancelGoalMsgCB(const std_msgs::Bool trigger){
	if(trigger.data){
		ROS_INFO("cancelGoalMsgCB");
		ac_->cancelGoal();
	}
}
void ProxyNode::sendGoalCB(const geometry_msgs::Pose& goal){
	goal_.target_pose.header.frame_id = "zsworld_frame";//odom
  	goal_.target_pose.header.stamp = ros::Time::now();
	goal_.target_pose.pose = goal;
  	ROS_INFO("Sending goal");
  	ac_->sendGoal(goal_);
}
void ProxyNode::sendforceGoalCB(const geometry_msgs::Pose& goal){
	goal_.target_pose.header.frame_id = "zsworld_frame";//odom
  	goal_.target_pose.header.stamp = ros::Time::now();
	goal_.target_pose.pose = goal;
  	ROS_INFO("Sending goal");
  	ac_->sendGoal(goal_);
	// //   不管成不成功，只要move_base执行完毕，都强制调整角度
	// while (!ac_->waitForResult(ros::Duration(1.0)))
	// 	ROS_INFO("forceGoalSending…");//此处需要修改，move_base action server的result为空
	ROS_INFO("Set Heading...");
	heading.data = tf::getYaw(goal.orientation);
	zs_heading_pub_.publish(heading);

}
void ProxyNode::sendGoal2DCB(const geometry_msgs::Pose2D& goal2D){
	goal_.target_pose.header.frame_id = "map";// zsworld_frame
  	goal_.target_pose.header.stamp = ros::Time::now();
	goal_.target_pose.pose.position.x = goal2D.x;
	goal_.target_pose.pose.position.y = goal2D.y;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal2D.theta*M_PI/180), goal_.target_pose.pose.orientation);
  	ROS_INFO_STREAM("Sending goal to " << "( " << goal_.target_pose.pose.position.x << ", " << goal_.target_pose.pose.position.y << ", " << goal2D.theta << " )");
  	ac_->sendGoal(goal_);
}
void ProxyNode::sendforceGoal2DCB(const geometry_msgs::Pose2D& goal2D){
	goal_.target_pose.header.frame_id = "map";// zsworld_frame
  	goal_.target_pose.header.stamp = ros::Time::now();
	goal_.target_pose.pose.position.x = goal2D.x;
	goal_.target_pose.pose.position.y = goal2D.y;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal2D.theta*M_PI/180), goal_.target_pose.pose.orientation);
  	ROS_INFO_STREAM("Sending forcegoal to " << "( " << goal_.target_pose.pose.position.x << ", " << goal_.target_pose.pose.position.y << ", " << goal2D.theta << " )");
  	ac_->sendGoal(goal_);//(boost::function <void(const geometry_msgs::Pose)>)(&ProxyNode::reconfigParameterCB, this, _1 )
	//   不管成不成功，只要move_base执行完毕，都强制调整角度
	// while (!ac_->waitForResult(ros::Duration(1.0)))
	// 	ROS_INFO("forceGoalSending...");
	ROS_INFO("waiting for 2 second");
	ros::Duration(2).sleep(); // sleep for half a second
	while(!setheading)
	{
		ROS_INFO("cannot setheading because robot are moving, waiting for 0.5 second");
		ros::Duration(0.5).sleep(); // sleep for half a second
	}
	heading.data = goal2D.theta;
	ROS_INFO("zsProxy: Set Heading to %f for 1 times, 0.5s after a time", heading.data);
	for(int i=0; i<1; i++)
	{
		zs_heading_pub_.publish(heading);
		// std::stringstream ss_h;
		// std::string heading_str_;
		// ss_h << heading.data;
		// ss_h >> heading_str_;
		// // rostopic pub --once my_topic std_msgs/String "hello there"
		// // std::system(("ros"))
		ROS_INFO("zsProxy: publish zs_heading %d", i);
		ros::Duration(0.5).sleep();
	}

}
void ProxyNode::reconfigParameterCB(const geometry_msgs::Pose& param){
	// 将double转为string，在C++11中可以应用新标准，也可以使用boost库，此处使用C++03标准
	// C++11:	std::string varAsString = std::to_string(myDoubleVar);
	// boost:	std::string str = boost::lexical_cast<std::string>(dbl);
	ROS_INFO("reconfigParameterCB...");
	std::stringstream ss_x, ss_y, ss_th;
	ss_x << param.position.x;
	ss_x >> start_pose_x_str_;
	ss_y << param.position.y;
	ss_y >> start_pose_y_str_;
	ss_th << tf::getYaw(param.orientation);
	ss_th >> start_pose_th_str_;
	// ROS_INFO(start_pose_x_str_.c_str());
	// ROS_INFO(start_pose_y_str_.c_str());
	// ROS_INFO(start_pose_th_str_.c_str());
	std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_x " + start_pose_x_str_).c_str());
	std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_y " + start_pose_y_str_).c_str());
	std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_th "+ start_pose_th_str_).c_str());
}
void ProxyNode::reconfigParameter2DCB(const geometry_msgs::Pose2D& param)
{
	ROS_INFO("reconfigParameter2DCB...");
	// std::stringstream ss_x, ss_y, ss_th;
	// ss_x << param.x;
	// ss_x >> start_pose_x_str_;
	// ss_y << param.y;
	// ss_y >> start_pose_y_str_;
	// ss_th << param.theta;
	// ss_th >> start_pose_th_str_;
	// // ROS_INFO(start_pose_x_str_.c_str());
	// // ROS_INFO(start_pose_y_str_.c_str());
	// // ROS_INFO(start_pose_th_str_.c_str());
	// ROS_INFO_STREAM("dynamic_reconfigure to place ("<< start_pose_x_str_ << ", " << start_pose_y_str_ << ", " << start_pose_th_str_ << " )");
	// std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_x " + start_pose_x_str_).c_str());
	// std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_y " + start_pose_y_str_).c_str());
	// std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_th "+ start_pose_th_str_).c_str());
	// movetozero.data=true;
	// ROS_INFO("robot Odometry move to zero");
	// zs_movetozero_pub_.publish(movetozero);

	// zs:20170601-从reconfigure到发送消息到rosaria，rosaria收到后运行moveto函数，该回调函数仍然订阅tf2D消息，故上位机不需要修改代码
	// 传过来的数据是米m，传到aria需要转化为毫米mm
	moveto_.x=param.x*1000;
	moveto_.y=param.y*1000;
	moveto_.theta=param.theta;
	ROS_INFO_STREAM("dynamic_reconfigure to place ("<< moveto_.x << ", " << moveto_.y << ", " << moveto_.theta << " )");
	zs_moveto_pub_.publish(moveto_);

}
// Goal Tolerance Parameters::yaw_goal_tolerance不是动态配置参数，这个需要修改move_base源代码
// void ProxyNode::reconfigMovebaseCB(const std_msgs::Bool& msg)
// {
// 	ROS_INFO("reconfigMovebaseCB...");
// 	if(msg.data)
// 		std::system("")
// }
void ProxyNode::reconfigmaxvel(const geometry_msgs::Pose2D& msg)
{
	ROS_INFO("reconfigParameterMove_base Max Vel...");
	std::stringstream ss_mx;
	ss_mx << msg.x;
	ss_mx >> max_x_str_;
	ROS_INFO_STREAM("dynamic_reconfigure move_base/max_vel_x to " << max_x_str_);
	std::system(("rosrun dynamic_reconfigure dynparam set move_base/TrajectoryPlannerROS max_vel_x " + max_x_str_).c_str());
}
void ProxyNode::rosaria_poseCB(const nav_msgs::Odometry msg)
{
	if(msg.twist.twist.linear.x==0&&msg.twist.twist.angular.z==0)
		setheading=true;
	else setheading=false;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "zsProxy");
	ros::NodeHandle n(std::string("~"));
    ProxyNode proxy(n);
    // ROS_INFO("zs: zs_proxy ini...");
	ros::MultiThreadedSpinner s(4);
 	ros::spin(s);

    return 0;
}
