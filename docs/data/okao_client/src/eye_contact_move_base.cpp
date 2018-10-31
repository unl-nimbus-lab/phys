
/*
2015.7.26

移動中はカウントしない

狙ったとおりに動かないのだとしたら何が原因だと考えられるか？

そもそも、どういったことが生じているか
1.最初は、向いている方向に止まるが、次第に、微妙なズレで終わる
2.stateのとおりに動いているか?
つまり、今は、行く回転、今は、戻る回転、などという変化が、正しく行われているか？


2015.7.23
move_baseを介して回転してみる
ためしに


*/

#include "eye_contact.h"


#define STATE1 1
#define STATE2 2
#define STATE3 3
#define STATE4 4

using namespace std;

namespace eye_contact {

  EyeContact:: EyeContact()
  {   

    state = STATE1; 
    move_state = false;
    contact_count = 0;
    contact_state = false;
    queue_size = 10;

    count_time[0] = 0;
    count_time[1] = 0;
    count_time[2] = 0;
    count_time[3] = 0;
    tm_param[0] = 3.0;
    tm_param[1] = 2.5;
    tm_param[2] = 0.5;
    tm_param[3] = 3.0;

    //見つめる時間
    threshold_time[0] = ros::Duration(tm_param[0]).toSec();
    //目をそらす時間
    threshold_time[1] = ros::Duration(tm_param[1]).toSec();
    //相手に体を向ける時間
    threshold_time[2] = ros::Duration(tm_param[2]).toSec();
    //相手の視線の向きに体を向ける時間
    threshold_time[3] = ros::Duration(tm_param[3]).toSec();

    for(int i = 0; i < queue_size; ++i)
      point_buff.push_back(0);
    
    dir_horizon = 0;
    gaze_horizon = 0;
    dir_conf = 0;
    gaze_conf = 0;

    tolerance = 7;
    conf_th = 100;

    img_width = 640;
    img_height = 360;

    micro_motion = 0;
    look_motion = 0;
    not_found = 0;

    eye_sub = nh.subscribe("/humans/okao_server", 1, 
			   &EyeContact::Callback, this);

    eye_pub = nh.advertise<eyeballs_msgs::Eyeballs>("/humans/eye_contact", 1);

    //set up the move thread
    move_thread = new boost::thread(boost::bind(&EyeContact::moveThread, this));
    //for comanding the base
    vel_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
    
    former_time = ros::Time::now().toSec();
  }
  
  EyeContact::~EyeContact()
  {
    move_thread->interrupt();
    move_thread->join();
    delete move_thread;
  }

  void EyeContact::Callback(const humans_msgs::HumansConstPtr& msg)
  {
    now_time = ros::Time::now().toSec();

    blink_torf = 2;

    micro_motion_switch(micro_motion);

      
    for(int i = 0; i < msg->human.size(); ++i)
      {
	//顔を検出したかどうかの判定
	if(msg->human[i].face.persons.size())
	  {

	    //まばたきの判定
	    blink_torf = blink_check( msg->human[i].face.open_level );

	    //目が合ったかどうかの判定
	    contact_state = contact_check(msg->human[i].face.direction, msg->human[i].face.gaze_direction);
	    
	    //顔が右左のどちら側にあるか判定
	    int face_width = abs(msg->human[i].face.position.rt.x - msg->human[i].face.position.lt.x);
	    gap = face_right_and_left_check(msg->human[i].face.position.rt.x - face_width/2);

	    //名前とIDののセット
	    name_and_id_check(msg->human[i].face.persons);

	  }
	else
	  {
	    //目が合ったかどうかの判定
	    humans_msgs::Direction dir;
	    humans_msgs::XYConf gaze_dir;
	    dir.conf = 0;
	    gaze_dir.conf = 0;
	    contact_state = contact_check_noface(dir, gaze_dir);
	  }

	//現在のアイコンタクト状態判定および瞳の動き設定
	eyecontact_check(state);
		
	//各種表示
	fps = 1./ros::Duration(now_time-former_time).toSec();
	
	cout << "now state: "<<state<< ",persons.size:" << msg->human.size() << endl;
	  /*
	  cout << "Duration(now-former):"<<ros::Duration(now_time-former_time).toSec()<<endl;	
	  cout << "count_time[0]:"<<count_time[0]
	  <<", count_time[1]:"<<count_time[1]
	  <<", count_time[2]:"<<count_time[2]<<endl;
	  cout << "threshold_time[0]:"<<threshold_time[0]
	  <<", threshold_time[1]:"<<threshold_time[1]
	  <<", threshold_time[2]:"<<threshold_time[2]<<endl;
	  cout << "fps:"<< fps <<endl;
	  */
      }

    former_time = ros::Time::now().toSec();
    ebs.state = state;
    ebs.fps = fps;
    
    ebs.header.stamp = ros::Time::now();
    //cout <<"blink:"<<blink_torf<<endl;
    ebs.blink = blink_torf;

    eye_pub.publish( ebs );
  }

  void EyeContact::micro_motion_switch(int now_micro_motion)
  {
    if( now_micro_motion > 0)
      micro_motion = -3;
    else
      micro_motion = 3;
  }

  int EyeContact::blink_check(vector<humans_msgs::DegConf> open_level)
  {
    if(open_level.size())
      {
	int open_deg0 = open_level[0].deg;
	int open_conf0 = open_level[0].conf;
	
	int open_deg1 = open_level[1].deg;
	int open_conf1 = open_level[1].conf;
	
	if((open_deg0 < 200 && open_conf0 > 100) && (open_deg1 < 200 && open_conf1 > 100))
	  {
	    //ROS_INFO("blink!");
	    return 0;
	  }
	else
	  {
	    //ROS_ERROR("no blink!");
	    return 1;
	  }
      }
    else
      {
	//ROS_ERROR("no get blink!");
	return 2;
      }
  }

  bool EyeContact::contact_check(humans_msgs::Direction dir, humans_msgs::XYConf gaze_dir)
  {
    dir_horizon = dir.x;
    gaze_horizon = gaze_dir.x;
    dir_conf = dir.conf;
    gaze_conf = gaze_dir.conf;

    //視線方向のセッティング
    if(dir_horizon > 0)
      look_motion = -150;
    else
      look_motion = 150;    

    /*
    cout <<"face:" << dir_horizon <<", gaze:" << gaze_horizon 
	 << ", face_conf:"<< dir_conf << ", gaze_conf:"<< gaze_conf << endl;
    */
    point_buff.push_back(EyeContactDirAndGaze(dir_horizon, gaze_horizon, dir_conf, gaze_conf));
    point_buff.erase(point_buff.begin());
    

    int point_sum = accumulate(point_buff.begin(), point_buff.end(), 0);
    //cout << "point_sum:" << point_sum << endl;
    if( point_sum > queue_size/2 )
      {
	return true;
      }
    else
      {
	return false;
      }
  } 

  bool EyeContact::contact_check_noface(humans_msgs::Direction dir, humans_msgs::XYConf gaze_dir)
  {
    point_buff.push_back(EyeContactDirAndGaze(0, 0, 0, 0));
    point_buff.erase(point_buff.begin());

    int point_sum = accumulate(point_buff.begin(), point_buff.end(), 0);
    //cout << "point_sum:" << point_sum << endl;
    if( point_sum > queue_size/2 )
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  int EyeContact::face_right_and_left_check(int right_or_left)
  { 
    if(right_or_left > img_width/2)
      {
	//cout << "face left" << endl;
	return 150;
      }
    else
      {
	//cout << "face right" << endl;
	return -150;
      }
  }

  void EyeContact::eyecontact_check(int now_state)
  {
    double diff = ros::Duration(now_time-former_time).toSec();
    //状態遷移
    switch (now_state)
      {
      case 1:
      //アイコンタクト成立ならstate2へ
	if(contact_state)
	  {
	    state = STATE2;
	    count_time[0] = 0;
	  }
	else
	  {
	    //動いていないときは、そもそもstate4は必要ない
	    if(!move_state)
	      {
		count_time[0] = count_time[0] + diff;
		if( count_time[0] > threshold_time[0] )
		  {
		    state = STATE4;	
		    move_state = true;
		    count_time[0] = 0;
		  }
		else
		  {
		    state = STATE1;
		  }
	      }


	    ebs.right.x = micro_motion;
	    ebs.left.x = micro_motion;
	  }  
      break;

      case 2:

	if( !contact_state )
	  {
	    state = STATE1;
	    count_time[1] = 0;
	  }
	else
	  {
	    count_time[1] = count_time[1] + diff;
	    if( count_time[1] > threshold_time[1] )
	      {
		state = STATE3;
		count_time[1] = 0;
	      }
	    else
	      {
		state = STATE2;
	      }
	  }
	ebs.right.x = micro_motion;
	ebs.left.x = micro_motion;
	break;
      
      case 3:
	double deci, inte;
	if( !contact_state )
	  {
	    state = STATE1;
	    count_time[2] = 0;
	  }
	else
	  {
	    count_time[2] = count_time[2] + diff;
	    if( count_time[2] > threshold_time[2] )
	      {
		state = STATE2;
		count_time[2] = 0;
	      }
	    else
	      {
		state = STATE3;
	      }
	  }
	ebs.right.x = gap + micro_motion;
	ebs.left.x = gap + micro_motion;
	//見つめる時間の変更
	deci = 0.1*GetRandom(0,9);
	inte = GetRandom(1,4);
	tm_param[1] = inte + deci;
	threshold_time[1] =  ros::Duration(tm_param[1]).toSec();
	break;

      case 4:
	
	if( contact_state )
	  {
	    state = STATE1;
	    count_time[3] = 0;
	  }
	else
	  {	    
	    if(!move_state)
	      {
		count_time[3] = count_time[3] + diff;
		if( count_time[3] > threshold_time[3] )
		  {
		    state = STATE1;
		    move_state = true;	
		    count_time[3] = 0;
		  }
		else
		  {
		    state = STATE4;
		  }
	      }

	  }
	ebs.right.x = micro_motion;
	ebs.left.x = micro_motion;
	break;

      }
  }

  void EyeContact::name_and_id_check(vector<humans_msgs::Person> persons)
  {
    if(persons.size())
      {
	ebs.name = persons[0].name;
	ebs.okao_id = persons[0].okao_id;
      }
    else
      {
	ebs.name = "undefined";
	ebs.okao_id = 0;
      }
  }


  void EyeContact::face_not_found_case(int now_state)
  {
    if(state != STATE4)
      {
	state = STATE4;
	not_found = 0;
      }
    
    //ebs.right.x = 200*sin(not_found*10*M_PI/180.);
    //ebs.left.x =  200*sin(not_found*10*M_PI/180.);
    ebs.right.x = micro_motion;
    ebs.left.x = micro_motion;
    cout << "not_found:"<<not_found <<endl;
    ++not_found;
  }

  int EyeContact::GetRandom(int min, int max)
  {
    return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
  }

  int EyeContact::PointEyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf)
  {
    //cout <<"horizon:" <<horizon << ", conf:"<< conf<< endl;
    int diagonally_point = queue_size/3;
    int straight_point = 1;
    int zero_point = 0;
    if(f_conf>100 && g_conf>100)
      {
	if( f_horizon < 0 && g_horizon > 0 )
	  {
	    return diagonally_point;
	  }    
	else if( f_horizon > 0 && g_horizon < 0 )
	  {
	    return diagonally_point;
	  }
	else if(abs(f_horizon) < tolerance && tolerance>abs(abs(abs(f_horizon) - abs(g_horizon))))
	  {
	    return straight_point;
	  }
	else
	  return zero_point;
      }
    else 
      return zero_point;
  }

  int EyeContact::EyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf)
  {
    //cout <<"horizon:" <<horizon << ", conf:"<< conf<< endl;
    int diagonally_point = queue_size/3;
    int straight_point = 1;
    int zero_point = 0;
    if(f_conf>100 && g_conf>100)
      {
	if(abs(f_horizon) < tolerance)
	  {
	    return diagonally_point;
	  }
	else
	  return zero_point;
      }
    else 
      return zero_point;
  }

 //動作部
  void EyeContact::moveThread()
  { 
    ros::NodeHandle nmv;
    ros::Rate rate(30);
    bool state1_back = false;

    geometry_msgs::Point origin_point;
    geometry_msgs::Quaternion origin_quat;

    double roll, pitch, yaw;

    now_pose =
      ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose");

    origin_point = now_pose->pose.pose.position;
    origin_quat = now_pose->pose.pose.orientation;

    GetRPY(origin_quat, roll, pitch, yaw);

    double rot = 0;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
      move_state = false;
    }

    bool send_goal = true;

    while(nmv.ok())
      {
	if( move_state )
	  { 
	    geometry_msgs::Twist cmd_vel;
	    move_base_msgs::MoveBaseGoal goal;

	    if(dir_horizon > 0)
	      {
		rot = -180.+dir_horizon;
	      }
	    else if(dir_horizon < 0)
	      {
		rot = 180.+dir_horizon;
	      }
	    else
	      rot = 0;

	    goal.target_pose.header.frame_id = "map";
	    goal.target_pose.header.stamp = ros::Time::now();
	    
	    if(state==STATE1)
	      {	    
		goal.target_pose.pose.position = origin_point;
		goal.target_pose.pose.orientation = origin_quat;
	      }
	    else if(state==STATE4)
	      {
		goal.target_pose.pose.position = origin_point;
		double now_th = yaw/2.*(M_PI/180.); 
		double del_th = rot/2.*(M_PI/180.);
		  
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw((yaw+rot)*M_PI/180.);
	      }

	    if(send_goal)
	      {
		ac.sendGoal(goal);
		send_goal = false;
	      }	    
	    else
	      {
		if(state==STATE2 || state==STATE3)
		  {
		    ROS_INFO("rotation stop because face get");
		    ac.cancelAllGoals();
		    move_state = false;
		    send_goal = true;
		    now_pose =
		      ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose");
		    
		    origin_point = now_pose->pose.pose.position;
		    origin_quat = now_pose->pose.pose.orientation;
		    GetRPY(origin_quat, roll, pitch, yaw);
		  }

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		  {	    
		    ROS_INFO("OK");
		    move_state = false;
		    send_goal = true;
		  }
		else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
		  {
		    ROS_ERROR("MISS");
		    move_state = false;
		    send_goal = true;
		  }
	      }
	  }	

	rate.sleep();	
      }
  }
  
  void EyeContact::GetRPY(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw)
  {
    tf::Quaternion btq(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(btq).getRPY(roll, pitch, yaw);
  }
  
  void EyeContact::GetTfRPY(const tf::Quaternion btq, double &roll,double &pitch,double &yaw)
  {
    tf::Matrix3x3(btq).getRPY(roll, pitch, yaw);
  }
  /*
  void EyeContact::GetQtFromYaw(geometry_msgs::Quaternion &quat, double yaw)
  {
    quat = tf::createQuaternionMsgFromYaw(yaw);
  }
  */
  bool EyeContact::qtRoughlyEq(geometry_msgs::Quaternion src1, geometry_msgs::Quaternion src2)
  {
    double tol = 0.1;
    if( (fabs(src1.w - src2.w) < tol) &&  (fabs(src1.z - src2.z) < tol))
      {
	return true;
      }
    return false;
  }
  
  void EyeContact::publishZeroVelocity()
  {
    geometry_msgs::Twist cmd_vel;
    //cmd_vel.linear.x = 0.0;
    //cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub.publish(cmd_vel);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "eye_contact");
  
  eye_contact::EyeContact ec;
  
  ros::spin();
  return 0;
}
