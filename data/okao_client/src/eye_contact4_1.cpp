/*
2015.7.13

move_base経由でRosAriaに回転動作も加える

各要所を関数化する

STATE1のとき、人物が見ている方向に目を向ける
そして、たまにチラチラ人物を見る
肝心なのは、STATE1とSTATE2の違いをはっきりさせること
ようするに、目が合う前と目が合った後の変化をつけるということ

2015.7.12
見つめる時間はランダムに

2015.7.10
自律的なベース動作に対応する

2015.7.6
using eyeballs_msgs
*/

#include <ros/ros.h>
#include <iostream>

#include "humans_msgs/Humans.h"
#include "eyeballs_msgs/Eyeballs.h"

#include <fstream>
#include <functional>
#include <algorithm>
#include <numeric>

#define STATE1 1
#define STATE2 2
#define STATE3 3
#define STATE4 4

using namespace std;

class EyeContact
{
private:
  ros::NodeHandle nh;
  ros::Subscriber eye_sub;
  ros::Publisher eye_pub;
  eyeballs_msgs::Eyeballs ebs;

  vector<int> point_buff;
  int queue_size;
  int tolerance;
  int conf_th;
  int contact_count;
  int img_width;
  int img_height;
  //現在目が合っているかどうか判定
  bool contact_state;
  int gap;

  double count_time[3];
  double threshold_time[3];
  double tm_param[2];
  double test,fps;

  stringstream file_name;
  int state;
  double former_time, now_time;

  int micro_motion;
  int look_motion;
  int blink_torf;
  int not_found;
  //ofstream ofs;

public:
  EyeContact()
  {   
    state = STATE1; 
    contact_count = 0;

    queue_size = 10;

    count_time[0] = 0;
    count_time[1] = 0;
    count_time[2] = 0;
    tm_param[0] = 2.0;
    tm_param[1] = 0.5;
    tm_param[2] = 4.0;
    //見つめる時間
    threshold_time[0] = ros::Duration(tm_param[0]).toSec();
    //目をそらす時間
    threshold_time[1] = ros::Duration(tm_param[1]).toSec();
    //相手の視線の向きに目を向ける時間
    threshold_time[2] = ros::Duration(tm_param[2]).toSec();

    for(int i = 0; i < queue_size; ++i)
      point_buff.push_back(0);
    
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
    former_time = ros::Time::now().toSec();
  }
  ~EyeContact()
  {

  }

  void Callback(const humans_msgs::HumansConstPtr& msg)
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

	    //現在のアイコンタクト状態判定および瞳の動き設定
	    eyecontact_check(state);

	    //名前とIDののセット
	    name_and_id_check(msg->human[i].face.persons);
       
	    //各種表示
	    fps = 1./ros::Duration(now_time-former_time).toSec();
	    cout << "now state: "<<state<<endl;
	    cout << "Duration(now-former):"<<ros::Duration(now_time-former_time).toSec()<<endl;	
	    cout << "count_time[0]:"<<count_time[0]
		 <<", count_time[1]:"<<count_time[1]
		 <<", count_time[2]:"<<count_time[2]<<endl;
	    cout << "threshold_time[0]:"<<threshold_time[0]
		 <<", threshold_time[1]:"<<threshold_time[1]
		 <<", threshold_time[2]:"<<threshold_time[2]<<endl;
	    cout << "fps:"<< fps <<endl;
	  }
	else
	  {
	    ROS_ERROR("face not found");
	    //顔が見つからなかった場合の瞳の動き設定
	    face_not_found_case(state);

	  }
      }

    former_time = ros::Time::now().toSec();
    ebs.state = state;
    ebs.fps = fps;
    
    ebs.header.stamp = ros::Time::now();
    cout <<"blink:"<<blink_torf<<endl;
    ebs.blink = blink_torf;

    eye_pub.publish( ebs );
  }

  void micro_motion_switch(int now_micro_motion)
  {
    if( now_micro_motion > 0)
      micro_motion = -3;
    else
      micro_motion = 3;
  }

  int blink_check(vector<humans_msgs::DegConf> open_level)
  {
    if(open_level.size())
      {
	int open_deg0 = open_level[0].deg;
	int open_conf0 = open_level[0].conf;
	
	int open_deg1 = open_level[1].deg;
	int open_conf1 = open_level[1].conf;
	
	if((open_deg0 < 200 && open_conf0 > 100) && (open_deg1 < 200 && open_conf1 > 100))
	  {
	    ROS_INFO("blink!");
	    return 0;
	  }
	else
	  {
	    ROS_ERROR("no blink!");
	    return 1;
	  }
      }
    else
      {
	ROS_ERROR("no get blink!");
	return 2;
      }
  }

  bool contact_check(humans_msgs::Direction dir, humans_msgs::XYConf gaze_dir)
  {
    int dir_horizon = dir.x;
    int gaze_horizon = gaze_dir.x;
    int dir_conf = dir.conf;
    int gaze_conf = gaze_dir.conf;

    //視線方向のセッティング
    if(dir_horizon > 0)
      look_motion = -150;
    else
      look_motion = 150;    

    cout <<"face:" << dir_horizon <<", gaze:" << gaze_horizon 
	 << ", face_conf:"<< dir_conf << ", gaze_conf:"<< gaze_conf << endl;
    
    point_buff.push_back(EyeContactDirAndGaze(dir_horizon, gaze_horizon, dir_conf, gaze_conf));
    point_buff.erase(point_buff.begin());
    

    int point_sum = accumulate(point_buff.begin(), point_buff.end(), 0);
    cout << "point_sum:" << point_sum << endl;
    if( point_sum > queue_size/2)
      {
	return true;
      }
    else
      {
	return false;
      }
  } 
  
  int face_right_and_left_check(int right_or_left)
  { 
    if(right_or_left > img_width/2)
      {
	cout << "face left" << endl;
	return 150;
      }
    else
      {
	cout << "face right" << endl;
	return -150;
      }
  }

  void eyecontact_check(int now_state)
  {
    //状態遷移
    if(now_state == STATE1 || now_state == STATE4)
      {
	if(contact_state)
	  {
	    state = STATE2;
	    count_time[2] = 0;
	  }
	else
	  {
	    //ここで相手の視線方向を見たり、相手の方を見たりする
	    count_time[2] = count_time[2] + ros::Duration(now_time-former_time).toSec();
	    if( count_time[2] < threshold_time[2] )
	      {
		
	      }
	    else if(count_time[2] > threshold_time[2] && count_time[2] < (threshold_time[1] + threshold_time[2]) )
	      {
		look_motion = micro_motion;
	      }
	    else
	      {
		count_time[2] = 0;
	      }
	    state = STATE1;
	    ebs.right.x = look_motion+micro_motion;
	    ebs.left.x = look_motion+micro_motion;
	  }  
      }
    else if(now_state == STATE2)
      {
	//state2になったときに目をそらす時間count_time[1]はリセット。
	//見つめる時間count_time[0]はインクリメント
	count_time[1] = 0;
	count_time[0] = count_time[0] + ros::Duration(now_time-former_time).toSec();
	if( !contact_state )
	  state = STATE1;
	else
	  {
	    if( count_time[0] > threshold_time[0] )
	      state = STATE3;
	    else
	      state = STATE2;
	  }
	ebs.right.x = micro_motion;
	ebs.left.x = micro_motion;
      }
    else if(now_state == STATE3)
      {
	count_time[0] = 0;
	count_time[1] = count_time[1] + ros::Duration(now_time-former_time).toSec();
	
	if( !contact_state )
	  state = STATE1;
	else
	  {
	    if( count_time[1] > threshold_time[1] )
	      state = STATE2;
	    else
	      state = STATE3;
	  }
	ebs.right.x = gap + micro_motion;
	ebs.left.x = gap + micro_motion;
	//見つめる時間の変更
	double deci = 0.1*GetRandom(0,9);
	double inte = GetRandom(1,4);
	tm_param[0] = inte + deci;
	threshold_time[0] =  ros::Duration(tm_param[0]).toSec();
      }
    
  }

  void name_and_id_check(vector<humans_msgs::Person> persons)
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

  void face_not_found_case(int now_state)
  {
    if(state != STATE4)
      {
	state = STATE4;
	not_found = 0;
      }
    
    ebs.right.x = 200*sin(not_found*10*M_PI/180.);
    ebs.left.x =  200*sin(not_found*10*M_PI/180.);
    //ebs.right.x = look_motion+micro_motion;
    //ebs.left.x = look_motion+micro_motion;
    cout << "not_found:"<<not_found <<endl;
    ++not_found;
  }

  int GetRandom(int min, int max)
  {
    return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
  }

  int PointEyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf)
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

  int EyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf)
  {
    //cout <<"horizon:" <<horizon << ", conf:"<< conf<< endl;
    int diagonally_point = queue_size/3;
    int straight_point = 1;
    int zero_point = 0;
    if(f_conf>100 && g_conf>100)
      {
	if(abs(f_horizon) < tolerance && tolerance>abs(abs(abs(f_horizon) - abs(g_horizon))))
	  {
	    return diagonally_point;
	  }
	else
	  return zero_point;
      }
    else 
      return zero_point;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eye_contact");
  
  EyeContact ec;
  
  ros::spin();
  return 0;
}
