/*
2015.7.13
STATE1のとき、人物が見ている方向に目を向ける
そして、たまにチラチラ人物を見る
肝心なのは、STATE1とSTATE2の違いをはっきりさせること
ようするに、目が合う前と目が合った後の変化をつけるということ

2015.7.12
見つめる時間はランダムに

2015.7.10
自律的なベース動作に対応する

RosAriaに回転動作も加える

2015.7.6
using eyeballs_msgs
*/

#include <ros/ros.h>
#include <iostream>

#include "humans_msgs/Humans.h"
#include "eyeballs_msgs/Eyeballs.h"
#include "std_msgs/Bool.h"
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

  vector<int> point_buff;
  int queue_size;
  int tolerance;
  int conf_th;
  int contact_count;
  int img_width;
  int img_height;

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
    cout << "callback" <<endl;
    now_time = ros::Time::now().toSec();
    eyeballs_msgs::Eyeballs ebs;
    blink_torf = 2;

    if( micro_motion > 0)
      micro_motion = -3;
    else
      micro_motion = 3;
      
    for(int i = 0; i < msg->human.size(); ++i)
      {
	if(msg->human[i].face.persons.size())
	  {
	    //まばたきの判定
	    if(msg->human[i].face.open_level.size())
	      {
		int open_deg0 = msg->human[i].face.open_level[0].deg;
		int open_conf0 = msg->human[i].face.open_level[0].conf;
		
		int open_deg1 = msg->human[i].face.open_level[1].deg;
		int open_conf1 = msg->human[i].face.open_level[1].conf;
		
		if((open_deg0 < 200 && open_conf0 > 100) && (open_deg1 < 200 && open_conf1 > 100))
		  {
		    blink_torf = 0;
		    ROS_ERROR("blink!");
		  }
		else
		  {
		    blink_torf = 1;
		    ROS_ERROR("no blink!");
		  }
	      }
	    else
	      blink_torf = 2;
	    
	    int dir_horizon = msg->human[i].face.direction.x;
	    int gaze_horizon = msg->human[i].face.gaze_direction.x;
	    int dir_conf = msg->human[i].face.direction.conf;
	    int gaze_conf = msg->human[i].face.gaze_direction.conf;
	    
	    cout <<"face:" << dir_horizon <<", gaze:" << gaze_horizon 
		 << ", face_conf:"<< dir_conf << ", gaze_conf:"<< gaze_conf << endl;
	    
	    point_buff.push_back(EyeContactDirAndGaze(dir_horizon, gaze_horizon, dir_conf, gaze_conf));
	    point_buff.erase(point_buff.begin());
	    
	    //現在目が合っているかどうか判定
	    bool contact_state;
	    int point_sum = accumulate(point_buff.begin(), point_buff.end(), 0);
	    cout << "point_sum:" << point_sum << endl;
	    if( point_sum > queue_size/2)
	      {
		contact_state = true;
	      }
	    else
	      {
		contact_state = false;
	      }
	    	    
	    
	    //顔が右左のどちら側にあるか判定
	    int gap;
	    int face_width = abs(msg->human[i].face.position.rt.x - msg->human[i].face.position.lt.x);
	    if((msg->human[i].face.position.rt.x - face_width/2) > img_width/2)
	      {
		cout << "face left" << endl;
		gap = 150;
	      }
	    else
	      {
		cout << "face right" << endl;
		gap = -150;
	      }


	    if(dir_horizon > 0)
	      look_motion = -150;
	    else
	      look_motion = 150;	    

	    //状態遷移
	    if(state == STATE1 || state == STATE4)
	      {
		if(contact_state)
		  {
		    state = STATE2;
		    //ebs.right.x = gap;
		    //ebs.left.x = gap;
		    //ebs.right.x = micro_motion;
		    //ebs.left.x = micro_motion;
		    count_time[2] = 0;
		  }
		else
		  {
		    //ここで相手の視線方向を見たり、相手の方を見たりする
		    //どうやって判断するか？
		    //おそらく、時間で。
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
	    else if(state == STATE2)
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
	    else if(state == STATE3)
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
	    
	    //test = test + ros::Time::now().toSec();
	    //cout << "test:"<<test<<endl;
	    fps = 1./ros::Duration(now_time-former_time).toSec();
	    cout << "now state: "<<state<<endl;
	    cout << "Duration(now-former):"<<ros::Duration(now_time-former_time).toSec()<<endl;	
	    cout << "count_time[0]:"<<count_time[0]<<", count_time[1]:"<<count_time[1]<<endl;
	    cout << "threshold_time[0]:"<<threshold_time[0]<<", threshold_time[1]:"<<threshold_time[1]<<endl;
	    cout << "fps:"<< fps <<endl;
	    
	    if(msg->human[i].face.persons.size())
	      {
		ebs.name = msg->human[i].face.persons[0].name;
		ebs.okao_id = msg->human[i].face.persons[0].okao_id;
	      }
	  }
	else
	  {
	    ROS_ERROR("face not found");
	    //顔が見つからなかった場合の処理
	    if(state != STATE4)
	      {
		state = STATE4;
		not_found = 0;
	      }
	    
	    //ebs.right.x = 200*sin(not_found*10*M_PI/180.);
	    //ebs.left.x =  200*sin(not_found*10*M_PI/180.);
	    ebs.right.x = look_motion+micro_motion;
	    ebs.left.x = look_motion+micro_motion;
	    ++not_found;
	  }
      }
    former_time = ros::Time::now().toSec();
    ebs.state = state;
    ebs.fps = fps;
    
    cout << "ebs fps:"<<ebs.fps<<endl;
    ebs.header.stamp = ros::Time::now();
    cout << "torf:"<<blink_torf<<endl;

    //まばたき検出時のみまばたき
    if(blink_torf == 0)
      ebs.blink = 0;
    else 
      ebs.blink = 1;
    
    eye_pub.publish( ebs );
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
