#include <ros/ros.h>
#include <iostream>

#include "humans_msgs/Humans.h"
#include "std_msgs/Bool.h"
#include <fstream>
#include <functional>
#include <algorithm>
#include <numeric>

#define STATE1 1
#define STATE2 2
#define STATE3 3

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

  double count_time[2];
  double threshold_time[2];
  double test;

  stringstream file_name;
  int state;
  ros::Time former_time;
  ros::Time now_time;
  //ofstream ofs;

public:
  EyeContact()
  {   
    state = STATE1; 
    contact_count = 0;

    queue_size = 10;


    count_time[0] = 0;
    count_time[1] = 0;

    //見つめる時間
    threshold_time[0] = ros::Duration(1.0).toSec();

    //目をそらす時間
    threshold_time[1] = ros::Duration(0.5).toSec();

    for(int i = 0; i < queue_size; ++i)
      point_buff.push_back(0);
    
    tolerance = 7;
    conf_th = 100;
    eye_sub = nh.subscribe("/humans/okao_server", 1, 
			   &EyeContact::Callback, this);

    eye_pub = nh.advertise<std_msgs::Bool>("/humans/eye_contact", 1);
    former_time = ros::Time::now();
  }
  ~EyeContact()
  {

  }

  void Callback(const humans_msgs::HumansConstPtr& msg)
  {
    now_time = ros::Time::now();
    std_msgs::Bool torf;
    for(int i = 0; i < msg->human.size(); ++i)
      {
	int dir_horizon = msg->human[i].face.direction.x;
	int gaze_horizon = msg->human[i].face.gaze_direction.x;
	int dir_conf = msg->human[i].face.direction.conf;
	int gaze_conf = msg->human[i].face.gaze_direction.conf;

	cout <<"face:" << dir_horizon <<", gaze:" << gaze_horizon 
	     << ", face_conf:"<< dir_conf << ", gaze_conf:"<< gaze_conf << endl;

	point_buff.push_back(PointEyeContactDirAndGaze(dir_horizon, gaze_horizon, dir_conf, gaze_conf));
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


	//状態遷移
	bool mode;
	if(state == STATE1)
	  {
	    if(contact_state)
	      {
		state = STATE2;
		mode = true;
	      }
	    else
	      {
		state = STATE1;
		mode = false;
	      }  
	  }
	else if(state == STATE2)
	  {
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
	    mode = false;
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
	    mode = true;
	  }

	test = test + ros::Time::now().toSec();
	cout << "test:"<<test<<endl;

	cout << "now state: "<<state<<endl;
	/*
	cout << "Duration(1.0):"<<ros::Duration(1.0).toSec()<<endl;
	*/
	cout << "Duration(now-former):"<<ros::Duration(now_time-former_time).toSec()<<endl;
	
	cout << "count_time[0]:"<<count_time[0]<<", count_time[1]:"<<count_time[1]<<endl;
	cout << "threshold_time[0]:"<<threshold_time[0]<<", threshold_time[1]:"<<threshold_time[1]<<endl;
	cout << "fps:"<<1./ros::Duration(now_time-former_time).toSec()<<endl;

	former_time = ros::Time::now();
	torf.data= mode;	
       
	eye_pub.publish( torf );
      }
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


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eye_contact");
  
  EyeContact ec;
  
  ros::spin();
  return 0;
}
