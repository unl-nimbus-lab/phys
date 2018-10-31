/*
インタラクションのテスト
1.kinect_v2_clientから得た関節位置情報を足し合わせて合計値を作る
2.合計値が閾値をはみ出したら減衰させる
3.値をパブリッシュする

rqt_plotで可視化して見る(もしかしたら値がでかすぎるかもしれない)
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "humans_msgs/Humans.h"
#include "eyeballs_msgs/Eyeballs.h"

#define MAX 1
#define MIN -1

using namespace std;

class MyClass
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;  
  double init_val;
  double max_val;
  bool init_ok;
  eyeballs_msgs::Eyeballs ebs;

public:
  MyClass()
  {
    sub = nh.subscribe("/humans/recog_info", 1, &MyClass::callback, this);
    //pub =  nh.advertise<std_msgs::Float64>("/pub_topic", 1); 
    pub = nh.advertise<eyeballs_msgs::Eyeballs>("/humans/eye_contact", 1);

    init_ok = true;
  }
  
  ~MyClass()
  {

  }

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {
    int sum_num = 0;
    double sum_pos = 0;
    for(int i = 0; i < msg->human.size(); ++i)
      {
	if( i == 0 )
	  {
	    for(int j = 0; j < msg->human[i].body.joints.size(); ++j)
	      {
		//double sum_tmp = func_sum_pos(msg->human[i].body.joints[j].position, j);
		geometry_msgs::Point p = msg->human[i].body.joints[j].position;
		double sum_tmp = p.x + p.y + p.z;
		//cout <<sum_tmp<<endl;
		if( sum_tmp )
		  {
		    ++sum_num;
		    sum_pos += sum_tmp;
		  } 
	      }
	  }
      }
    double output = sum_pos/sum_num;
    if(!isnan(output))
      {
	/*
	if( init_ok )
	  {
	    init_val = output;
	    max_val = fabs(output);
	    init_ok = false;
	  }
	if(max_val < fabs(output))
	  max_val = fabs(output);
	*/
	if( output > MAX )
	  output = MAX;
	if( output < MIN )
	  output = MIN;

	//std_msgs::Float64 msg_str;
	//msg_str.data = (output - init_val)/max_val;
	//msg_str.data = output;
	//ROS_INFO("output:%f, sum_pos:%f, max_val:%f", msg_str.data, sum_pos, max_val);
	ebs.right.x = ebs.left.x = 200*output;
	ebs.state = 3;
	ebs.blink = 1;
	cout << ebs.right.x <<", " << ebs.left.x << endl;
	pub.publish( ebs );
      }
    else
      {
	init_ok = true;
      }
  }

  /*
  double func_sum_pos(geometry_msgs::Point p, int j)
  {
    if(j!=7 || j!=11 || j!=21 || j!=22 || j!=23 || j!=24 || j!=13 || j!=14 || j!=15 || j!=17 || j!=18 || j!=19)
      {
	return p.x + p.y + p.z;
      }
    else
      return 0;
  } 
  */
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_node");
  MyClass mc;
  ros::spin();
  return 0;
}

