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
  double prv_val;
  double prv_diff;
  double now_val;
  double now_diff;
  double amp, max, min;
  double origin;
  int width,height;

  eyeballs_msgs::Eyeballs ebs;

public:
  MyClass()
  {
    sub = nh.subscribe("/humans/recog_info", 1, &MyClass::callback, this);
    //pub =  nh.advertise<std_msgs::Float64>("/pub_topic", 1); 
    pub = nh.advertise<eyeballs_msgs::Eyeballs>("/humans/eye_contact", 1);
    prv_val = 0;
    prv_diff = 0;
    amp = 1;
    max = 1;
    min = -1;
    origin = 0;
    width = 640;
    height = 360;
    //init_ok = true;
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
		geometry_msgs::Point p = msg->human[i].body.joints[j].position_color_space;

		if( p.x > 0 )
		  {
		    double sum_tmp = (p.x - (double)width/2.);
		    //cout <<"name:" <<msg->human[i].body.joints[j].joint_name 
		    //	 <<", p.x:" <<p.x <<", sum_tmp:"<< sum_tmp <<endl;
		    ++sum_num;
		    sum_pos += sum_tmp;
		  } 
	      }
	  }
      }

    double output = (sum_pos/sum_num) / ((double)width/2.);

    if(!isnan(output))
      {
	/*
	if( output > MAX )
	  output = MAX;
	if( output < MIN )
	  output = MIN;
	*/
	double out_val = 200*output;

	/*
	if( out_val > 200 )
	  out_val = 200;
	if( out_val < -200 )
	  out_val = -200;
	*/

	ebs.right.x = ebs.left.x = out_val;
	ebs.state = 3;
	ebs.blink = 1;
	//cout << ebs.right.x <<", " << ebs.left.x << endl;
	pub.publish( ebs );

	prv_val = now_val;
	prv_diff = now_diff;
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

