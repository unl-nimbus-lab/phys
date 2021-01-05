/*
2015.9.1
RosAriaに回転を食わせてみる


インタラクションのテスト
1.kinect_v2_clientから得た関節位置情報を足し合わせて合計値を作る
2.合計値が閾値をはみ出したら減衰させる
3.値をパブリッシュする

rqt_plotで可視化して見る(もしかしたら値がでかすぎるかもしれない)
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "humans_msgs/Humans.h"
#include "geometry_msgs/Twist.h"

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

  int width,height;

  geometry_msgs::Twist cmdvel;

public:
  MyClass()
  {
    sub = nh.subscribe("/humans/recog_info", 1, &MyClass::callback, this);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    prv_val = 0;
    prv_diff = 0;
;
    width = 640;
    height = 360;

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
	double out_val = 180*output;
	cmdvel.linear.x = 0; 
	cmdvel.angular.z = out_val*M_PI/180.0; 
	pub.publish( cmdvel );

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

