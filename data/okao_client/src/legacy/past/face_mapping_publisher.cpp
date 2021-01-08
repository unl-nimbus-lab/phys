/*
2015.1.15-----------------
やはり、service方式でデータを受け取る方がよい
なぜなら、パブサブだと、データが来たときにしか更新されない
そうじゃなくて、今、この瞬間の状態がどうなっているかを知りたい

 
2014.8.29-----------------
pub/sub方式で表示する
point background

2014.6.13-----------------
srv

2014.6.6-------------------
搭載する仕様の予定

1.顔の位置に点をポイント
2.名前表示
　今見ているヒト→みずいろ
　過去に見たヒト→おれんじ
3.人らしき物体（NiTEトラッキングok,OkAOVision is No!）のマッピング




*/


#include <ros/ros.h>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

//#include <math.h>
#include "MsgToMsg.hpp"

//オリジナルのメッセージ
#include <humans_msgs/PersonPoseImgArray.h>
#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"
#include "humans_msgs/HumansSrv.h"

using namespace std;

map<int, humans_msgs::PersonPoseImg> dbhuman;
map<int, humans_msgs::PersonPoseImg> pastDbhuman;

class FaceMappingPub
{
private:
  ros::NodeHandle n;
  ros::Publisher face_now_pub_ ;
  ros::Publisher face_past_pub_ ;
  ros::ServiceClient okaoStack_;
  ros::Subscriber recogInfo_sub_;


public:
  FaceMappingPub()
  {
    face_now_pub_
      = n.advertise<humans_msgs::PersonPoseImgArray
		    >("/now_human", 10);
    face_past_pub_
      = n.advertise<humans_msgs::PersonPoseImgArray
		    >("/past_human", 10);
    okaoStack_
      = n.serviceClient<okao_client::OkaoStack>("stack_send");
    recogInfo_sub_ 
      = n.subscribe("/humans/RecogInfo", 1, &FaceMappingPub::callback, this);
  }

  ~FaceMappingPub()
  {
    dbhuman.clear();
    pastDbhuman.clear();
  }

  void callback(const humans_msgs::HumansConstPtr& rein)
  {

    humans_msgs::PersonPoseImgArray ppia, ppia_past;
    if(rein->num)
      {
	for(int i = 0; i < rein->num; ++i)
	  {
	    humans_msgs::PersonPoseImg ppi;
	    okao_client::OkaoStack stack;
	    
	    stack.request.person.okao_id = rein->human[i].max_okao_id;
	    okaoStack_.call( stack );
	    
	    ppi.person.hist = rein->human[i].max_hist;
	    ppi.person.okao_id = rein->human[i].max_okao_id;
	    ppi.person.name = stack.response.person.name;
	    ppi.pose.position = rein->human[i].p;
	    ppi.pose.orientation.w = 1;

	    ppi.image = stack.response.image;
	    ppi.header.stamp = ros::Time::now();
	    ppi.header.frame_id = rein->human[i].header.frame_id;
	    dbhuman[rein->human[i].max_okao_id] = ppi;
	  }
      }
    //cout<< "input ok" << endl;

    map<int, humans_msgs::PersonPoseImg>::iterator it = dbhuman.begin();
    ros::Time now = ros::Time::now();
    //if(dbhuman.size())
    //  {
    int test = 0;
    while(it != dbhuman.end())
      {
	//timeout
	double timeout = 10.0;
	if ((now - it->second.header.stamp).toSec() > timeout)
	  {
	    /*
	    humans_msgs::Person person;
	    person = it->second.person;
	    sensor_msgs::Image gImg;
	    cv_bridge::CvImage cvImg;
	    cvImg = it->second.image;
	    cv::cvtColor(cvImg.image, cvImg.image, CV_RGB2GRAY);
	    gImg.height = cvImg.rows;
	    gImg.width = cvImg.cols;
	    gImg.encoding = "mono16";
gImg.step = cvImg.data
	    */
	    ppia_past.ppis.push_back( it->second );
	    //eraseする
	    //cout << it->first << " is erase" <<endl;
	    //dbhuman.erase( it->first );
	  }
	else
	  {
	    //cout << it->second.person << endl;
	    ppia.ppis.push_back( it->second );
	  }
	++it; 
	++test;
      }
    // }
    
    //cout<<test <<endl;
    ppia.header.stamp = ros::Time::now();
    ppia.header.frame_id = "map";
    //.num = dbhumans.response.dst.num;
    face_now_pub_.publish( ppia );
    face_past_pub_.publish( ppia_past );
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_mapping_publisher");
  FaceMappingPub pmp;
  ros::spin();
  return 0;
}



