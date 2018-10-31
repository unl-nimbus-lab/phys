/*
2014.12.5-----------
tracking data mapping for kinectv2

 */

#include <ros/ros.h>

#include <turtlesim/Spawn.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

//#include <math.h>
#include "MsgToMsg.hpp"

//オリジナルのメッセージ
#include <humans_msgs/Humans.h>
#include "okao_client/OkaoStack.h"

using namespace std;

class HPass
{
public:
  HPass()
    : rein_sub_( nh_, "/humans/RecogInfo", 100 ),
      track_sub_( nh_, "/humans/OkaoServerNot", 100 ),
      sync( MySyncPolicy( 10 ), rein_sub_, track_sub_ )
  { 
    sync.registerCallback( boost::bind( &HPass::callback, this, _1, _2 ) );
    path_pub_ = nh_.advertise<nav_msgs::Path>("/humans/path", 10);
  }
  ~HPass()
  {
  }

  void callback(const humans_msgs::HumansConstPtr& rein, 
		const humans_msgs::HumansConstPtr& track)
  {



  }


private:
  ros::NodeHandle nh_;
ros::Publisher path_pub_;
  typedef message_filters::Subscriber< humans_msgs::Humans > HumansSub;
  HumansSub rein_sub_, track_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    humans_msgs::Humans, humans_msgs::Humans
    > MySyncPolicy;

  message_filters::Synchronizer< MySyncPolicy > sync;
};








int main(int argc, char** argv)
{
  ros::init(argc, argv, "humans_path");
  ros::NodeHandle n;
  ros::Publisher ma_pub_ = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
  //ros::Publisher path_pub_;
  ros::Subscriber path_sub_;
  map<long, geometry_msgs::Point> path_buf;

  while(ros::ok)
    {
      cout <<"test"<<endl;
      humans_msgs::HumansConstPtr track = ros::topic::waitForMessage<humans_msgs::Humans>("/humans/RecogInfo");
      humans_msgs::HumansConstPtr tracknf = ros::topic::waitForMessage<humans_msgs::Humans>("/humans/OkaoServerNot");
      //認識人数の取得
      int p_num = track->num;
      ros::Time map_time = ros::Time::now();      
      
      visualization_msgs::MarkerArray ma;
      
      //humans_msgs::Humans tp;
      for(int i = 0; i < p_num ; ++i)
	{

	  visualization_msgs::Marker m;
	  m.header.frame_id =  "/map";
	  m.header.stamp = map_time;
	  m.id = track->human[i].body.tracking_id;
	  
	  m.ns = "path";
	  m.type = m.LINE_STRIP;
	  m.action = 0;
	  
	  cout << "(x,y,z): " << track->human[i].p.x <<"," <<track->human[i].p.y<<", "<<track->human[i].p.z <<endl;
	  geometry_msgs::Point pt;

	  pt.x = path_buf[track->human[i].body.tracking_id].x;
	  pt.y = path_buf[track->human[i].body.tracking_id].y;
	  pt.z = path_buf[track->human[i].body.tracking_id].z;
	  m.points.push_back(pt);

	  pt.x = track->human[i].p.x;
	  pt.y = track->human[i].p.y;
	  pt.z = 0;
	  m.points.push_back(pt);
	  
	  path_buf[track->human[i].body.tracking_id].x = pt.x;
	  path_buf[track->human[i].body.tracking_id].y = pt.y;
	  path_buf[track->human[i].body.tracking_id].z = pt.z;

	  m.scale.x = 2;
	  m.color.a = 1;
	  //m.color.r = 1;
	  m.color.g = 1;
	  //m.lifetime = ros::Duration(1);
	  ma.markers.push_back(m);
	}
      ma_pub_.publish(ma);
      //path_pub_.publish(tp);
    }

  return 0;
}

