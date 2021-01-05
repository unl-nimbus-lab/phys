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

class HPath
{
public:
  HPath()
    : rein_sub_( nh_, "/humans/RecogInfo", 100 ),
      track_sub_( nh_, "/humans/OkaoServerNot", 100 ),
      sync( MySyncPolicy( 10 ), rein_sub_, track_sub_ )
  { 
    sync.registerCallback( boost::bind( &HPath::callback, this, _1, _2 ) );
    path_pub_ = nh_.advertise<nav_msgs::Path>("/humans/path", 10);
  }
  ~HPath()
  {
  }

  void callback(const humans_msgs::HumansConstPtr& rein, 
		const humans_msgs::HumansConstPtr& track)
  {
//認識人数の取得
      int p_num = rein->num;
      ros::Time time = ros::Time::now();      
      
      //visualization_msgs::MarkerArray ma;
      nav_msgs::Path hpt;

      humans_msgs::Humans rt;
      rt.num = p_num;
      //tp.header.push_back(rein->header);
      rt.human = rein->human;
      //rt.human.push_back( track->human );

      for(int i = 0; i < p_num ; ++i)
	{
	  /*
	  //visualization_msgs::Marker m;
	  m.header.frame_id =  "/map";
	  m.header.stamp = map_time;
	  m.id = track->human[i].body.tracking_id;
	  
	  m.ns = "path";
	  m.type = m.LINE_STRIP;
	  m.action = 0;
	  */
	  //cout << "(x,y,z): " << track->human[i].p.x <<"," <<track->human[i].p.y<<", "<<track->human[i].p.z <<endl;
	  //geometry_msgs::Point pt;

	  geometry_msgs::PoseStamped ps;
	  ps.pose.position.x = rt.human[i].p.x;
	  ps.pose.position.y = rt.human[i].p.y;
	  ps.pose.position.z = 0;

	  hpt.poses.push_back(ps);

	  path_buf[rt.human[i].body.tracking_id] = hpt;
	  //path_buf[rt->human[i].body.tracking_id].pose.position.y;
	  //path_buf[track->human[i].body.tracking_id].pose.position.z;

	  /*
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
	  //ma.markers.push_back(m);
	  */
	  //path_buf[].pt.poses.push_back();
	  cout<<path_buf[rt.human[i].body.tracking_id]<<endl;
	}

      //ma_pub_.publish(ma);
      //path_pub_.publish(tp);
    }


  


private:
  ros::NodeHandle nh_;
  ros::Publisher path_pub_;
  map<long, nav_msgs::Path> path_buf;
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
  HPath hp;
  ros::spin();
  
  return 0;
}

