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

class TrackMapping
{
private:
  ros::NodeHandle n;
  ros::Publisher ma_pub_;
  ros::Publisher path_pub_;
  ros::Subscriber tracking_sub_;
  map<long, nav_msgs::Path> path_buf;

public:
  TrackMapping()
  {
    //図形メッセージをパブリッシュ
    ma_pub_ = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    path_pub_ = n.advertise<humans_msgs::Humans>("/humans/HumansPath", 1);
    tracking_sub_ = n.subscribe("/humans/OkaoServerNot", 1, &TrackMapping::callback, this);
  }

  void callback(const humans_msgs::HumansConstPtr& track)
  {

    humans_msgs::Humans hs;
    visualization_msgs::Marker m;
    visualization_msgs::MarkerArray ma;

    //認識人数の取得
    int p_num = track->num;
    ros::Time map_time = ros::Time::now();      
    
    humans_msgs::Humans tp;
    for(int i = 0; i < p_num ; ++i)
      {
	visualization_msgs::Marker m;
	m.header.frame_id =  "/map";
	m.header.stamp = map_time;
	m.id = track->human[i].body.tracking_id;
	
	m.ns = "tracking";
	m.type = m.SPHERE;
	m.action = 0;
	geometry_msgs::PointStamped dst;
	dst.header.frame_id = "map";
	dst.header.stamp = map_time;
	geometry_msgs::PointStamped src;
	src.point =  track->human[i].body.joints[3].position;
	src.header.frame_id = track->header.frame_id;
	src.header.stamp = map_time;
	MsgToMsg::transformHead(src, &dst); 
	cout<< "o[ "<< i <<" ](x, y) = ("<< 
	  track->human[i].body.joints[3].position.x << ", " << 
	  track->human[i].body.joints[3].position.y << 
	  ") ---> t(x, y) = (" <<
	  dst.point.x <<", "<<dst.point.y << ")"<<endl;
	m.pose.position.x = dst.point.x;
	m.pose.position.y = dst.point.y;
	m.pose.position.z = 0;
	m.scale.x = .2;
	m.scale.y = .2;
	m.scale.z = .2;
	m.color.a = 1;
	m.color.r = 1;
	m.color.g = 1;
	m.lifetime = ros::Duration(5.0);

	ma.markers.push_back(m);

	//MsgToMsg::bodyAndFaceToMsg(track->human[i].body,track->human[i].face, &tp.human[i]);
      }
    /*
    if( ros::Duration(5) > map_time - track->header.stamp ) 
      {
    */
	ma_pub_.publish(ma);
	path_pub_.publish(tp);
	/*
      }
    */
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_mapping");
  TrackMapping FMObject;
  ros::spin();
  return 0;
}

