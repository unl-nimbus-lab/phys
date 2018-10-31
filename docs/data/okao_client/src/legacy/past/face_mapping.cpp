/*
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

#include <turtlesim/Spawn.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
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
template<class T> inline std::string toString(T x) {std::ostringstream sout;sout<<x;return sout.str();}

class FaceMapping
{
private:
  ros::NodeHandle n;
  ros::Publisher markerArray_pub_;
  ros::Subscriber recogInfo_sub_;

public:
  FaceMapping()
  {
    //図形メッセージをパブリッシュ
    markerArray_pub_ = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    recogInfo_sub_ = n.subscribe("/humans/RecogInfo", 1, &FaceMapping::callback, this);
  }

  void callback(const humans_msgs::HumansConstPtr& rein)
  {

    visualization_msgs::MarkerArray points, lines, names;
    //認識人数の取得
    int p_num = rein->num;
    
    ros::Time map_time = ros::Time::now();      
    float pr,pg,pb,pa,br,bg,bb,ba; 
    float ts,ps,ls,bs;
    
    for(int i = 0; i < p_num ; ++i)
      {
	if( rein->human[i].d_id )
	  { 
	    ros::ServiceClient okaoStack = n.serviceClient<okao_client::OkaoStack>("okao_stack");
	    okao_client::OkaoStack stack;
	    visualization_msgs::Marker point, line, name;

	    stack.request.rule = "req";
	    stack.request.okao_id = rein->human[i].max_okao_id;
	    okaoStack.call(stack);
	    
	    point.header.frame_id =  line.header.frame_id = name.header.frame_id = "/map";
	    point.header.stamp = line.header.stamp = name.header.stamp = ros::Time::now();
	    point.id = line.id = name.id = rein->human[i].d_id;
	    point.action = line.action = name.action = visualization_msgs::Marker::ADD;
	    point.pose.orientation.w = line.pose.orientation.w = name.pose.orientation.w = 1.0;
	    
	    point.ns = "points";
	    line.ns = "lines";
	    name.ns = "names";
	    
	    point.type = visualization_msgs::Marker::POINTS;
	    line.type = visualization_msgs::Marker::LINE_STRIP;	      
	    name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	    /* 
	    //humans_msgs::Joints joint;
	    geometry_msgs::PointStamped pst;
	    pst.header.stamp = ros::Time::now();
	    pst.header.frame_id = "/map";
	    MsgToMsg::transformHead( rein->human[i].p, &pst );
	    */
	    geometry_msgs::Point pt0, pt1;
	    pt0.x = rein->human[i].p.x;
	    pt0.y = rein->human[i].p.y;
	    pt0.z = 0; 
	    pt1.x = pt0.x + 1.0;
	    pt1.y = pt0.y + 1.0;
	    pt1.z = pt0.z + 1.0;
	    
	    point.points.resize(1);
	    line.points.resize(2);
	    
	    point.points[0].x = pt0.x;
	    point.points[0].y = pt0.y;
	    point.points[0].z = pt0.z;
	    
	    line.points[0].x = pt0.x;
	    line.points[0].y = pt0.y;
	    line.points[0].z = pt0.z;
	    line.points[1].x = pt1.x;
	    line.points[1].y = pt1.y;
	    line.points[1].z = pt1.z;
	    
	    name.pose.position.x = pt1.x;
	    name.pose.position.y = pt1.y;
	    name.pose.position.z = pt1.z; 
	    
	    if(rein->human[i].max_hist > 10)
	      {	      
		pr = 0.8f;
		pg = 0.3f;
		pb = 0.f;
		pa = 1.0;
		
		br = 0.0f;
		bg = 0.0f;
		bb = 1.0f;
		ba = 0.3;
		
		ts = 0.8;
		ps = 0.5;
		ls = 0.1;
	      }
	    else
	      {
		pr = 0.6f;
		pg = 0.0f;
		pb = 0.0f;
		pa = 1.0; 
		
		br = 0.3f;
		bg = 0.0f;
		bb = 0.0f;
		ba = 0.3;
		
		ts = 0.4;
		ps = 0.2;
		ls = 0.05;
	      }
	    
	    point.color.r = line.color.r = name.color.r = pr;
	    point.color.g = line.color.g = name.color.g = pg;
	    point.color.b = line.color.b = name.color.b = pb;
	    point.color.a = line.color.a = name.color.a = pa;
	    
	    name.scale.z = ts;
	    point.scale.x = point.scale.y = ps;    
	    line.scale.x = ls;  
	    
	    string idtext = toString<int>(rein->human[i].d_id);
	    string histtext = toString<int>(rein->human[i].max_hist);
	    name.text = idtext +","+ stack.response.name +","+ histtext;

	    points.markers.push_back( point );
	    lines.markers.push_back( line );
	    names.markers.push_back( name );
	  }
      }
    markerArray_pub_.publish( points );
    markerArray_pub_.publish( lines );
    markerArray_pub_.publish( names );
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_mapping");
  FaceMapping FMObject;
  ros::spin();
  return 0;
}



