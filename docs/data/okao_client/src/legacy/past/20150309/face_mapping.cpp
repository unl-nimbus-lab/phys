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

    visualization_msgs::MarkerArray mapping_points, mapping_lines, mapping_names;
    //認識人数の取得
    int p_num = rein->num;
    
    //メモリの確保
    //mapping_points.markers.resize(p_num);
    //mapping_lines.markers.resize(p_num);
    //mapping_names.markers.resize(p_num);

    ros::Time map_time = ros::Time::now();      
    float pr,pg,pb,pa,br,bg,bb,ba; 
    float ts,ps,ls,bs;
    
    for(int i = 0; i < p_num ; ++i)
      {
	if( rein->d_id )
	  { 
	    ros::ServiceClient okaoStack = n.serviceClient<okao_client::OkaoStack>("okao_stack");
	    okao_client::OkaoStack stack;
	    stack.request.rule = "req";
	    stack.request.okao_id = rein->human[i].max_okao_id;
	    okaoStack.call(stack);
	    
	    mapping_points.markers[i].header.frame_id =  mapping_lines.markers[i].header.frame_id = mapping_names.markers[i].header.frame_id = "/map";
	    mapping_points.markers[i].header.stamp = mapping_lines.markers[i].header.stamp = mapping_names.markers[i].header.stamp = ros::Time::now();
	    mapping_points.markers[i].id = mapping_lines.markers[i].id = mapping_names.markers[i].id = rein->human[i].d_id;
	    mapping_points.markers[i].action = mapping_lines.markers[i].action = mapping_names.markers[i].action = visualization_msgs::Marker::ADD;
	    mapping_points.markers[i].pose.orientation.w = mapping_lines.markers[i].pose.orientation.w = mapping_names.markers[i].pose.orientation.w = 1.0;
	    
	    mapping_points.markers[i].ns =   "points";
	    mapping_lines.markers[i].ns =    "lines";
	    mapping_names.markers[i].ns =    "names";
	    
	    mapping_points.markers[i].type = visualization_msgs::Marker::POINTS;
	    mapping_lines.markers[i].type =  visualization_msgs::Marker::LINE_STRIP;	      
	    mapping_names.markers[i].type =  visualization_msgs::Marker::TEXT_VIEW_FACING;
	    
	    //humans_msgs::Joints joint;
	    geometry_msgs::PointStamped pst;
	    pst.header.stamp = ros::Time::now();
	    pst.header.frame_id = "/map";
	    MsgToMsg::transformHead( rein->human[i].p, &pst );
	    
	    geometry_msgs::Point pt0, pt1;
	    pt0.x = pst.point.x;
	    pt0.y = pst.point.y;
	    pt0.z = 0; 
	    pt1.x = pt0.x + 1.0;
	    pt1.y = pt0.y + 1.0;
	    pt1.z = pt0.z + 1.0;
	    
	    mapping_points.markers[i].points.resize(1);
	    mapping_lines.markers[i].points.resize(2);
	    
	    mapping_points.markers[i].points[0].x = pt0.x;
	    mapping_points.markers[i].points[0].y = pt0.y;
	    mapping_points.markers[i].points[0].z = pt0.z;
	    
	    mapping_lines.markers[i].points[0].x = pt0.x;
	    mapping_lines.markers[i].points[0].y = pt0.y;
	    mapping_lines.markers[i].points[0].z = pt0.z;
	    mapping_lines.markers[i].points[1].x = pt1.x;
	    mapping_lines.markers[i].points[1].y = pt1.y;
	    mapping_lines.markers[i].points[1].z = pt1.z;
	    
	    mapping_names.markers[i].pose.position.x = pt1.x;
	    mapping_names.markers[i].pose.position.y = pt1.y;
	    mapping_names.markers[i].pose.position.z = pt1.z; 
	    
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
	    
	    mapping_points.markers[i].color.r = mapping_lines.markers[i].color.r = mapping_names.markers[i].color.r = pr;
	    mapping_points.markers[i].color.g = mapping_lines.markers[i].color.g = mapping_names.markers[i].color.g = pg;
	    mapping_points.markers[i].color.b = mapping_lines.markers[i].color.b = mapping_names.markers[i].color.b = pb;
	    mapping_points.markers[i].color.a = mapping_lines.markers[i].color.a = mapping_names.markers[i].color.a = pa;
	    
	    mapping_names.markers[i].scale.z = ts;
	    mapping_points.markers[i].scale.x = ps;    
	    mapping_lines.markers[i].scale.x = ls;  
	    
	    string idtext = toString<int>(rein->human[i].d_id);
	    string histtext = toString<int>(rein->human[i].max_hist);
	    mapping_names.markers[i].text = idtext +","+ stack.response.name +","+ histtext;
	  }
      }
    markerArray_pub_.publish(mapping_points);
    markerArray_pub_.publish(mapping_lines);
    markerArray_pub_.publish(mapping_names);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_mapping");
  FaceMapping FMObject;
  ros::spin();
  return 0;
}



