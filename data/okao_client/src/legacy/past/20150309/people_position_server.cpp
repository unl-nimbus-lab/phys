/*
2014.12.18----------------
人物の位置を一時的に保存しておくモジュール
OKAO_IDをもらったら最新の人物位置を返す


2014.6.3------------------
認識した人数分の人物位置を一時的に記録しておくモジュール

map<int,point3D>で、okao_idをキーにする

 
*/

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>

//オリジナルメッセージ
#include "humans_msgs/Humans.h"
#include "okao_client/OkaoPosSrv.h"
#include "okao_client/OkaoStack.h"

using namespace std;

class pointStamp{
public:
  ros::Time t; 
  float x;
  float y;
  pointStamp(){
  }
};

map<int, pointStamp> dicts;

class Server
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_;
  ros::ServiceServer srv_;
  time_t timer;
  struct tm *local;

public:
  Server()
  {
    sub_ = n.subscribe("/humans/RecogInfo", 1, &Server::callback, this);
    srv_ = n.advertiseService("okao_pos_srv", &Server::sendData, this);
  }

  string people_name(int okao_id)
  {
    ros::ServiceClient okaoStack = 
      n.serviceClient<okao_client::OkaoStack>("okao_stack");
    okao_client::OkaoStack stack;

    stack.request.rule = "req";
    stack.request.okao_id = okao_id;
    okaoStack.call(stack);

    return stack.response.name;
  }
  
  void callback(const humans_msgs::HumansConstPtr& rein)
  {
    timer = time( NULL );
    local = localtime( &timer );

    int year = local->tm_year + 1900;
    int month = local->tm_mon + 1;
    int day = local->tm_mday;
    int hour = local->tm_hour;
    int min = local->tm_min;
    int sec = local->tm_sec;
    int isdst = local->tm_isdst; 

    for(int i = 0; i < rein->num; ++i)
      {
	pointStamp ps;
	//position.id = rein->human[i].max_okao_id;
	ps.x = rein->human[i].p.x;
	ps.y = rein->human[i].p.y;
	ps.t = rein->header.stamp;
	dicts[ rein->human[i].max_okao_id ] = ps;

	cout <<"update [ "<< year << "/" << month << "/" << day
	     << " " << hour << ":" << min << ":" << sec  
	     << " ]: " << people_name( rein->human[i].max_okao_id )
	     << " , (x , y) = ("<< ps.x 
	     << " , " << ps.y << ")" << endl;
      }  
  }

  bool sendData(okao_client::OkaoPosSrv::Request &req,
		okao_client::OkaoPosSrv::Response &res)
  {
    pointStamp position;
    if(req.rule == "req")
      {
	cout << "requast people: "
	     << people_name( req.okao_id ) << endl;
	res.res_x = dicts[ req.okao_id ].x;
	res.res_y = dicts[ req.okao_id ].y;
	res.res_t = dicts[ req.okao_id ].t;
	return true;
      }
    else
      {
	std::cout<< "false"<<std::endl;
	return false;
      }
  }
};
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_position_server");
  Server SRV;
  ros::spin();
  return 0;
}
