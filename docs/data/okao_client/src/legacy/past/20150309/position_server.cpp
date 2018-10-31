/*
2014.11.26------------------

 
*/



#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <map>

#include <humans_msgs/Humans.h>
#include "MsgToMsg.hpp"

using namespace std;

#define OKAO_MAX 30
#define OKAO 3
#define BODY_MAX 6
#define HEAD 3

int d_id = 0;
long long now_tracking_id[BODY_MAX] = {0};

class RecogInfo
{
private:
  ros::NodeHandle nh;
  ros::Publisher recog_pub_;
  ros::Subscriber bind_sub_;
  map<long, int> tracking_id_buf;
  map<int, map<int, int> > hist;

public:
  RecogInfo()
  {
    recog_pub_ = nh.advertise<humans_msgs::Humans>("/humans/RecogInfo", 1);
    bind_sub_ = nh.subscribe("/humans/RecogInfo", 1, &RecogInfo::callback, this);
  }

  void callback(const humans_msgs::HumansConstPtr& bind)
  {

    int p_num = bind->num;
    int maxOkaoId = 0, maxHist = 0;
 
    humans_msgs::Humans recog;
    recog.human.resize(p_num);
    recog.num = p_num;

    for( int p_i = 0; p_i < p_num; ++p_i )
      {
	long tracking_id = bind->human[ p_i ].body.tracking_id;
	if( tracking_id )
	  {
	    map< long, int >::iterator tracking_id_find = tracking_id_buf.find( tracking_id );
	    if( tracking_id_find !=  tracking_id_buf.end())
	      {
		//キーの取得(d_id)
		d_id = tracking_id_find->second;
	      }
	    else 
	      {
		++d_id;
		tracking_id_buf[tracking_id] = d_id; 
	      }	 
	    //今回見ているtracking_idの保持
	    now_tracking_id[p_i] = tracking_id;
 
	    int o_id[OKAO] = {0}, o_conf[OKAO] = {0};
	    for(int i = 0; i < OKAO; ++i)
	      {
		o_id[i] = bind->human[p_i].face.persons[i].okao_id;
		o_conf[i] = bind->human[p_i].face.persons[i].conf;
	      }
	    histogram( (d_id) , o_id, o_conf, &maxOkaoId, &maxHist );
	    cout << "d_id: "<<d_id << ", tracking_id: "<< tracking_id << " ---> max id: "<< maxOkaoId << ", max hist: " << maxHist << endl;

	    MsgToMsg::bodyAndFaceToMsg(bind->human[p_i].body,bind->human[p_i].face, &recog.human[p_i]);

	    geometry_msgs::Point h_point;
	    h_point.x = bind->human[p_i].body.joints[3].position.x;
	    h_point.y = bind->human[p_i].body.joints[3].position.y;
	    h_point.z = bind->human[p_i].body.joints[3].position.z;

	    geometry_msgs::PointStamped pst;
	    pst.header.stamp = ros::Time::now();
	    pst.header.frame_id = "/map";

	    MsgToMsg::transformHead( h_point, &pst );

	    recog.human[p_i].d_id = d_id;
	    recog.human[p_i].max_okao_id = maxOkaoId;
	    recog.human[p_i].max_hist = maxHist;
	    recog.human[p_i].p.x = pst.point.x;
	    recog.human[p_i].p.y = pst.point.y;
	    recog.human[p_i].p.z = pst.point.z;
	    //name? conf? lab? grade?
	  }
      }

    recog_pub_.publish(recog);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_recog_info");
  RecogInfo RIObject;
  ros::spin();
  return 0;
}  




/*
class point3D{
public: 
  int id;
  float x;
  float y;
  float z;
  ros::Time t;
  point3D(){
  }
};

map<int, point3D> dicts;

bool sendData(okao_client::OkaoPosSrv::Request &req,
	      okao_client::OkaoPosSrv::Response &res)
{
  point3D position;
  if(req.rule == "update")
    {
      if(req.req_x && req.req_y && req.req_z)
	{
	  std::cout<< "update! id: " << req.u_id << ", pos(" << req.req_x << "," << req.req_y<<","<<req.req_z<<")" <<std::endl;
	  position.id = req.okao_id;
	  position.x = req.req_x;
	  position.y = req.req_y;      
	  position.z = req.req_z;
	  position.t = ros::Time::now();
	  dicts[ req.u_id ] = position;
	}
      return true;
    }
  else if(req.rule == "request")
    {
      std::cout<< "requast!"<<std::endl;
      res.n = dicts.size();
      res.okao_id = dicts[ req.u_id ].id;
      res.res_x = dicts[ req.u_id ].x;
      res.res_y = dicts[ req.u_id ].y;
      res.res_z = dicts[ req.u_id ].z;
      res.stamp =  dicts[ req.u_id ].t;
      return true;
    }
  else if(req.rule == "allnumreq")
    {
      std::cout<< "allnumreq!"<<std::endl;
      res.n = dicts.size();
      return true;
    }
  else if(req.rule == "allidreq")
    {
      std::cout<< "allidreq!"<<std::endl;
      res.n = dicts.size();
      map<int,point3D>::iterator it = dicts.begin();
      int allid = 0;
      res.allid.resize(dicts.size());
      while(it != dicts.end())
	{
	  res.allid[allid] =  it->first;
	  ++it;
	  ++allid;
	}
      return true;
    }
  else
    {
      std::cout<< "false"<<std::endl;
      return false;
    }
}

int main(int argc, char** argv){

  ros::init(argc, argv, "people_position_server");
  ros::NodeHandle n;

  ros::ServiceServer okaoService = n.advertiseService("OkaoClient_srv",sendData);
  
  ros::spin();
  return 0;
}
*/
