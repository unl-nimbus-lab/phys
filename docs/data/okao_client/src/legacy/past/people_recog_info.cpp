/*
2014.11.1--------------------------------


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
  ros::Publisher path_pub_;
  ros::Subscriber bind_sub_;
  map<long, int> tracking_id_buf;
  map<int, map<int, int> > hist;

public:
  RecogInfo()
  {
    recog_pub_ = 
      nh.advertise<humans_msgs::Humans>("/humans/RecogInfo", 1);
    path_pub_ = 
      nh.advertise<humans_msgs::Humans>("/humans/HumansPath", 1);
    bind_sub_ = 
      nh.subscribe("/humans/OkaoServer", 1, &RecogInfo::callback, this);
  }

  void histogram(int d_id, int *o_id, int *o_conf, int *maxOkaoId, int *maxHist)
  {
    for(int i = 0; i < OKAO; ++i)
      {
	hist[d_id][o_id[i]] = hist[d_id][o_id[i]] + o_conf[i]/100;
      }
 
    //最大値のヒストグラムとそのOKAO_IDを出力
    for(int i = 0; i < OKAO_MAX; ++i)
      {
	if(hist[d_id][i] > hist[d_id][*maxOkaoId])
	  {
	    *maxOkaoId = i;
	    *maxHist = hist[ d_id ][ i ];
	  }
      }
  }

  //ヒストグラムの初期化
  void initHistogram(int d_id)
  {
    //初期化
    for( int j = 0; j < OKAO_MAX; j++ )
      {
	hist[ d_id ][ j ] = 0;    
      }
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
	    cout << "d_id: "<<d_id << ", tracking_id: "
		 << tracking_id << " ---> max id: "
		 << maxOkaoId <<", max hist: " << maxHist << endl;

	    MsgToMsg::bodyAndFaceToMsg(bind->human[p_i].body,
				       bind->human[p_i].face, 
				       &recog.human[p_i]);

	    ros::Time t = bind->header.stamp;
	    geometry_msgs::PointStamped h_point;
	    h_point.point.x = bind->human[p_i].body.joints[3].position.x;
	    h_point.point.y = bind->human[p_i].body.joints[3].position.y;
	    h_point.point.z = bind->human[p_i].body.joints[3].position.z;
	    h_point.header.stamp = t;
	    h_point.header.frame_id = bind->header.frame_id;

	    geometry_msgs::PointStamped pst;
	    pst.header.stamp = t;
	    pst.header.frame_id = "/map";

	    std::string camera_frame = bind->header.frame_id;
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
    /*
    //初期化処理
    for( int p_i = 0; p_i < p_num; ++p_i )
      {
	map< long, int >::iterator tracking_id_find = tracking_id_buf.find( now_tracking_id[p_i] );
	if( tracking_id_find == tracking_id_buf.end() )
	  {
	    //このときのmapを削除
	    //ヒストグラムの初期化処理を行う
	    map< int, map< int, int > >::iterator lost_hist_find = hist.find( tracking_id_find->second );
	    //hist.clear( lost_hist_find->second );
	    //initHistogram( tracking_id_find->second );
	    //tracking_id_buf.clear( tracking_id_find->second );
	  }
      }
    */    
    recog.header.stamp = ros::Time::now();
    recog.header.frame_id = "recog";
    recog_pub_.publish(recog);
    path_pub_.publish(recog);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_recog_info");
  RecogInfo RIObject;
  ros::spin();
  return 0;
}  

