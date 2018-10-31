/*
ヒストグラムの出力



*/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <fstream>

#include <humans_msgs/Humans.h>
#include "MsgToMsg.hpp"

using namespace std;

#define OKAO_MAX 15
#define OKAO 3
#define BODY_MAX 6
#define HEAD 3

int d_id = 0;
//long long now_tracking_id[BODY_MAX] = {0};
template<class T> inline std::string toString(T x) {std::ostringstream sout;sout<<x;return sout.str();}




class RecogInfo
{
private:
  ros::NodeHandle nh;
  ros::Publisher recog_pub_;
  ros::Publisher path_pub_;
  //  ros::Subscriber okao_sub_;

  map<long, int> tracking_id_buf;
  map<int, map<int, int> > hist;
  //vector<long> now_tracking_id;
  int frame;
  //ファイル出力
  stringstream name;
  
  ros::Time start;

  typedef message_filters::Subscriber< 
    humans_msgs::Humans > HumansSubscriber; 

  HumansSubscriber okao_sub, okaoNot_sub;

  typedef message_filters::sync_policies::ApproximateTime<
    humans_msgs::Humans, humans_msgs::Humans
    > MySyncPolicy;

  message_filters::Synchronizer< MySyncPolicy > sync;

public:
  RecogInfo() :
    okao_sub( nh, "/humans/okao_server", 100 ),
    okaoNot_sub( nh, "/humans/okao_server_not", 100 ), 
    sync( MySyncPolicy( 100 ), okao_sub, okaoNot_sub )
  {
    //ファイル名
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    name <<"/home/uema/histdata/" 
	 << pnow->tm_year+1900  << pnow->tm_mon 
	 << pnow->tm_mday<< pnow->tm_hour
	 << pnow->tm_min << pnow->tm_sec; 
    sync.registerCallback( boost::bind( &RecogInfo::callback, this, _1, _2 ) );
    frame = 0;    
    start = ros::Time::now();
    recog_pub_ = 
      nh.advertise<humans_msgs::Humans>("/humans/recog_info", 1);
  }
  ~RecogInfo()
  {

    tracking_id_buf.clear();
    hist.clear();
  }

  void histogram(int d_id, int *o_id, int *o_conf, int *maxOkaoId, int *maxHist)
  {

    for(int i = 0; i < OKAO; ++i)
      {
	hist[d_id][o_id[i]] 
	  = hist[d_id][o_id[i]] + o_conf[i]/100;
      }

    stringstream ss, framess, didss;

    //最大値のヒストグラムとそのOKAO_IDを出力
    for(int i = 1; i <= OKAO_MAX; ++i)
      {
	if(hist[d_id][i] > hist[d_id][*maxOkaoId])
	  {
	    *maxOkaoId = i;
	    *maxHist = hist[ d_id ][ i ];
	  }
	
	ss << toString(  i  ) 
	   <<", " << toString( hist[d_id][i] )<<endl;; 
	  //<<", per: "<< toString( (double)member[ i ]/ALLCOUNT ) << endl;
	//name << "okao_cost_" << toString( ros::Time::now() ) <<".txt" ;
	//string namestr = name.str();

      }
    time_t now_f = time(NULL);
    struct tm *pnow_f = localtime(&now_f);

    framess << toString( frame );
    didss << toString( d_id ); 
    std::ofstream ofs( name.str().c_str(), std::ios::out | std::ios::app );
    ofs <<"d_id: " << didss.str() << ", frame: "  << framess.str() 
	<< ", time: " //<< toString( ros::Time::now() - start )
	<< pnow_f->tm_hour << ":"<< pnow_f->tm_min << ":" << pnow_f->tm_sec
	<< endl << ss.str() << endl;
    ++frame;
  }

  void callback(
		const humans_msgs::HumansConstPtr& okao,
		const humans_msgs::HumansConstPtr& okaoNot
		)
  {
    humans_msgs::Humans recog;
    vector<long> now_tracking_id;

    int okao_num = okao->num;
    int okao_recog_num = 0;

    //okaoについての処理
    for( int p_i = 0; p_i < okao_num; ++p_i )
      {

	long tracking_id = okao->human[ p_i ].body.tracking_id;
	if( tracking_id )
	  {
	    map< long, int >::iterator tracking_id_find 
	      = tracking_id_buf.find( tracking_id );
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

	    now_tracking_id.push_back( tracking_id );

	    int o_id[OKAO] = {0}, o_conf[OKAO] = {0};
	    for(int i = 0; i < OKAO; ++i)
	      {
		o_id[i] 
		  = okao->human[p_i].face.persons[i].okao_id;
		o_conf[i] 
		  = okao->human[p_i].face.persons[i].conf;
	      }
	    int maxOkaoId = 0, maxHist = 0;
	    histogram( (d_id) , o_id, o_conf, &maxOkaoId, &maxHist );

	    cout <<"face found[ " << ros::Time::now() 
		 << " ], d_id: "<<d_id << ", tracking_id: "
		 << tracking_id << " ---> max id: "
		 << maxOkaoId <<", max hist: " << maxHist << endl;

	    humans_msgs::Human h;
	    MsgToMsg::bodyAndFaceToMsg( 
				       okao->human[p_i].body,
				       okao->human[p_i].face, 
				       &h
					);
	    
	    ros::Time t = okao->header.stamp;
	    geometry_msgs::PointStamped h_point;
	    h_point.point.x 
	      = okao->human[ p_i ].body.joints[ HEAD ].position.x;
	    h_point.point.y 
	      = okao->human[ p_i ].body.joints[ HEAD ].position.y;
	    h_point.point.z 
	      = okao->human[ p_i ].body.joints[ HEAD ].position.z;
	    h_point.header.stamp 
	      = t;
	    h_point.header.frame_id 
	      = okao->header.frame_id;

	    //geometry_msgs::PointStamped pst;
	    //pst.header.stamp = t;
	    //pst.header.frame_id = "map";
	    //std::string camera_frame = okao->header.frame_id;
	    //MsgToMsg::transformHead( h_point, &pst );

	    h.d_id = d_id;
	    h.max_okao_id = maxOkaoId;
	    h.max_hist = maxHist;
	    h.header.stamp = t;
	    h.header.frame_id = okao->header.frame_id;
	    h.p = h_point.point;
	    recog.human.push_back( h );
	    
	    //name? conf? lab? grade?
	    ++okao_recog_num;
	  }
      }

    //okaoNotについての処理
    int okaoNot_num = okaoNot->num;
    int okaoNot_recog_num = 0;
    for( int p_i = 0; p_i < okaoNot_num; ++p_i)
      {
	long tracking_id = okaoNot->human[ p_i ].body.tracking_id;
	if( tracking_id )
	  {
	    map< long, int >::iterator tracking_id_find 
	      = tracking_id_buf.find( tracking_id );
	    if( tracking_id_find !=  tracking_id_buf.end())
	      {
		//キーの取得(d_id)
		d_id = tracking_id_find->second;

		//人物情報の取得
		int o_id[OKAO] = {0}, o_conf[OKAO] = {0};
		int maxOkaoId = 0, maxHist = 0;
		histogram( (d_id) , o_id, o_conf, &maxOkaoId, &maxHist );

		cout <<"face not found[ " << ros::Time::now() 
		     << " ], d_id: "<<d_id << ", tracking_id: "
		     << tracking_id << " ---> max id: "
		     << maxOkaoId <<", max hist: " << maxHist << endl;

		//人物位置の更新
		ros::Time t = okaoNot->header.stamp;
		geometry_msgs::PointStamped h_point;
		h_point.point.x 
		  = okaoNot->human[ p_i ].body.joints[ HEAD ].position.x;
		h_point.point.y 
		  = okaoNot->human[ p_i ].body.joints[ HEAD ].position.y;
		h_point.point.z 
		  = okaoNot->human[ p_i ].body.joints[ HEAD ].position.z;
		h_point.header.stamp 
		  = t;
		h_point.header.frame_id 
		  = okaoNot->header.frame_id;
		
		//geometry_msgs::PointStamped pst;
		//pst.header.stamp = t;
		//pst.header.frame_id = "map";
		//pst.header.frame_id = okaoNot->header.frame_id;
		//std::string camera_frame = okao->header.frame_id;
		//MsgToMsg::transformHead( h_point, &pst );

		humans_msgs::Human h;

		humans_msgs::Body b;
		MsgToMsg::bodyToBody(
				     okaoNot->human[p_i].body,
				     &b
				     );
		h.body = b;
		h.d_id = d_id;
		h.max_okao_id =  maxOkaoId;
		h.max_hist = maxHist;
		//h.p = pst.point;
		h.header.stamp = t;
		h.header.frame_id = okaoNot->header.frame_id;
		h.p = h_point.point;
		recog.human.push_back( h );
		++okaoNot_recog_num;
	      }	    
	  }
	//recog.header.frame_id = okao->header.frame_id;
      }
    
    /*
    //初期化処理
    map< long, int >::iterator past_tracking_id;
    for(past_tracking_id = tracking_id_buf.begin(); 
	past_tracking_id != tracking_id_buf.end() ; past_tracking_id++)
      {
    	cout << "p_i: "<< past_tracking_id->first << " , ";
	vector<long>::iterator eq 
	  = find( now_tracking_id.begin(), now_tracking_id.end(), past_tracking_id->first);
	if( eq == now_tracking_id.end() ) 
	  {
	    //ここでヒストグラムなどの初期化を行う
	    cout<<"lost_d_id[ "<< past_tracking_id->second <<" ]" <<endl; 
	    cout<<"lost_tracking_id[ "<< past_tracking_id->first <<" ]" << endl; 
	  }
      }    
    */

    //人物を見つけなかった場合,初期化処理をする
    if( okao_recog_num + okaoNot_recog_num == 0 )
      {
	//見ていないtracking_idのリクエスト
	/*
tracking_id_buf.erase(lost_tracking_id);
hist.erase(lost_d_id);
	 */
      }

    recog.num = okao_recog_num + okaoNot_recog_num;
    recog.header.stamp = ros::Time::now();
    recog.header.frame_id = "recog";
    recog_pub_.publish( recog );
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_recog_info");
  RecogInfo RIObject;
  ros::spin();
  return 0;
}
