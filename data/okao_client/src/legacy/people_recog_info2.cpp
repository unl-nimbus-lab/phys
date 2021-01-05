/*
2015.4.6--------------------------------
persons[0]の内部を埋めたい
そのためには、okao_idとname, grade, laboratoryを結びつけておく必要がある

2015.3.5--------------------------------
一位と二位の投票数を使って
人物信頼度の比を求める

信頼度投票指標 = 一位の投票数/二位の投票数

信頼度投票指標が2以上なら、確定
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
#include <functional>

#include <humans_msgs/Humans.h>
#include "MsgToMsg.hpp"

using namespace std;

#define OKAO_MAX 30
#define OKAO 3
#define BODY_MAX 6
#define HEAD 3

int d_id = 0;
//long long now_tracking_id[BODY_MAX] = {0};


class RecogInfo
{
private:
  ros::NodeHandle nh;
  ros::Publisher recog_pub_;
  ros::Publisher path_pub_;
  //  ros::Subscriber okao_sub_;

  map<long, int> tracking_id_buf;
  map<int, map<int, int> > hist;
  map<int, humans_msgs::Person> prop_buf;
  //vector<long> now_tracking_id;

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
    sync.registerCallback( boost::bind( &RecogInfo::callback, this, _1, _2 ) );
    
    recog_pub_ = 
      nh.advertise<humans_msgs::Humans>("/humans/recog_info", 1);
  }
  ~RecogInfo()
  {
    tracking_id_buf.clear();
    hist.clear();
  }

  void histogram(int d_id, int *o_id, int *o_conf, int *maxOkaoId, int *maxHist, double *magni)
  {
    for(int i = 0; i < OKAO; ++i)
      {
	hist[d_id][o_id[i]] 
	  = hist[d_id][o_id[i]] + o_conf[i]/100;
      }
 
    //最大値のヒストグラムとそのOKAO_IDを出力
    //二位との比も出力したい
  
    vector<int> hist_pool;
    map<int,int> hist_to_okao;

    for(int i = 0; i < OKAO_MAX; ++i)
      {
	hist_pool.push_back( hist[d_id][i] );
	hist_to_okao[hist[d_id][i]] = i;
      }

    //投票結果のソート
    sort( hist_pool.begin(), hist_pool.end(), 
	  greater<int>() );

    //cout << "secondHist: "<<hist_pool[1]<<endl;
    //一位と二位の倍率を出力
    *maxOkaoId = hist_to_okao[hist_pool[0]];
    *maxHist = hist_pool[0];
    *magni = (double)hist_pool[0]/(double)hist_pool[1];
  }

  void callback(
		const humans_msgs::HumansConstPtr& okao,
		const humans_msgs::HumansConstPtr& okaoNot
		)
  {
    humans_msgs::Humans recog;
    vector<long> now_tracking_id;

    int okao_num = okao->human.size();
    int okao_recog_num = 0;

    //okaoについての処理
    for( int p_i = 0; p_i < okao_num; ++p_i )
      {

	long tracking_id = okao->human[ p_i ].body.tracking_id;
	if( tracking_id )
	  {
	    map< long, int >::iterator tracking_id_find 
	      = tracking_id_buf.find( tracking_id );

	    //tracking_idを過去に見ていたかどうかを調べる(d_id)
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

	    //humans_msgs::Face fc;

	    //メッセージ内にある一つの人物において、一位から三位までの個人情報を取得する。
	    for(int i = 0; i < OKAO; ++i)
	      {

		o_id[i] 
		  = okao->human[p_i].face.persons[i].okao_id;
		o_conf[i] 
		  = okao->human[p_i].face.persons[i].conf;
	      }

	    //personの保持
	    humans_msgs::Person ps;
	    ps.okao_id = okao->human[p_i].face.persons[0].okao_id;
	    ps.hist = okao->human[p_i].face.persons[0].hist;
	    ps.conf = okao->human[p_i].face.persons[0].conf;
	    ps.name = okao->human[p_i].face.persons[0].name;
	    ps.laboratory = okao->human[p_i].face.persons[0].laboratory;
	    ps.grade = okao->human[p_i].face.persons[0].grade;
	    prop_buf[ ps.okao_id ] = ps;


	    //prop_buf[ ]= fc;
	    int maxOkaoId = 0, maxHist = 0;
	    double magni = 0.0;
	    histogram( (d_id) , o_id, o_conf, &maxOkaoId, &maxHist, &magni );

	    cout <<"face found[ " << ros::Time::now() 
		 << " ], d_id: "<<d_id << ", tracking_id: "
		 << tracking_id << " ---> max id: "
		 << maxOkaoId <<", max hist: " << maxHist << ", magni: " << magni << endl;

	    humans_msgs::Human h;
	    h = okao->human[ p_i ];

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

	    h.d_id = d_id;
	    h.max_okao_id = maxOkaoId;
	    h.max_hist = maxHist;
	    h.magni = magni;
	    h.header.stamp = t;
	    h.header.frame_id = okao->header.frame_id;
	    h.p = h_point.point;
	    recog.human.push_back( h );

	    ++okao_recog_num;
	  }
      }

    //okaoNotについての処理
    int okaoNot_num = okaoNot->human.size();
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
		double magni = 0.0;
		histogram( (d_id) , o_id, o_conf, &maxOkaoId, &maxHist, &magni );

		cout <<"face not found[ " << ros::Time::now() 
		     << " ], d_id: "<<d_id << ", tracking_id: "
		     << tracking_id << " ---> max id: "
		     << maxOkaoId <<", max hist: " << maxHist << ", magni: " << magni << endl;

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
	    
		humans_msgs::Human h;

		humans_msgs::Body b;
		h.body = okaoNot->human[p_i].body;
		h.d_id = d_id;
		h.max_okao_id =  maxOkaoId;
		h.max_hist = maxHist;
		h.magni = magni;
		//h.p = pst.point;
		h.header.stamp = t;
		h.header.frame_id = okaoNot->header.frame_id;
		h.p = h_point.point;
		h.face.persons.push_back( prop_buf[ maxOkaoId ] );
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
