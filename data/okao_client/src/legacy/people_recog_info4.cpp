/*
2015.4.23------------------------------
つーか倍率magniはいらない
いるのは、人物IDの一位と二位の関係

リクエストがあったら、その人物の順位を記録する。
そして、ファイル出力する。

もしその順位記録ファイルがあったら、それを参考にして人物認識をする
もしなければ、別にそれは使わないでいいと思う

でもいまは、そういう学習機能はいらないかな。
ファイルに順位を書き出しておくぐらいで。
そのファイルを自動作成できるようにするのは、また後でやる。


あと、このモジュールも複数起動させられるようにしておくべきか？
だとしたらd_idを文字列にするか、あるいはそもそも廃止するか
今後の展開によって考える


2015.4.18-------------------------------
とりあえずUnknown処理
どういうアルゴリズムで？

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
#include <fstream>

#include <humans_msgs/Humans.h>
#include "MsgToMsg.hpp"

using namespace std;

#define OKAO_MAX 20
#define OKAO 3
#define BODY_MAX 6
#define HEAD 3
#define THRESHOLD 1.8
#define HIST_THRESHOLD 10
#define NUM_THRESHOLD 5

int d_id = 0;


class RecogInfo
{
private:
  ros::NodeHandle nh;
  ros::Publisher recog_pub_;
  ros::Publisher path_pub_;

  map<long, int> tracking_id_buf;
  //map<int, map<int, int> > hist;
  map<int, map<int, double> > hist;
  map<int, humans_msgs::Person> prop_buf;
  map<long long, map<int, double> > id_bind_magni;
  map<long long, int> id_num;

  stringstream file_name;

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

    //unknownの場合
    humans_msgs::Person unknown;
    unknown.okao_id = 0;
    unknown.name = "Unknown";
    unknown.laboratory = "Unknown";
    unknown.grade = "Unknown";
    prop_buf[ 0 ] = unknown;

    //決定できない場合
    humans_msgs::Person undetermined;
    unknown.name = "Undetermined";
    unknown.laboratory = "Undetermined";
    unknown.grade = "Undetermined";

    //ファイル名
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    file_name <<"/home/uema/histdata/" 
	      << pnow->tm_year+1900<< "-"<< pnow->tm_mon<< "-" 
	      << pnow->tm_mday<< "_" <<pnow->tm_hour<<"-"
	      << pnow->tm_min<< "-" << pnow->tm_sec<<".txt"; 

  }
  ~RecogInfo()
  {
    tracking_id_buf.clear();
    hist.clear();
    id_bind_magni.clear();
    id_num.clear();
  }

  void histogram(int d_id, int *o_id, int *o_conf, int *maxOkaoId, 
		 double *maxHist, double *magni, long long tracking_id)
  {
    id_num[tracking_id] = id_num[tracking_id] + 1;

    for(int i = 0; i < OKAO; ++i)
      {
	hist[d_id][o_id[i]] 
	  = hist[d_id][o_id[i]] + (double)o_conf[i]/100.;
      }
 
    //最大値のヒストグラムとそのOKAO_IDを出力
    vector<double> hist_pool, hist_file;
    map<int,int> hist_to_okao;

    for(int i = 0; i < OKAO_MAX; ++i)
      {
	hist_pool.push_back( hist[d_id][i] );
	hist_file.push_back( hist[d_id][i] );
	hist_to_okao[hist[d_id][i]] = i;
      }

    //投票結果のソート
    sort( hist_pool.begin(), hist_pool.end(), 
	  greater<int>() );

    //一位と二位の倍率を出力
    *maxOkaoId = hist_to_okao[hist_pool[0]];
    *maxHist = hist_pool[0];
    *magni = (double)hist_pool[0]/(double)hist_pool[1];

    //アウトプット用関数
    //histOutputFile(hist_file, *maxOkaoId);

  }

  //ファイル出力
  void histOutputFile(vector<double> hist_file, int okao_id)
  {
    //ファイル名
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);

    stringstream tag, hist_line;

    tag <<"[okao_id:" << okao_id << " time:"  <<pnow->tm_hour<<"-"
	<< pnow->tm_min<< "-" << pnow->tm_sec << "], "; 

    hist_line << tag.str();

    ofstream ofs( file_name.str().c_str(), ios::out | ios::app );

    for(int i = 0; i<hist_file.size(); ++i)
      {
	//cout <<"hist_pool[" << i << "] = " << hist_pool[ i ] <<endl;
	if(hist_file[i] >= 0 && hist_file[i] < 10 )
	  hist_line << "    " << fixed << setprecision(2) << hist_file[ i ] << ",";
	else if(hist_file[i] >= 10 && hist_file[i] < 100 )
	  hist_line << "   " << fixed << setprecision(2) << hist_file[ i ] << ",";
	else if(hist_file[i] >= 100 && hist_file[i] < 1000 )
	  hist_line << " " << fixed << setprecision(2) << hist_file[ i ] << ",";
	else 
	  hist_line << "" << fixed << setprecision(2) << hist_file[ i ] << ",";
      }
    //hist_line << endl ;
    ofs << hist_line.str() << endl;
  }

  //人物の認識についての処理
  void personRecogProcess(long long tracking_id, int *okao_id, 
			  double hist, map<long long, int> id_num)
  {
    if( id_bind_magni[ tracking_id ][ *okao_id ] < magni )
      id_bind_magni[ tracking_id ][ *okao_id ] = magni;
    
    //もし閾値より小さいなら,unknown処理
    if( (id_bind_magni[ tracking_id ][ *okao_id ] < THRESHOLD) 
	|| (id_num[ tracking_id ] < NUM_THRESHOLD) )
      {
	*okao_id = 0;
      }  
    
    /*
    if( (id_bind_magni[ tracking_id ][ *okao_id ] < THRESHOLD) 
	|| (hist < HIST_THRESHOLD) )
      {
	*okao_id = 0;
      }
    */  
  }


  void okaoProcess(humans_msgs::Human src, humans_msgs::Humans *dst, 
		   std_msgs::Header src_header)
  {

    long long tracking_id = src.body.tracking_id;
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
	
	int o_id[OKAO] = {0}, o_conf[OKAO] = {0};
	
	//メッセージ内にある一人の人物につき、一位から三位までの個人情報を取得する。
	for(int i = 0; i < OKAO; ++i)
	  {
	    o_id[i] 
	      = src.face.persons[i].okao_id;
	    o_conf[i] 
	      = src.face.persons[i].conf;
	  }
	
	//personの保持
	humans_msgs::Person ps;
	ps = src.face.persons[0];

	prop_buf[ ps.okao_id ] = ps;
	
	int maxOkaoId = 0;
	double maxHist = 0.0, magni = 0.0;
	histogram( (d_id) , o_id, o_conf, &maxOkaoId, 
		   &maxHist, &magni, tracking_id );
	personRecogProcess( tracking_id, &maxOkaoId, maxHist,  magni );

	cout <<"face found[ " << ros::Time::now() 
	     << " ], d_id: "<<d_id << ", tracking_id: "
	     << tracking_id << " ---> max id: "
	     << maxOkaoId <<", max hist: " << maxHist << ", magni: " << magni << endl;
	
	humans_msgs::Human h;
	//h = src;
	
	ros::Time t = src_header.stamp;
	
	h.body.joints = src.body.joints;
	h.body.tracking_id = src.body.tracking_id;	
	h.body.is_tracked = src.body.is_tracked;
	h.body.left_hand_state = src.body.left_hand_state;
	h.body.right_hand_state = src.body.right_hand_state;
	h.d_id = d_id;
	h.max_okao_id = maxOkaoId;
	h.max_hist = maxHist;
	h.magni = magni;
	h.header.stamp = t;
	h.header.frame_id = src_header.frame_id;
	h.p = src.body.joints[ HEAD ].position;
	h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	dst->human.push_back( h );

      } 
  }

  void okaoNotProcess(humans_msgs::Human src, humans_msgs::Humans *dst, 
		      std_msgs::Header src_header)
  {
    long long tracking_id = src.body.tracking_id;
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
	    int maxOkaoId = 0;
	    double maxHist = 0, magni = 0.0;
	    histogram( (d_id) , o_id, o_conf, &maxOkaoId, &maxHist, &magni, tracking_id );
	    personRecogProcess( tracking_id, &maxOkaoId, maxHist, magni );
	    cout <<"face not found[ " << ros::Time::now() 
		 << " ], d_id: "<<d_id << ", tracking_id: "
		 << tracking_id << " ---> max id: "
		 << maxOkaoId <<", max hist: " << maxHist << ", magni: " << magni << endl;
	    
	    ros::Time t = src_header.stamp;

	    humans_msgs::Human h;
	    
	    h.body.joints = src.body.joints;
	    h.body.tracking_id = src.body.tracking_id;
	    h.body.is_tracked = src.body.is_tracked;
	    h.body.left_hand_state = src.body.left_hand_state;
	    h.body.right_hand_state = src.body.right_hand_state;
	    
	    h.d_id = d_id;
	    h.max_okao_id =  maxOkaoId;
	    h.max_hist = maxHist;
	    h.magni = magni;

	    h.header.stamp = t;
	    h.header.frame_id = src_header.frame_id;
	    h.p = src.body.joints[ HEAD ].position;
	    h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	    h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	    h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	    dst->human.push_back( h );
	  }	    
      }

  }

  void callback(
		const humans_msgs::HumansConstPtr& okao,
		const humans_msgs::HumansConstPtr& okaoNot
		)
  {
    humans_msgs::Humans recog;

    //okaoについての処理
    for( int p_i = 0; p_i < okao->human.size(); ++p_i )
      {
	okaoProcess(okao->human[p_i], &recog, okao->header);
      }

    //okaoNotについての処理
    for( int p_i = 0; p_i < okaoNot->human.size(); ++p_i )
      {
	okaoNotProcess(okaoNot->human[p_i], &recog, okaoNot->header);
      }
    
    //magniについての処理

    recog.num = recog.human.size();
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
