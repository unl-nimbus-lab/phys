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
#define HIST_THRESHOLD 20
#define NUM_THRESHOLD 5
#define MAX_FRAME 10
#define SLOPE 5

//int d_id = 0;


class RecogInfo
{
private:
  ros::NodeHandle nh;
  ros::Publisher recog_pub_;
  ros::Publisher path_pub_;

  vector<long long> tracking_id_buffer;
  map<long long, map<int, double> > hist;
  map<int, humans_msgs::Person> prop_buf;
  map<long long, map<int, double> > id_bind_magni;
  map<long long, int> id_num;
  map<long long, int > tracking_okao;
  map<long long, int > t_o_id;
  map<long long, double > tracking_hist;
  map<long long, bool > t_determined;
  map<long long, bool > t_known;

  //map<long long, vector<int> > tracking_okao;
  //map<long long, vector<double> > tracking_hist;

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
    undetermined.okao_id = -1;
    undetermined.name = "Undetermined";
    undetermined.laboratory = "Undetermined";
    undetermined.grade = "Undetermined";
    prop_buf[ -1 ] = undetermined;

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
    tracking_id_buffer.clear();
    hist.clear();
    tracking_okao.clear();
    tracking_hist.clear();
    id_num.clear();
    prop_buf.clear();
  }

  void histogram(long long tracking_id, int *o_id, int *o_conf, 
		 int *maxOkaoId, double *maxHist)
  {
    //frame数
    id_num[tracking_id] = id_num[tracking_id] + 1;

    for(int i = 0; i < OKAO; ++i)
      {
	hist[tracking_id][o_id[i]] 
	  = hist[tracking_id][o_id[i]] + (double)o_conf[i]/100.;
      }
 
    //最大値のヒストグラムとそのOKAO_IDを出力
    vector<double> hist_pool, hist_file;
    map<int,int> hist_to_okao;

    for(int i = 0; i < OKAO_MAX; ++i)
      {
	hist_pool.push_back( hist[tracking_id][i] );
	//hist_file.push_back( hist[tracking_id][i] );
	hist_to_okao[hist[tracking_id][i]] = i;
      }

    //投票結果のソート
    sort( hist_pool.begin(), hist_pool.end(), 
	  greater<int>() );

    //一位と二位の倍率を出力
    *maxOkaoId = hist_to_okao[hist_pool[0]];
    *maxHist = hist_pool[0];
    tracking_okao[tracking_id] = *maxOkaoId;
    tracking_hist[tracking_id] = *maxHist;
    /*
    vector<int> okao_array;
    vector<double> hist_array;

    for(int i=0; i<OKAO;)
    okao_array.push_back(hist_to_okao[i]);
    hist_array.push_back(hist_pool[i]);

    tracking_okao[tracking_id] = ;
    tracking_hist[tracking_id] = ;
    */
    //アウトプット用関数
    //histOutputFile(hist_file, *maxOkaoId);

  }

  void  getOkaoAndHist(long long tracking_id, int *maxOkaoId, double *maxHist)
  {
    *maxOkaoId = tracking_okao[tracking_id];
    *maxHist = tracking_hist[tracking_id];
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
  void recogProcess(long long t_id, int *okao_id, double hist, int frame)
  {

    if( frame < MAX_FRAME )
      {
	if( hist < HIST_THRESHOLD )
	  {
	    t_determined[t_id] = false;
	    t_known[t_id] = false;
	  }
	else
	  {
	    t_determined[t_id] = true;
	    t_known[t_id] = true;
	  }
      }
    else
      {
	t_determined[t_id] = true;
	if( (frame%MAX_FRAME) == 0 && t_known[t_id] == false )
	  {
	    if( hist > HIST_THRESHOLD*(frame/MAX_FRAME) )
	      {
		t_known[t_id] = true;
	      }
	  }
      }

  }
  

  void okaoProcess(humans_msgs::Human src, humans_msgs::Humans *dst, 
		   std_msgs::Header src_header)
  {

    long long tracking_id = src.body.tracking_id;
    int maxOkaoId = 0;
    double maxHist = 0.0;
    int o_id[OKAO] = {0}, o_conf[OKAO] = {0};
    if( tracking_id )
      {
	vector<long long>::iterator tracking_id_find 
	  = find(tracking_id_buffer.begin(), tracking_id_buffer.end(), tracking_id);	
	
	//tracking_idを過去に見ていたかどうかを調べる(d_id)
	if( tracking_id_find == tracking_id_buffer.end())
	  {
	    tracking_id_buffer.push_back(tracking_id); 
	    
	    for(int i = 0; i < src.face.persons.size(); ++i)
	      {
		o_id[i] 
		  = src.face.persons[i].okao_id;
		o_conf[i] 
		  = src.face.persons[i].conf;
		
		//personの保持
		humans_msgs::Person ps;
		ps = src.face.persons[i];
		
		prop_buf[ ps.okao_id ] = ps;
	      }

	    if(src.face.persons.size())
	      {	    
		
		histogram( tracking_id, o_id, o_conf, &maxOkaoId, &maxHist );
		recogProcess( tracking_id, &maxOkaoId, maxHist, id_num[tracking_id]);
		
		cout <<"face found[ " << ros::Time::now() 
		     << " ], tracking_id: "
		     << tracking_id << " ---> max id: "
		     << maxOkaoId <<", max hist: " << maxHist << ", frame: " 
		     << id_num[tracking_id] <<endl;
	      }	
	    else
	      {
		cout <<"face not found[ " << ros::Time::now() 
		     << " ], tracking_id: "
		     << tracking_id << " ---> max id: "
		     << maxOkaoId <<", max hist: " << maxHist << endl;
	      }
	  }	 
	else
	  {	    	    
	    //もし,顔を発見していたら以下の処理を行う    	
	    //メッセージ内にある一人の人物につき、一位から三位までの個人情報を取得する。
	    for(int i = 0; i < src.face.persons.size(); ++i)
	      {
		o_id[i] 
		  = src.face.persons[i].okao_id;
		o_conf[i] 
		  = src.face.persons[i].conf;
		
		//personの保持
		humans_msgs::Person ps;
		ps = src.face.persons[i];
		
		prop_buf[ ps.okao_id ] = ps;
	      }
	    	    	
	    if(src.face.persons.size())
	      {	    
		
		histogram( tracking_id, o_id, o_conf, &maxOkaoId, &maxHist );
		//人物をframe数とhistで確定するプロセス
		//もしx frame以下なら、マイナスをつける
		recogProcess( tracking_id, &maxOkaoId, maxHist, id_num[tracking_id]);
		
		cout <<"face found[ " << ros::Time::now() 
		     << " ], tracking_id: "
		     << tracking_id << " ---> max id: "
		     << maxOkaoId <<", max hist: " << maxHist << ", frame: " 
		     << id_num[tracking_id] <<endl;
	      }	
	    else
	      {
		getOkaoAndHist( tracking_id, &maxOkaoId, &maxHist);
		//人物をframe数とhistで確定するプロセス
		//もしx frame以下なら、マイナスをつける
		recogProcess( tracking_id, &maxOkaoId, maxHist, id_num[tracking_id]);

		cout <<"face not found[ " << ros::Time::now() 
		     << " ], tracking_id: "
		     << tracking_id << " ---> max id: "
		     << maxOkaoId <<", max hist: " << maxHist << endl;
	      }
	  }

	//if(maxHist > 0)
	//  {
	humans_msgs::Human h;
	ros::Time t = src_header.stamp;
	
	h.body.joints = src.body.joints;
	h.body.tracking_id = src.body.tracking_id;	
	h.body.is_tracked = src.body.is_tracked;
	h.body.left_hand_state = src.body.left_hand_state;
	h.body.right_hand_state = src.body.right_hand_state;
	//h.d_id = d_id;
	h.max_okao_id = maxOkaoId;
	h.max_hist = maxHist;
	//h.magni = magni;
	h.header.stamp = t;
	h.header.frame_id = src_header.frame_id;
	h.p = src.body.joints[ HEAD ].position;
	if( t_known[ tracking_id ] && t_determined[ tracking_id ] )
	  h.state = 2;
	else if( t_determined[ tracking_id ] )
	  h.state = 1;
	else
	  h.state = 0;

	//if(t_known[tracking_id])
	//  {
	h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	    //  }
	    //else
	    //{
	    // h.face.persons.push_back( prop_buf[ -1 ] );
	    // }
	//h.face.persons.push_back( prop_buf[ maxOkaoId ] );
	dst->human.push_back( h );
	// }
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
	okaoProcess(okaoNot->human[p_i], &recog, okaoNot->header);
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
