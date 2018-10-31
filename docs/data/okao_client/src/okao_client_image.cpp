#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
//ROS
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//オリジナルのメッセージ
#include <humans_msgs/Humans.h>
//#include "okao_client/OkaoStack.h"
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//通信用ライブラリ
#include <msgpack.hpp>
#include "zmq.hpp"
#include "zmq.h"

#include "picojson.h"
#include "Message.h"
#include "JsonToMsg.hpp"

#define OKAO 3 
#define SRVTEMPO 10

static const std::string OPENCV_WINDOW = "OKAO Client Window";

using namespace cv;
using namespace std;

int callbackCount = 0;
bool stackSrv = false;

class ImageConverter
{
private:
  ros::NodeHandle nh_;

  //パブリッシャとサブスクライバの定義
  ros::Publisher okaoData_pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;

  zmq::context_t context;
  zmq::socket_t* responder; 

  double f_cb_time, n_cb_time;

public:
  ImageConverter()
    : it_(nh_), context(3)
  {
    //画像データのサブスクライブとパブリッシュ
    image_sub_ = it_.subscribe("/usb_cam/image_raw",1, 
			       &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video",1);

    //検出した顔データのパブリッシュ
    okaoData_pub_ = nh_.advertise<humans_msgs::Humans>("/humans/okao_server",10);

    //ソケットの作成
    responder = new zmq::socket_t(context, ZMQ_REQ);
    assert( responder );
    try
      {
	responder->connect("tcp://133.19.23.33:50001");
      }
    catch(const zmq::error_t& e)
      {
	cout <<"Server Conect Error: " << e.what() << endl;
	return;
      } 
    //ウィンドウ
    cv::namedWindow(OPENCV_WINDOW);

    f_cb_time = ros::Time::now().toSec();
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    if( responder )
      {
	delete responder;
      } 
    //zmq_close( responder );
  }
  
  //コールバック関数
  void imageCb(const sensor_msgs::ImageConstPtr& imgmsg)
  {
    humans_msgs::Humans hums; 
    Mat rgbImage,grayImage;//画像イメージ格納用
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	//画像メッセージ(ROS)をMat型(OpenCV)に代入し、グレイスケール変換する
	cv_ptr = cv_bridge::toCvCopy(imgmsg, sensor_msgs::image_encodings::BGR8);
	rgbImage = cv_ptr->image;
	//resize(rgbImage, rgbImage, Size(1280, 720));
	cv::cvtColor(rgbImage,grayImage,CV_BGR2GRAY);

      }
    catch(cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s",e.what());
	return;
      }

    try
      {
	//画像の読み込み
	cv::Mat img = grayImage;//cv_ptr->image;
	//cout <<"cols:" << img.cols << ",rows:"<<img.rows << endl;
	std::vector<unsigned char> buf(img.data, img.data + img.cols * img.rows * img.channels());
	double width =  rgbImage.cols;
	double height = rgbImage.rows;
	double resize_width =  width / img.cols; 
	double resize_height = height / img.rows;
	//cout<<"img.cols: "<< img.cols <<",img.rows:"<< img.rows<< endl;
	//cout<<"resize_width: "<< resize_width << endl;
	// 画像をエンコードする必要があれば
	std::vector<int> encodeParam(2);
	encodeParam[0] = CV_IMWRITE_PNG_COMPRESSION;
	encodeParam[1] = 3;
	cv::imencode(".png", img, buf, encodeParam);
	//std::cout << "encoded size: " << buf.size() << std::endl;
	// 画像のパラメータ
	// エンコードした際はformatをRAWからPNG等に変えること
	// また、width, heightも適切に設定すること	
	picojson::object p;	
	p.insert(std::make_pair("mode",picojson::value(std::string("FaceRecognition"))));
	p.insert(std::make_pair("format",picojson::value(std::string("PNG"))));	
	p.insert(std::make_pair("width",picojson::value((double)img.cols)));
	p.insert(std::make_pair("height",picojson::value((double)img.rows)));
	p.insert(std::make_pair("depth",picojson::value((double)1)));

	picojson::value para = picojson::value(p); 
	std::string param = para.serialize().c_str();
	// リクエストメッセージの作成
	OkaoServer::RequestMessage reqMsg;
	reqMsg.img = buf;
	reqMsg.param = param;
	//	cout << "send to OKAOServer" << endl;

	// 送信
	double before = ros::Time::now().toSec();
	OkaoServer::sendRequestMessage(*responder, reqMsg);
	// 受信
	//cout << "receive from OKAOServer" << endl;
	OkaoServer::ReplyMessage repMsg;
	OkaoServer::recvReplyMessage(*responder, &repMsg);
	double after = ros::Time::now().toSec();
	//通信時間の計測
	double between = ros::Duration(after-before).toSec();

	//異常に遅かったらファイル出力する
	if(between > 0.5)
	  {
	    time_t now = time(NULL);
	    struct tm *pnow = localtime(&now);

	    std::ofstream ofs("error_log.txt", std::ios::out | std::ios::app);
	    ofs <<"send to recv time[" << pnow->tm_hour << ":" << pnow->tm_min 
		<<":" << pnow->tm_sec << "]" << std::endl 
		<<"between:" << between << std::endl
		<<"rep msg:"<< std::endl
		<< repMsg.okao<< std::endl;
	  }

	//std::cout << "repMsg.okao: " << repMsg.okao << std::endl;	
	const char* json = repMsg.okao.c_str();
	picojson::value v;
	std::string err;
	picojson::parse(v,json,json + strlen(json),&err);
	if(err.empty())
	  {
	    humans_msgs::Face face_msg;
	    bool p_ok = false;
	    JsonToMsg::face(v, &face_msg, 0, 0, &p_ok, 0);	

	    if( p_ok )
	      {

		/*
		for(int i = 0; i < face_msg.points.size(); ++i)
		  {
		    stringstream ss;
		    ss << i;
		    cv::circle( rgbImage, cv::Point(face_msg.points[i].x,face_msg.points[i].y), 10,
				red, 3, 4);
		    cv::putText( rgbImage, ss.str(), 
				 cv::Point(face_msg.points[i].x,face_msg.points[i].y), FONT_HERSHEY_SIMPLEX, 2.5, 
				 green, 2, CV_AA);
		  } 
		*/
		int lon = (face_msg.points[5].x - face_msg.points[3].x)*3;
		int rootX = face_msg.points[3].x + (face_msg.points[5].x - face_msg.points[3].x)/2;
		int rootY = face_msg.points[3].y + (face_msg.points[5].y - face_msg.points[3].y)/2;

		cv::Scalar color_f;
		cv::Scalar color_g;

		if(face_msg.gaze_direction.conf > 200)
		  {
		    int dir_g_h = face_msg.gaze_direction.x;
		    int dir_g_v = face_msg.gaze_direction.y;
		    int dir_g_c = face_msg.gaze_direction.conf;
		    
		    int dir_f_h = face_msg.direction.x;
		    int dir_f_v = face_msg.direction.y;
		    int dir_f_c = face_msg.direction.conf;

		    cout <<"gaze deg_v:" << dir_g_v << ", deg_h:" << dir_g_h << ", conf:"<< dir_g_c <<endl;
		    cout <<"face deg_v:" << dir_f_v << ", deg_h:" << dir_f_h << ", conf:"<< dir_f_c << endl;

		    //double rad = atan2((double)sin(dir_y*M_PI/180.), sin((double)dir_x*M_PI/180.));
		    //double rad_f = atan2((double)sin(dir_f_y*M_PI/180.), sin((double)dir_f_x*M_PI/180.));
		    double rad_f = (double)dir_f_h*M_PI/180.;
 		    double rad_g = (double)dir_g_h*M_PI/180.;
		    //cout << "rad_f to deg:" << rad_f * (180./M_PI) <<endl;
		    //cout << "deg:" << rad * (180/M_PI) << endl;
		    //double x_rad = direction_x*M_PI/180;
		    //double y_rad = face_msg.gaze_direction.y*M_PI/180;
		    //cout <<"sin(gaze): " << sin(rad_g) << ", sin(face): " << sin(rad_f)<<endl; 

		    int tol = 7;
		    /*
		    if(dir_f_h < 0 && dir_g_h > 0)
		      {
			color_g = cv::Scalar(0,0,255);
			color_f = cv::Scalar(255,0,0);
		      }
		    else if(dir_f_h > 0 && dir_g_h < 0)
		      {
			color_g = cv::Scalar(0,255,255);
			color_f = cv::Scalar(255,255,0);
		      }
		    else if( (abs(abs(dir_f_h) - abs(dir_g_h) ) < tol) && abs(dir_f_h) < tol )
		      {
			color_g = cv::Scalar(255,0,255);
			color_f = cv::Scalar(255,0,255);
		      }
		    else
		      {
			color_g = cv::Scalar(0,0,0);
			color_f = cv::Scalar(0,0,0);
		      }
		    */
		    if( (abs(abs(dir_f_h) - abs(dir_g_h) ) < tol) && (abs(dir_f_h) < tol) )
		      {
			color_g = cv::Scalar(255,0,255);
			color_f = cv::Scalar(255,0,255);
		      }
		    else
		      {
			color_g = cv::Scalar(0,0,0);
			color_f = cv::Scalar(0,0,0);
		      }
		    cv::line( rgbImage, cv::Point(rootX, rootY),  cv::Point(rootX+lon*sin(rad_g), rootY),color_g, 3, 4);
		    //cv::line( rgbImage, cv::Point(rootX, rootY),  cv::Point(lon*cos(y_rad), lon*sin(y_rad)),red, 3, 4);
		    int face_w = face_msg.position.rb.x - face_msg.position.lt.x;
		    int face_h = face_msg.position.rb.y - face_msg.position.lt.y;
		    int face_cx = face_msg.position.lt.x + face_w / 2;
		    int face_cy = face_msg.position.lt.y + face_h / 2;
		    cv::line( rgbImage, cv::Point(face_cx, face_cy),  cv::Point(face_cx+lon*sin(rad_f), face_cy),color_f, 3, 4);
		  }

		cv::Point lt(face_msg.position.lt.x, face_msg.position.lt.y);
		cv::Point rb(face_msg.position.rb.x, face_msg.position.rb.y);
		//cv::Point rb_out = bottom;

		//cv::rectangle( rgbImage, lt, rb, red, 5, 8);		
		//cv::putText( rgbImage, face_msg.persons[0].name, 
		//	     rb, FONT_HERSHEY_SIMPLEX, 2.5, 
		//	     green, 2, CV_AA);
	      }

	    humans_msgs::Human hum;
	    hum.face = face_msg;
	    hums.human.push_back(hum);
	    cv::imshow(OPENCV_WINDOW, rgbImage);
	    
	    cv::waitKey(1);
	  }
      }   
    catch(const zmq::error_t& e)
      {
	std::cout << e.what() << std::endl;
	return ;
      }   


    okaoData_pub_.publish(hums);

    n_cb_time = ros::Time::now().toSec();
    double cb_between = ros::Duration(n_cb_time - f_cb_time).toSec();
    //異常に遅かったらファイル出力する
    if(cb_between > 0.5)
      {
	time_t now = time(NULL);
	struct tm *pnow = localtime(&now);
	
	std::ofstream ofs("error_log.txt", std::ios::out | std::ios::app);
	ofs <<"callback time[" << pnow->tm_hour << ":" << pnow->tm_min 
	    <<":" << pnow->tm_sec << "]" << std::endl 
	    <<"cb_between:" << cb_between << std::endl;
      }
    f_cb_time = n_cb_time;

  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "okao_client_image");
  
  ImageConverter ic;
  
  ros::spin();
  return 0;
}
