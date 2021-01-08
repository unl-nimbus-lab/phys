/*
2015.1.22------------
okaoとokaoNotは、やはり同じパブリッシュ先を持つべき

2014.12.8-------------
画像とjointを同期
 
 */


//basic
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
//ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
//オリジナルのメッセージ
#include <humans_msgs/Humans.h>
//#include "okao_client/OkaoStack.h"
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//通信用
#include <msgpack.hpp>
#include "zmq.hpp"
#include "zmq.h"
#include "picojson.h"
#include "Message.h"
//message
#include "JsonToMsg.hpp"
#include "MsgToMsg.hpp"

#define OKAO 3 
#define SRVTEMPO 10
#define NECK 2
#define HEAD 3
#define SHOULDER_L 4
#define SHOULDER_R 8
#define SPINE_S 20

#define HORIZON 70
#define VERTICAL 60
#define FACE_W 0.15
#define FACE_H 0.20
#define MAX_W 1920
#define MAX_H 1080


static const std::string OPENCV_WINDOW = "OKAO Client Window";
using namespace cv;
using namespace std;



class OkaoClient 
{
public:
  OkaoClient() :
    it_(nh_),
    image_sub_( it_, "/camera/image/color", 100 ),
    humans_sub_( nh_, "/humans/kinect_v2", 100 ),
    sync( MySyncPolicy( 100 ), image_sub_, humans_sub_ )
  {
    sync.registerCallback( boost::bind( &OkaoClient::okaoClientCallback, this, _1, _2 ) );

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

    image_pub_ 
      = it_.advertise("/camera/image/face_detect",1);
    discovery_pub_ 
      = nh_.advertise<humans_msgs::Humans>("/humans/okao_server",10);
    undiscovered_pub_ 
      = nh_.advertise<humans_msgs::Humans>("/humans/okao_server_not",10);
    //ウィンドウ
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~OkaoClient()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    if( responder )
      {
    	delete responder;
      } 
    //zmq_close( responder );
  } 
 
  std::string idToEncoding(int type)
  { 
    // cannot unfortunately be in a switch
    switch (type)
      {
      case CV_8UC1:
	return "mono8";
      case CV_16UC1:
	return "mono16";
      case CV_8UC3:
	return "bgr8";
      case CV_8UC4:
	return "bgra8";
      case CV_16UC3:
	return "bgr16";
      case CV_16UC4:
	return "bgra16";
      default:
	return "";
      }
}
  

  void okaoClientCallback(
			  const sensor_msgs::ImageConstPtr& imgmsg,
			  const humans_msgs::HumansConstPtr& kinect
			  )
  {  
    int okao_i = 0;
    int no_okao_i = 0; 
    humans_msgs::Humans okao_human, no_okao_human;   
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(imgmsg, sensor_msgs::image_encodings::BGR8);
	for(int i = 0; i < kinect->num; i++)
	  {
	    //POS head2d, head3d, neck2d;
	    Point top, bottom;
	    geometry_msgs::Point head2d, neck2d;//, top, bottom;
	    head2d.x 
	      = kinect->human[i].body.joints[HEAD].position_color_space.x;	   
	    head2d.y 
	      = kinect->human[i].body.joints[HEAD].position_color_space.y;

	    neck2d.x 
	      = kinect->human[i].body.joints[SPINE_S].position_color_space.x;	   
	    neck2d.y 
	      = kinect->human[i].body.joints[SPINE_S].position_color_space.y;

	    double diff_w =  fabs(head2d.y-neck2d.y);
	    double diff_h =  fabs(head2d.y-neck2d.y);

	    top.x = head2d.x - diff_w;
	    top.y = head2d.y - diff_h;

	    bottom.x = head2d.x + diff_w;
	    bottom.y = head2d.y + diff_h;
	    /*
	    cout << "cut (" << cut.x << "," << cut.y << ")"<<endl;
	    if( cut.x < 0 )
	      cut.x = 0;
	    else if( cut.x >= (cv_ptr->image.cols - diff_w*2) )
	      cut.x = cv_ptr->image.cols-diff_w*2-1;

	    if( cut.y < 0 )
	      cut.y = 0;
	    else if( cut.y >= (cv_ptr->image.rows - diff_h*2) )
	      cut.y = cv_ptr->image.rows-diff_h*2-1;
	    */
	    top.x = max(0, top.x);
	    top.y = max(0, top.y);
	    bottom.x = min(cv_ptr->image.cols-1, bottom.x);
	    bottom.y = min(cv_ptr->image.rows-1, bottom.y);

	    if (( top.x > bottom.x || top.y > bottom.y)||( top.x == bottom.x || top.y == bottom.y))
	      continue;
	    /*   
	    cout << "(" << top.x << "," << top.y << ")"
		 << "-"
		 << "(" << bottom.x << "," << bottom.y << ")"<<endl;
	    cout << "diff:" << "(" << diff_w << "," << diff_h<< ")" << endl;
	    cout << "image:" << "(" << cv_ptr->image.cols << "," << cv_ptr->image.rows << ")" << endl;
	    */
	    Mat cutRgbImage;
	    try
	      {
		cutRgbImage = Mat(cv_ptr->image, cv::Rect(top, bottom));
	      }
	    catch(cv_bridge::Exception& e)
	      {
		ROS_ERROR("cv_bridge exception: %s",e.what());
	      }
	    Mat rgbImage = cutRgbImage.clone();
	    if( rgbImage.cols > 1280 )
	      {
		cv::resize( rgbImage, rgbImage, 
			    cv::Size(1280, cutRgbImage.rows*1280/cutRgbImage.cols) );	
	      }
	    if( rgbImage.rows > 1024 )
	      {
		cv::resize( rgbImage, rgbImage, 
			    cv::Size(cutRgbImage.cols*1024/cutRgbImage.rows , 1024) );
	      }	

	    //rgbImage = cutRgbImage;	     
	    Mat grayImage;	   
	    cv::cvtColor(rgbImage,grayImage,CV_BGR2GRAY);

	    //test
	    cv::rectangle( cv_ptr->image, top,
			   bottom,
			   cv::Scalar(0,200,0), 5, 8);
	    
	    try
	      {
		cv::Mat img = grayImage;
		std::vector<unsigned char> 
		  buf(img.data, img.data + img.cols * img.rows * img.channels());

		std::vector<int> encodeParam(2);
		encodeParam[0] = CV_IMWRITE_PNG_COMPRESSION;
		encodeParam[1] = 3;
		cv::imencode(".png", img, buf, encodeParam);
		
		picojson::object p;	
		p.insert(std::make_pair("mode",
					picojson::value(std::string("FaceRecognition"))));
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
		// 送信
		OkaoServer::sendRequestMessage(*responder, reqMsg);
		// 受信
		OkaoServer::ReplyMessage repMsg;
		OkaoServer::recvReplyMessage(*responder, &repMsg);
		//std::cout << "repMsg.okao: " << repMsg.okao << std::endl;
	    	
		const char* json = repMsg.okao.c_str();
		picojson::value v;
		std::string err;
		picojson::parse(v,json,json + strlen(json),&err);
		if(err.empty())
		  {
		    humans_msgs::Face face_msg;
		    humans_msgs::Body body_msg;
		    bool p_ok = false;
		    JsonToMsg::face(v, &face_msg, top.x, top.y, &p_ok, 0);		    
		    MsgToMsg::bodyToBody(kinect->human[i].body, &body_msg);
		    
		    if( p_ok )
		      {
			++okao_i;
			humans_msgs::Human h;
			h.body = body_msg;
			h.face.persons.resize( OKAO );
			h.face = face_msg;
			okao_human.human.push_back( h );
			cv::Point lt(face_msg.position.lt.x, face_msg.position.lt.y);
			cv::Point rb(face_msg.position.rb.x, face_msg.position.rb.y);
			cv::Point rb_out = bottom;
			cv::Scalar red(0,0,200);
			cv::Scalar green(0,200,0);
			cv::rectangle( cv_ptr->image, lt, rb, red, 5, 8);
		
			cv::putText( cv_ptr->image, face_msg.persons[0].name, 
				     rb_out, FONT_HERSHEY_SIMPLEX, 2.5, 
				     green, 2, CV_AA);
			
			/*
			ros::ServiceClient client = nh_.serviceClient<
			  okao_client::OkaoStack>("stack_add");
			okao_client::OkaoStack stack;
			//stack.request.rule = "add";
			stack.request.person = face_msg.persons[0];
			
			sensor_msgs::Image output;
			cv::Mat outcutImage;
			cv::resize(rgbImage, outcutImage, cv::Size(128,128));
			output.height = outcutImage.rows; 
			output.width = outcutImage.cols;
			output.encoding = idToEncoding( outcutImage.type() );
			output.step 
			  = outcutImage.cols * outcutImage.elemSize();
			output.data.assign(outcutImage.data, outcutImage.data + size_t(outcutImage.rows*output.step));
			stack.request.image = output;
			
			if ( !client.call(stack) )
			  cout << "service missing!" << endl;	
			*/
		      }
		    else
		      {
			++no_okao_i;
			humans_msgs::Human h;
			h.body = body_msg;
			no_okao_human.human.push_back( h );
		      }
		  }	       
	      }    
	    catch(const zmq::error_t& e)
	      {
		std::cout << e.what() << std::endl;
		return ;
	      }	    
	  }
	//パブリッシュ
	okao_human.num = okao_i;
	no_okao_human.num = no_okao_i;
	okao_human.header.stamp = no_okao_human.header.stamp = ros::Time::now();
	okao_human.header.frame_id =  kinect->header.frame_id;
	no_okao_human.header.frame_id = kinect->header.frame_id;
	discovery_pub_.publish(okao_human);
	undiscovered_pub_.publish(no_okao_human);
	image_pub_.publish(cv_ptr->toImageMsg());
	//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	//cv::waitKey(1);
      }
    catch(cv_bridge::Exception& e)
      {	
	ROS_ERROR("cv_bridge exception: %s",e.what());
      } 
  }
  
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  ros::Publisher discovery_pub_;
  ros::Publisher undiscovered_pub_; 
  image_transport::Publisher image_pub_;
 
  zmq::context_t context;
  zmq::socket_t* responder; 

  typedef image_transport::SubscriberFilter ImageSubscriber;
  typedef message_filters::Subscriber< humans_msgs::Humans > HumansSubscriber;
  
  ImageSubscriber image_sub_;
  HumansSubscriber humans_sub_;
  
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, humans_msgs::Humans
    > MySyncPolicy;
  
  message_filters::Synchronizer< MySyncPolicy > sync;
};

int main(int argc, char** argv) 
{
  ros::init( argc, argv, "okao_client" );
  OkaoClient OCObj;
 
  ros::spin();
  
  return 0;
}


