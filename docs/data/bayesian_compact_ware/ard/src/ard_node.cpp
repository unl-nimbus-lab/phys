//rosservice call /ardrone/togglecam hsv129 canny edge 106 centerdetection47
//joystick_ps3.launch
/*  <node name="kalman_corrector" pkg="hough_test" type="kalman_corrector.py" output="screen">
    </node>
    <!-- required="true" kill the roslaunch if this node dies-->
    ard
    <node name="ard_node" pkg="ard" type="ard_node" output="screen">
    </node>

    <node name="hough_test_node" pkg="hough_test" type="hough_test_node" output="screen">
    </node>

	*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <stdio.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>


using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
float previous_error_x = 0.0;
float previous_error_y = 0.0;
float previous_error_apr = 0.0;

static const std::string OPENCV_WINDOW1 = "random";
int first_image_frame=1;
sig_atomic_t volatile g_request_shutdown = 0;
int pubtake_done = 1;

int Glob_Kp_max = 1000;
int Glob_Kd_max = 1000;

int Kp = 0;
int Kd = 0;

//int Glob_Kwp_max = 10000;
//int Kwp = 6000;
//
//int Glob_Kwd_max = 10000;
//int Kwd = 600;//500;

//int calib_first = 1;
//int calib_count = 1;
//float corridoor_width = 1.82;
//
//
//int pub_every = -1;
//int pub_cnt_every = 0;
//
//int pub_every_van = -1;
//int pub_cnt_every_van = 0;


/*cv::namedWindow(OPENCV_WINDOW1, CV_WINDOW_NORMAL);
cv::resizeWindow(OPENCV_WINDOW1, 500,800);*/
/*
void vanish_callback(const  geometry_msgs::Pose2D::ConstPtr& msg)
{
	  geometry_msgs::Pose2D point1;
	  geometry_msgs::Twist  pub_point;
	  //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'


	  //if(!(msg->x.emp)
	  //{
		  point1.x = msg->x;
		  point1.y = msg->y;

		  float px= 329.6735;
		  float py= 187.2286;
		  float error= (float)point1.x - px;
		  float k= 0.1;

		  pub_point.linear.x = 0.1;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
		  pub_point.linear.y = 0.0;
		  pub_point.linear.z = 0.0;
		  pub_point.angular.x = 0.0;
		  pub_point.angular.y = 0.0;
		  //pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
		  pub_point.angular.z = -(k*error);

		  pioneer_pub.publish(pub_point);

	  //}

	  cout <<  "  X   " <<  (float)point1.x << endl;

	  cout <<  "                Y   " << (float)point1.y << endl;
}
*/


class code_ard
{




public:
	ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber im_sub_;
	ros::Subscriber center_kalman;
	ros::Subscriber sub_vanish;
	ros::Publisher pub_ardrone_cmdvel;
	ros::Publisher pub_ardrone_takeoff;
	ros::Publisher pub_ardrone_land;
	ros::Publisher pub_ardrone_reset;
	ros::Publisher pub_check_destructor;
	ros::Publisher pub_check_speed;

	geometry_msgs::Twist  pub_point;
	geometry_msgs::Pose2D point1;
	//geometry_msgs::Quaternion quat_out;
	//geometry_msgs::Quaternion quat_in;

	cv::VideoWriter writer_apr;
	cv::Mat image;
	std_msgs::Empty doit;
	std_msgs::String desctruct_var;
	code_ard()
	:it_(nh_)
	{
		//\comment{/ardrone/bottom/image_rect_color might need but mostly not}
		im_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, &code_ard::imageCb, this);

		center_kalman = nh_.subscribe("/corrected_centers", 1, &code_ard::kalmanCallback,this);//nh_.subscribe("/corrected_centers",1,&codebridge::kalmanread,this);
		sub_vanish = nh_.subscribe("/vanishing_point", 1, &code_ard::vanish_callback,this);
		pub_ardrone_cmdvel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		//pub_ardrone_takeoff = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
		pub_ardrone_land = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
	//	pub_ardrone_reset = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);

		pub_check_destructor = nh_.advertise<std_msgs::String>("/testdestruct",1);
		pub_check_speed = nh_.advertise<std_msgs::Empty>("/checkspeed",1);

		std::stringstream ss;
		ss << "destruction" << cout;
		desctruct_var.data = ss.str();

		cv::namedWindow("Kp_Kd_Tuning", 1);

		createTrackbar( "Kp ", "Kp_Kd_Tuning", &Kp, Glob_Kp_max);
		createTrackbar( "Kd ", "Kp_Kd_Tuning", &Kd, Glob_Kd_max);

		//cv::namedWindow("kwp kwd Tuning", 1);
//		createTrackbar( "Kwp ", "Kp Kd kwd kwp Tuning", &Kwp, Glob_Kwp_max);
//
//		//cv::namedWindow("kwd  Tuning", 1);
//		createTrackbar( "Kwd ", "Kp Kd kwd kwp Tuning", &Kwd, Glob_Kwd_max);
//	   // createTrackbar( "Kd ", "Kp Kd Tuning", &Kd, Glob_Kd_max);


	//	cv::namedWindow(OPENCV_WINDOW1);

//		  std_msgs::String msg;
//		  88
//		  89     std::stringstream ss;
//		  90     ss << "hello world " << count;
//		  91     msg.data = ss.str();
//		  92
//		  93     ROS_INFO("%s", msg.data.c_str());
	}
	~code_ard()
	{
		cout<<"in_distructer"<<endl;
		//pub_ardrone_reset.publish(doit); // doit is a empty msg
		//pub_ardrone_land.publish(doit);

		//ros::spinOnce();
	//    cv::destroyWindow(OPENCV_WINDOW1);

	//    cv::destroyWindow("Kp_Kd_Tuning");

//	    cv::destroyWindow("kwp  Tuning");

	   // ros::Duration(1).sleep();

	}
	/* multiply quaternion quat_in1*quat_in2 not the opposite direction*/
/*		geometry_msgs::Quaternion quat_mult(geometry_msgs::Quaternion quat_in1,  geometry_msgs::Quaternion quat_in2)
		{
			geometry_msgs::Quaternion quat_out;
			float t0,t1,t2,t3,q0,q1,q2,q3,r0,r1,r2,r3;

			q0 = quat_in1.x;
			q1 = quat_in1.y;
			q2 = quat_in1.z;
			q3 = quat_in1.w;

			r0 = quat_in2.x;
			r1 = quat_in2.y;
			r2 = quat_in2.z;
			r3 = quat_in2.w;


			t0 = (r0*q0-r1*q1-r2*q2-r3*q3);

			t1 = (r0*q1+r1*q0-r2*q3+r3*q2);

			t2 = (r0*q2+r1*q3+r2*q0-r3*q1);

			t3 = (r0*q3-r1*q2+r2*q1+r3*q0);

			quat_out.x = t0;
			quat_out.y = t1;
			quat_out.z = t2;
			quat_out.w = t3;


			//quat_out.x = t0;

			return quat_out;
		}

		geometry_msgs::Quaternion quat_inv(geometry_msgs::Quaternion quat_in)
		{
			geometry_msgs::Quaternion quat_out;


			quat_out.x = quat_in.x;
			quat_out.y = -quat_in.y;
			quat_out.z = -quat_in.z;
			quat_out.w = -quat_in.w;

			return quat_out;

		}

		geometry_msgs::Quaternion quat_me(geometry_msgs::Point vec_in)
		{
			geometry_msgs::Quaternion quat_out;
			quat_out.x = 0;
			quat_out.y = vec_in.x;
			quat_out.z = vec_in.y;
			quat_out.w = vec_in.z;

			return quat_out;
		}

		geometry_msgs::Point unquat_me(geometry_msgs::Quaternion quat_in)
		{
			geometry_msgs::Point vec_out;

			vec_out.x = quat_in.y;
			vec_out.y = quat_in.z;
			vec_out.z = quat_in.w;

			return vec_out;
		}*/

  /* multiply quaternion quat_in1*quat_in2 not the opposite direction*/
	geometry_msgs::Quaternion quat_mult(geometry_msgs::Quaternion quat_in1,  geometry_msgs::Quaternion quat_in2)
	{
		geometry_msgs::Quaternion quat_out;
		float t0,t1,t2,t3,q0,q1,q2,q3,r0,r1,r2,r3;

		q0 = quat_in1.w;
		q1 = quat_in1.x;
		q2 = quat_in1.y;
		q3 = quat_in1.z;
		//q3 = quat_in1.w;
		r0 = quat_in2.w;
		r1 = quat_in2.x;
		r2 = quat_in2.y;
		r3 = quat_in2.z;



		t0 = (r0*q0-r1*q1-r2*q2-r3*q3);

		t1 = (r0*q1+r1*q0-r2*q3+r3*q2);

		t2 = (r0*q2+r1*q3+r2*q0-r3*q1);

		t3 = (r0*q3-r1*q2+r2*q1+r3*q0);

		quat_out.w = t0;
		quat_out.x = t1;
		quat_out.y = t2;
		quat_out.z = t3;


		//quat_out.x = t0;

		return quat_out;
	}

	geometry_msgs::Quaternion quat_inv(geometry_msgs::Quaternion quat_in)
	{
		geometry_msgs::Quaternion quat_out;


		quat_out.x = -quat_in.x;
		quat_out.y = -quat_in.y;
		quat_out.z = -quat_in.z;
		quat_out.w = quat_in.w;

		return quat_out;

	}

	geometry_msgs::Quaternion quat_me(geometry_msgs::Point vec_in)
	{
		geometry_msgs::Quaternion quat_out;
		quat_out.w = 0;
		quat_out.x = vec_in.x;
		quat_out.y = vec_in.y;
		quat_out.z = vec_in.z;

		return quat_out;
	}

	geometry_msgs::Point unquat_me(geometry_msgs::Quaternion quat_in)
	{
		geometry_msgs::Point vec_out;

		vec_out.x = quat_in.x;
		vec_out.y = quat_in.y;
		vec_out.z = quat_in.z;

		return vec_out;
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
//		if (first_image_frame)
//		{
//			string filename_apr = "/home/ashishkb/Desktop/aprtag.avi";
//			int fcc = CV_FOURCC('D','I','V','3');
//			int fps = 10;
//			cv::Size frameSize(640,360);
//			writer_apr = VideoWriter(filename_apr,fcc,fps,frameSize);
//
//			while (!writer_apr.isOpened()) {
//			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }
//
//
//			first_image_frame = 0;
//		}

		cv_bridge::CvImagePtr cv_ptr;
		cv_bridge::CvImagePtr cv_ptr_resized;
		geometry_msgs::Twist point_check;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::namedWindow("Kp_Kd_Tuning", 1);

		createTrackbar( "Kp ", "Kp_Kd_Tuning", &Kp, Glob_Kp_max);
		createTrackbar( "Kd ", "Kp_Kd_Tuning", &Kd, Glob_Kd_max);
		waitKey(3);
//		point_check.linear.x = 0.0;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
//		point_check.linear.y = 0.0;//-(kwp*net_error);
//		point_check.linear.z = 0.0;
//		point_check.angular.x = 0.0;
//		point_check.angular.y = 0.0;
//		//pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
//		point_check.angular.z = 0.0;//+(kd*diff_error);//0.0;//

//		pub_cnt_every = pub_cnt_every+1;
//
//		if(pub_cnt_every==pub_every) //publish every pub_every times that is every 3rd time currently
//		{
//			pub_ardrone_cmdvel.publish(point_check);
//		//	pub_cnt_every =0;
//		//	cout << "in_imagecb_pub" << endl;
//
//		}


	//	image=cv_ptr->image;
		//cv::waitKey(3);

	//	circle(image, Point2f(point1.x,point1.y), 2, Scalar(255,0,255), 2, CV_AA);//Point2f(pub_point.linear.x,pub_point.linear.y)

//	cv::imshow(OPENCV_WINDOW1, image);

		//writer_apr.write(image);

//	cv::waitKey(3);


	}

	void vanish_callback(const  geometry_msgs::Pose2D& msg)
	{
	//	float error1; //
	//	float k2 = 0.00;

	//	geometry_msgs::Pose2D point1;
		//geometry_msgs::Twist  pub_point;
		  //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'


		  //if(!(msg->x.emp)
		  //{
		point1.x = msg.x;
		point1.y = msg.y;




		float px= 330.4499; // ardrone bottom values
		float py= 179.0147;
		float current_error_x = (float)point1.x - px;
		float current_error_y = (float)point1.y - py;
		float diff_error_x = previous_error_x - current_error_x;
		float diff_error_y = previous_error_y - current_error_y;
		previous_error_x = current_error_x;
		previous_error_y = current_error_y;
		float kp=0;//= 0.0037;  // 100 slider 10 -> 0.001 --- 100 -> 0.01
		float kd=0;//= 0.001;  /// 1000 slider 10 -> 0.0001  100 -> 0.001   1000 -> 0.01




		//waitKey(3);


		kp = (float)Kp/1000;  //370/100000 =
		kd = (float)Kd/1000;

		pub_point.linear.y = -(kp*current_error_x)+(kd*diff_error_x);///'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
		pub_point.linear.x = -(kp*current_error_y)+(kd*diff_error_y);
		//pub_point.linear.z = 0.0;
	//	pub_point.angular.x = 0.0;
	//	pub_point.angular.y = 0.0;
		//pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
		pub_point.angular.z = 0.0; //-(kp*current_error)+(kd*diff_error);//0.0;//
/*
		if(pub_point.angular.z < -1)
		{
			pub_point.angular.z=-1;
		}
		else if(pub_point.angular.z>1)
		{
			pub_point.angular.z=1;
		}
*/

//		if(pub_cnt_every_van == pub_every_van) //publish every pub_every times that is every 3rd time currently
//		{
//			//pub_ardrone_cmdvel.publish(pub_point);
//			//pub_cnt_every_van =0;
//
//		}
	//	pub_ardrone_cmdvel.publish(pub_point);
	//	pub_check_speed.publish(doit);
//		pub_cnt_every_van = pub_cnt_every_van+1;

		  //}
		cout << "   cmd_x"  << pub_point.linear.x  << " kp "<<kp<< "     kd" << kd; //kd*diff_error
		cout <<    "   cmd_y" << pub_point.linear.y  << "  error_x  "<< current_error_x << "  error_y "<< current_error_y << endl;
		 // cout <<  "  X   " <<  (float)point1.x;

		//  cout <<  "                Y   " << (float)point1.y << endl;


		//ardrone_cmdvel.publish();
//		if(pubtake_done == 1)
//		{
//			pub_ardrone_reset.publish(doit);
//			pub_ardrone_reset.publish(doit);
//			ros::Duration(1).sleep();
//			pub_ardrone_takeoff.publish(doit);
//			cout<< "takeoff" << endl;
//			ros::Duration(3).sleep();
//			pubtake_done = 0;
//		}
		//cout<< "in_vanish_callback"<< endl;

	}

	void kalmanCallback(const  geometry_msgs::Pose2D& msg)
	{

/*

	//  float width_error_pos = 0;
	//  float width_error_neg = 0;
	  float width_error_cur = 0;
	  float net_error = 0;
	  float sum_error = 0;
	  float kwp = 0.00;
	  float kwd = 0.00;
	  int size = 0;
	  float width_1 = 0;
	  float width_2 = 0;
	  int id_1 = 0; //first id taken for calculating corridoor width
	  int id_2 = 0; //second id taken to calculate corridoor width
	  int lock_odd =0;
	  int lock_even = 0;
	  cout<< "next read " << endl<<endl;

	  if(!(msg->detections.empty()))
	  {
			size = msg->detections.size();
			geometry_msgs::Twist  pub_point_apr;
			geometry_msgs::Pose point_detect_ar[size];
			geometry_msgs::Pose point_arframe[size];

		//cout << size << endl;
//		  if((size > 1) && (calib_first==1) && (calib_count <= 30))

//		  {
//			  for(int i = 0; i < size; i++)
//			  {
//			  	  point_detect_ar[i].x = msg->detections[i].pose.pose.position.x;
//			  	//  point_detect_ar[i].y = msg->detections[i].pose.pose.position.y; // not needed
//			  	  if((msg->detections[i].id%2 == 1) && (lock_odd ==0) )
//			  	  {
//			  		  id_1 = msg->detections[i].id;
//			  		  cout<< "got_first" << endl;
//			  		  lock_odd = lock_odd + 1;
//			  	  }
//			  	  else if((msg->detections[i].id%2 == 0)  && (lock_even == 0))
//			  	  {
//			  		  id_2 = msg->detections[i].id;
//			  		  cout<< "got_second" << endl;
//			  		  lock_even = lock_even + 1;
//			  	  }
//
//
//
//			  }
//			  corridoor_width = (corridoor_width + abs(point_detect_ar[0].x - point_detect_ar[1].x))/calib_count;
//
//			  if(calib_count == 30)
//			  {
//				  calib_first = 0; // make it zero when we are done with 30 readings atleast
//				  cout << "corridoor width " << corridoor_width << endl;
//			  }
//			  calib_count = calib_count + 1;
//
//		  }
//  else
//  {



			for(int i = 0; i < size; i++)
			{
				point_detect_ar[i].orientation = msg->detections[i].pose.pose.orientation;
				point_detect_ar[i].position = msg->detections[i].pose.pose.position;
				//  point_detect_ar[i].y = msg->detections[i].pose.pose.position.y;
				point_arframe[i].orientation = quat_mult(point_detect_ar[i].orientation, quat_me(point_detect_ar[i].position));
				point_arframe[i].orientation = quat_mult(point_arframe[i].orientation, quat_inv(point_detect_ar[i].orientation));
				point_arframe[i].position	 = unquat_me(point_arframe[i].orientation);
			//	cout <<  "april frame z:  " << point_arframe[i].position.z << "april frame x:  " << point_arframe[i].position.x <<"the y:" <<point_arframe[i].position.y <<endl;

//
				if (msg->detections[i].id%2 == 0)//msg->detections[i].id%2 == 1)//((msg->detections[i].id%2 == 1))
				{
				  width_error_cur = (point_arframe[i].position.z + corridoor_width/2);
				  cout << "rightwall " << width_error_cur << endl;
				}
				else if (msg->detections[i].id%2 == 1)//msg->detections[i].id%2 == 0)//if((msg->detections[i].id%2 == 0))
				{
				  width_error_cur =  -(point_arframe[i].position.z + corridoor_width/2);
				  cout << "leftwall " << width_error_cur << endl;
				}
				sum_error = sum_error + width_error_cur;
				cout << "z in meter?  "<< point_arframe[i].position.z <<  "size "<< size << endl;

			}

			kwp = (float)Kwp/10000;
			kwd = (float)Kwd/10000;

			net_error = sum_error/size;
			float current_error = net_error;//(float)point1.x - px;
			float diff_error = previous_error_apr-current_error;
			previous_error_apr = current_error;

			//pub_point_apr.linear.x = 0.09;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
			pub_point_apr.linear.y = (kwp*current_error) - kwd*(diff_error);//0.1;//;
		//	pub_point_apr.linear.z = 0.0;
		//	pub_point_apr.angular.x = 0.0;
		//	pub_point_apr.angular.y = 0.0;
			//pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
		//	pub_point_apr.angular.z = 0.0;//+(kd*diff_error);//0.0;//
			if(pub_point_apr.linear.y < -1)
			{
				pub_point_apr.linear.y = -1;
			}
			else if(pub_point_apr.linear.y > 1)
			{
				pub_point_apr.linear.y = 1;
			}

			pub_ardrone_cmdvel.publish(pub_point_apr);
			//pub_check_speed.publish(doit);
			cout << "LINEAR Y  " <<  pub_point_apr.linear.y << "   kwp   " << kwp << endl;
			cout << "  size "<< size << "  neterr "<< net_error << " diff_error "<< diff_error << endl;//"       " <<kd <<endl; //kd*diff_error



		//  }

//
//
//		  point1.x = msg->detections[0].pose.pose.position.x;
//		  point2.x = msg->detections[1].pose.pose.position.x;
//		  point1.y = msg->detections[0].pose.pose.position.y;
//
//		  int tag= msg->detections[0].id;
//		  int tag1= msg->detections[1].id;
//		  if(tag1!= tag)
//		  {
//			 float width= point1.x+point2.x;
//
//		  }
//
//
//		  float px= 329.6735;
//		  float py= 187.2286;
//
//		  float current_error= (float)point1.x - px;
//		  float diff_error =previous_error-current_error;
//		  previous_error=current_error;
//		  float kp=0;//= 0.0037;  // 100 slider 10 -> 0.001 --- 100 -> 0.01
//		  float kd=0;//= 0.001;  /// 1000 slider 10 -> 0.0001  100 -> 0.001   1000 -> 0.01
//
//
//
//
//		  		//waitKey(3);
//
//
//		  kp = (float)Kp/1000;
//		  kd = (float)Kd/1000;
//
//
//		  error= (float)point1.x - px;
//		  pub_point.linear.x = 0.0;//'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]';
//		  pub_point.linear.y = 0.0;
//		  pub_point.linear.z = 0.0;
//		  pub_point.angular.x = 0.0;
//		  pub_point.angular.y = 0.0;
//		  //pub_point.angular.z = 0.0; //'[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.1]';
//		  pub_point.angular.z = -(kp*current_error)+(kd*diff_error);
//
//		  //pub_ardrone_cmdvel.publish(pub_point);
//		  //pub_check_speed.publish(doit);
//		  //pioneer_pub.publish(pub_point);
//		  cout <<   pub_point.angular.z << "      " << kp << "       " << kd << endl;
	  }


	  //cout <<  "  X   " <<  (float)point1.x;

	  //cout <<  "                Y   " << (float)point1.y;
	  //cout <<  "                             (-k*(error))   " << (float)-(k*error) << endl;

*/
	}

};


void mySigIntHandler(int sig)
{
		  g_request_shutdown = 1;
		  cout<<"in_sigint"<< endl;
		  code_ard ic_sig;
		//  ic_sig.pub_check_destructor.publish(ic_sig.desctruct_var);

		  ic_sig.pub_ardrone_land.publish(ic_sig.doit);

		  ros::Duration(3).sleep();
		 // ic_sig.pub_ardrone_reset.publish(ic_sig.doit);

		  ros::shutdown();
}


int main(int argc, char **argv)
{
	 ros::init(argc, argv, "ard_node", ros::init_options::NoSigintHandler);

//	 ros::NodeHandle n;
//	 ros::Subscriber sub = n.subscribe("/tag_detections", 1, aprilCallback);
//	 ros::Subscriber sub2 = n.subscribe("/vanishing_point", 1, vanish_callback);
//	 ros::Subscriber image_sub = n.subscribe("/ardrone/front/image_rect_color", 1, imageCb); // /ardrone/front/image_rect_color
//	// ros::Subscriber sub_vanish = n.subscribe("/vanishing_point", 1, vanish_callback);
//	 //   /RosAria/cmd_vel
//	 ardrone_cmdvel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //ros::Publisher
//	 ardrone_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
//	 ardrone_land = n.advertise<std_msgs::Empty>("/ardrone/land",1);;

	 code_ard ic;

	 signal(SIGINT, mySigIntHandler);
	 ros::spin();
	 //while(ros::ok());
	 ///ic.pub_check_destructor.publish(ic.desctruct_var);

	 return 0;
}
