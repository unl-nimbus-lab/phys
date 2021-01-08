#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define HOR 70
//static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher laser_pub_;
  //image_geometry::PinholeCameraModel cam_model_;
  //image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera_link/image/depth", 1, 
      &ImageConverter::imageCb, this);
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/kinect2/laser", 20);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat depthImage = cv_ptr->image;
    sensor_msgs::LaserScan ls;

    ros::Time a = ros::Time::now();
    double min_depth, tmp_depth;
    int r_start = 0;//depthImage.rows / 2;
    //int i = 0;
    int deg = 55;
    int max_deg = 70;
    double inc = 0;
    //uint32_t ranges_size = depthImage.cols;
    //ls.ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

    for(int c = 0; c < depthImage.cols; ++c)
      { 
	//inc = inc + depthImage.cols/max_deg;
	double min_depth = 255, tmp_depth;
	for(int r = r_start; r < depthImage.rows; r++)
	  {     
	    tmp_depth = (int)depthImage.at<uchar>(r,c);
	    if( (tmp_depth > (100.*255./4500.)) && (tmp_depth < min_depth) )
	      min_depth = tmp_depth;
	  }

	//double l = (c - depthImage.cols/2) * min_depth *;
	double d =  min_depth*(4500.0/255.0)/100;
	//Kinect v2の視角から(x,y)求める
	double beta_x = ( HOR/2 )*(c - depthImage.cols/2 )/((double)depthImage.cols/2.);
	double l = d / cos(beta_x*M_PI/180.);
	ls.ranges.push_back( l );

	cout<< "w: "<< c - depthImage.cols/2  << " , laser: "<< l 
	    << " , deg "<< beta_x << endl;
      }
    //ros::Time b = ros::Time::now();


    ls.angle_min = -1*35*M_PI/180.;
    ls.angle_max = 35*M_PI/180.;
    ls.angle_increment = (ls.angle_max - ls.angle_min)/(depthImage.cols - 1);
    ls.scan_time = 0;
    ls.range_min = 0.5;
    ls.range_max = 4.5;
    ls.header.stamp = ros::Time::now();
    ls.header.frame_id = "camera_link";


    laser_pub_.publish(ls);
    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
