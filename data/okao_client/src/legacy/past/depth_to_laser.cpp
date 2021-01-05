#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher laser_pub_;
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
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
    int r_start = depthImage.rows / 2;
    //int i = 0;
    for(int c = 0; c < depthImage.cols; ++c)
      { 
	min_depth = 255;
	for(int r = r_start; r < depthImage.rows; r++)
	  {     
	    tmp_depth = (int)depthImage.at<uchar>(r,c);
	    if( (tmp_depth > (50*255/4500)) && (tmp_depth < min_depth) )
	      min_depth = tmp_depth;
	  }
	double l = depthImage.cols/2 - c;
	double d =  min_depth*(4500.0/255.0)/100;
	ls.ranges.push_back( sqrt(l*l + d*d) );

	cout<< "(l , d) = "<< l 
	    << " , "<< d <<  " ) --->  "<< sqrt(l*l + d*d) <<endl;
      }
    //ros::Time b = ros::Time::now();


    ls.angle_min = -3.14;
    ls.angle_max = 3.14;
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
