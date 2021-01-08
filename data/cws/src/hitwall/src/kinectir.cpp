#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

bool KeepGoing = false;

//void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
void scanCallback (const sensor_msgs::LaserScan scan_msg)
{
  size_t num_ranges = scan_msg.ranges.size();
  
  //Show all ranges
  std::cout << "\n Rng\tDist: " << " \n";
  int RangeDivider =(int)((float)num_ranges / (float)16);
  int x;
  for (x = 0; x <= num_ranges; x=x+RangeDivider){
    //double RangeValue = scan_msg->ranges[x];
    double RangeValue = scan_msg.ranges[x];
   	std::cout << " " << x << "\t" << RangeValue << "\n";
   	
   	/*
   	//sensor properties
   	std::cout << "\n\n ang min " << scan_msg.angle_min << "\n ang max " << scan_msg.angle_max << "\n ang inc " << scan_msg.angle_increment << "\n time inc " << scan_msg.time_increment << "\n scan time " << scan_msg.scan_time << "\n range min " << scan_msg.range_min << "\n range max " << scan_msg.range_max;
   	*/
   	
   	/* sensor properties results
		 ang min -0.521568 / ang max 0.524276
		 range min 0.45    / range max 10
	*/
    }
    
    //hitwall using scan
    
    int indexMiddleValue = (int)((float)num_ranges / (float)2);
    double rangeMiddleValue = scan_msg.ranges[indexMiddleValue];
    if(rangeMiddleValue>0 && rangeMiddleValue < 1.00){
    	KeepGoing = false;
    }
    
    std::cout << "\n";
}

/*
void subFunc(sensor_msgs::Image eventMsg)
{
	int dataHeight = eventMsg.height;
	int dataWidth = eventMsg.width;
	int dataStep = eventMsg.step;
	
	// This message contains an uncompressed image
	// (0, 0) is at top-left corner of image

	//uint32 height         # image height, that is, number of rows
	//uint32 width          # image width, that is, number of columns

	//uint8 is_bigendian    # is this data bigendian?
	//uint32 step           # Full row length in bytes
	//uint8[] data          # actual matrix data, size is (step * rows)
	
	int centerInd = floor((dataStep*dataHeight/2.0)-(dataStep/2.0));
	int centerValue = eventMsg.data[centerInd];
	//KeepGoing = false;
	std::cout << " \n h: " << dataHeight << " w: " << dataWidth <<  " s: " << dataStep  << " i: " << centerInd << " v: " << centerValue;
}
*/

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kinectir");
	ros::NodeHandle nodeName;
	ros::Rate loop_rate(10);
	
	ros::Publisher pubName = nodeName.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	
		ros::Subscriber subName = nodeName.subscribe("scan", 100, scanCallback);
		
	while (ros::ok())
  	{
  	
  		if(KeepGoing == true){
	  		geometry_msgs::Twist twistMsg;
			twistMsg.linear.x=0.2;
			twistMsg.linear.y=0.0;
			twistMsg.linear.z=0.0;
			twistMsg.angular.x=0.0;
			twistMsg.angular.y=0.0;
			twistMsg.angular.z=0.0;
	  	
	  		pubName.publish(twistMsg);
	  	} else {
	  		geometry_msgs::Twist twistMsg;
			twistMsg.linear.x=0.0;
			twistMsg.linear.y=0.0;
			twistMsg.linear.z=0.0;
			twistMsg.angular.x=0.0;
			twistMsg.angular.y=0.0;
			twistMsg.angular.z=0.0;
	  	
	  		pubName.publish(twistMsg);
	  	
	  		std::cout << " Really Stopped!!!";
	  	}
	  	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

}
