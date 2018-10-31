
#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <raw_base_placement/BaseScanLinearRegression.h>
#include "placement_wrt_workspace/LaserScanLinearRegression.h"
#include "placement_wrt_workspace/LaserScanLinearRegressionUtil.h"


using namespace raw_base_placement;

class LaserScanLinearRegressionService {
protected:

	ros::NodeHandle nh;

	std::string basescan_topic;
	ros::Duration time_to_wait;

	LaserScanLinearRegression::ScanItemFilter scanfilter;
	LaserScanLinearRegression::RegressionAnalysis regAnalysis;
	LaserScanLinearRegressionUtil util;
public:

	LaserScanLinearRegressionService(ros::NodeHandle nh, std::string basescan_topic) {
		this->nh = nh;
		this->basescan_topic = basescan_topic;
		time_to_wait = ros::Duration(2.0);
	}



	bool baseScanLinearRegression(BaseScanLinearRegressionRequest& req,
									BaseScanLinearRegressionResponse& res) {

		if (fabs(req.filter_maxAngle) <= 0.0001 && fabs(req.filter_minAngle) <= 0.0001) {
			req.filter_minAngle = -M_PI_4;
			req.filter_maxAngle = M_PI_4;
		}

		if (fabs(req.filter_maxDistance) <= 0.0001 && fabs(req.filter_minDistance) <= 0.0001) {
			req.filter_minDistance = 0.02;
			req.filter_maxDistance = 0.8;
		}


		ROS_INFO("Wait for base_scan message");

		sensor_msgs::LaserScanConstPtr scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(basescan_topic, nh, time_to_wait);

		if (scan == 0) {
			ROS_ERROR("No scan received within timeframe on topic %s", basescan_topic.c_str());
			return false;
		}

		//ROS_INFO("Converting");

		std::vector<LaserScanLinearRegression::ScanItem> data = util.convert(scan);

		//ROS_INFO("Filtering");

		std::vector<LaserScanLinearRegression::ScanItem> filtered_data = scanfilter.filterByDistance(data, req.filter_minDistance, req.filter_maxDistance);
		filtered_data = scanfilter.filterByAngle(filtered_data, req.filter_minAngle, req.filter_maxAngle);

		double center, a, b;

		ROS_INFO("Calculating Coeffs");

		regAnalysis.calculateCoefficient(filtered_data, center, a, b);
		
		ROS_INFO("a: %f, b: %f, c: %f", a, b, center);

		res.center = center;
		res.a = a;
		res.b = b;

		return true;
	}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "raw_base_placement_wrt_workspace_ss");
  ros::NodeHandle n;

  std::string topic = "/scan_front";

  LaserScanLinearRegressionService lslrs(n, topic);

  ros::ServiceServer service = n.advertiseService("scan_front_linearregression",
  		  &LaserScanLinearRegressionService::baseScanLinearRegression, &lslrs);

  ROS_INFO("LaserScanLinearRegressionService is ready");

  

  ros::Rate loop_rate(15);
  while (ros::ok()) {    
     ros::spinOnce();
     loop_rate.sleep();
  }
  return 0;
}
