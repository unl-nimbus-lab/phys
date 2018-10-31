/*
 * LaserScanLinearRegressionUtil.h
 *
 *  Created on: Jun 12, 2012
 *      Author: matthias
 */

#ifndef LASERSCANLINEARREGRESSIONUTIL_H_
#define LASERSCANLINEARREGRESSIONUTIL_H_

#include <vector>

#include <sensor_msgs/LaserScan.h>
#include "placement_wrt_workspace/LaserScanLinearRegression.h"

class LaserScanLinearRegressionUtil {
public:
	LaserScanLinearRegressionUtil();
	virtual ~LaserScanLinearRegressionUtil();

	std::vector<LaserScanLinearRegression::ScanItem> convert(sensor_msgs::LaserScanConstPtr scan);

};

#endif /* LASERSCANLINEARREGRESSIONUTIL_H_ */
