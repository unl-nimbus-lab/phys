/**
 * Define cv::Scalar()s for common colors.
 *
 * @author Carsten Könemann
 */
#ifndef __COLOR__
#define __COLOR__

#include <opencv2/opencv.hpp>

static const cv::Scalar RED    = cv::Scalar(  0,   0, 255);
static const cv::Scalar GREEN  = cv::Scalar(  0, 255,   0);
static const cv::Scalar BLUE   = cv::Scalar(255,   0,   0);

static const cv::Scalar YELLOW = cv::Scalar(  0, 255, 255);

static const cv::Scalar WHITE  = cv::Scalar(255, 255, 255);
static const cv::Scalar BLACK  = cv::Scalar(  0,   0,   0);

#endif //__COLOR__
