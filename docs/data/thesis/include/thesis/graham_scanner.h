/**
 * @author Carsten Könemann
 */
#ifndef __GRAHAM_SCANNER__
#define __GRAHAM_SCANNER__

#include <opencv2/core/core.hpp>

class GrahamScanner
{
  public:
    static cv::Point2f p0;
  
    static void grahamScan(std::vector<cv::Point2f>& input,
                           std::vector<cv::Point2f>& output);
};

#endif //__GRAHAM_SCANNER__
