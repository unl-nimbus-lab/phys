/**
 * @author Carsten Könemann
 */

#include <thesis/graham_scanner.h>

#include <thesis/math2d.h>

cv::Point2f GrahamScanner::p0;

/**
 * @return Angle between a->b and the x-axis in degrees (0 - 180).
 */
inline float angle(const cv::Point2f& a, const cv::Point2f& b)
{
  if(b.x == a.x)
  {
    return 90.0f;
  }
  float m = (b.y - a.y) / (b.x - a.x);
  if(m < 0)
  {
    return 180.0f + atan(m) * RAD;
  }
  return atan(m) * RAD;
}

struct grahamScanComparator
{
  bool operator() (const cv::Point2f& a, const cv::Point2f& b)
  {
    return angle(GrahamScanner::p0, a) < angle(GrahamScanner::p0, b);
  }
};

void GrahamScanner::grahamScan(std::vector<cv::Point2f>& input, std::vector<cv::Point2f>& output)
{
  // Determine point with the lowest y coordinate.
  // If there are multiple points with the same y value,
  // use the one with the lowest x value instead.
  // Create a vector containing all non-p0 points in the process.
  std::vector<cv::Point2f> nonP0;
  p0 = input.front();
  for(size_t i = 1; i < input.size(); i++)
  {
    if(input[i].y < p0.y || (input[i].y == p0.y && input[i].x < p0.x))
    {
      nonP0.push_back(p0);
      p0 = input[i];
    }
    else
    {
      nonP0.push_back(input[i]);
    }
  }
  // Sort points by angle they and p0 make with the x-axis
  sort(nonP0.begin(), nonP0.end(), grahamScanComparator());
  // Compute convex hull of the input set of points
  output.push_back(p0);
  output.push_back(nonP0[0]);
  for(size_t i = 1; i < nonP0.size() && output.size() > 1; )
  {
    // det2f() is
    // < 0, if C is right of AB
    // = 0, if C is on AB
    // > 0, if C is left of AB
    if(det2f(output[output.size()-2], output.back(), nonP0[i]) > 0)
    {
      output.push_back(nonP0[i]);
      i++;
    }
    else
    {
      output.pop_back();
    }
  }
}
