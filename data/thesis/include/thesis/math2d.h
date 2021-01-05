/**
 * @author Carsten Könemann
 */
#ifndef __MATH2D__
#define __MATH2D__

#include <thesis/math.h>

#include <opencv2/opencv.hpp>

/**
 * @return The determinant.
 */
inline float det2f(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c)
{
  return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

inline float dot2f(cv::Point2f a, cv::Point2f b)
{
  return (a.x * b.x) + (a.y * b.y);
}

inline float cross2f(cv::Point2f a, cv::Point2f b)
{
  return (a.x * b.y) - (b.x * a.y);
}

inline float mag2f(cv::Point2f p)
{
  return sqrt(p.x*p.x + p.y*p.y);
}

inline float dist2f(cv::Point2f a, cv::Point2f b)
{
  return mag2f(b - a);
}

inline float angle2f(float x, float y)
{
  return atan(y / x);
}

inline float angle2f(const cv::Point2f& dir)
{
  return atan(dir.y / dir.x);
}

inline float angle2f(cv::Point2f dir1, cv::Point2f dir2)
{
  return acos(dot2f(dir1, dir2) / (mag2f(dir1) * mag2f(dir2)));
}

inline cv::Point2f centroid2f(const std::vector<cv::Point2f>& v)
{
  cv::Point2f c;
  for(size_t i = 0; i < v.size(); i++)
  {
    c.x += v[i].x;
    c.y += v[i].y;
  }
  c.x /= v.size();
  c.y /= v.size();
  return c;
}

bool insideConvexPolygon(const std::vector<cv::Point2f>& corners,
                         const cv::Point2f& p);

bool intersectLines(cv::Point2f a1, cv::Point2f a2,
                    cv::Point2f b1, cv::Point2f b2,
                    cv::Point2f& intersection);

bool intersectLineSegments(cv::Point2f a1, cv::Point2f a2,
                           cv::Point2f b1, cv::Point2f b2,
                           cv::Point2f& intersection);

#endif //__MATH2D__
