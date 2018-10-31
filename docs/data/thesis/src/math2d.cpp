/**
 * @author Carsten Könemann
 */

#include <thesis/math2d.h>

using namespace std;

bool insideConvexPolygon(const std::vector<cv::Point2f>& corners,
                         const cv::Point2f& p)
{
  bool result = true;
  for(size_t i = 0; i < corners.size(); i++)
  {
    unsigned int j = i + 1;
    if(j >= corners.size())
    {
      j = 0;
    }
    // det2f() is > 0, if C is left of AB
    if(!(det2f(corners[i], corners[j], p) > 0))
    {
      result = false;
      break;
    }
  }
  return result;
}

bool intersectLines(cv::Point2f a1, cv::Point2f a2,
                    cv::Point2f b1, cv::Point2f b2,
                    cv::Point2f& intersection)
{
  cv::Point2f da = cv::Point(a2.x - a1.x, a2.y - a1.y);
  cv::Point2f db = cv::Point(b2.x - b1.x, b2.y - b1.y);
  float d = cross2f(da, db);
  // Lines are not parallel, compute intersection
  if(d != 0)
  {
    float temp = cross2f(b1 - a1, da) / d;
    intersection = db * temp + b1;
    return true;
  }
  // Lines are parallel, they do not intersect
  else
  {
    return false;
  }
}

bool intersectLineSegments(cv::Point2f a1, cv::Point2f a2,
                           cv::Point2f b1, cv::Point2f b2,
                           cv::Point2f& intersection)
{
  if(intersectLines(a1, a2, b1, b2, intersection))
  {
    if(intersection.x <= max(a1.x, a2.x) && intersection.x >= min(a1.x, a2.x)
    && intersection.y <= max(a1.y, a2.y) && intersection.y >= min(a1.y, a2.y)
    && intersection.x <= max(b1.x, b2.x) && intersection.x >= min(b1.x, b2.x)
    && intersection.y <= max(b1.y, b2.y) && intersection.y >= min(b1.y, b2.y))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}
