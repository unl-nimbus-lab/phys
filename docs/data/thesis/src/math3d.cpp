/**
 * @author Carsten Könemann
 */

#include <thesis/math3d.h>

cv::Point3f getYPR(const cv::Point3f forward, const cv::Point3f up)
{
  cv::Point3f ypr;
  // Yaw, rotation about the y-axis
  ypr.x = angle2f(forward.x, forward.z);
  // Pitch, rotation about the x-axis
  ypr.y = angle2f(forward.z, forward.y);
  // Create reference up-vector (y-axis)
  cv::Matx31f yAxis = cv::Matx31f(0.0f, 1.0f, 0.0f);
  // Rotate reference up-vector about x-axis (pitch)
  cv::Matx33f rotX = cv::Matx33f(1.0f,       0.0f,        0.0f,
                                 0.0f, cos(ypr.y), -sin(ypr.y),
                                 0.0f, sin(ypr.y),  cos(ypr.y));
  yAxis = rotX * yAxis;
  // Rotate reference up-vector about y-axis (yaw)
  cv::Matx33f rotY = cv::Matx33f( cos(ypr.x), 0.0f,  sin(ypr.x),
                                        0.0f, 1.0f,        0.0f,
                                 -sin(ypr.x), 0.0f,  cos(ypr.x));
  yAxis = rotY * yAxis;
  // Get rotated reference up-vector as cv::Point3f
  cv::Point3f upReference;
  upReference.x = yAxis(1);
  upReference.y = yAxis(2);
  upReference.z = yAxis(3);
  // Roll, rotation about the z-axis
  ypr.z = angle3f(upReference, up);
  // Done
  return ypr;
}

/*cv::Point3f getYPR(const cv::Point3f forward, const cv::Point3f up)
{
  cv::Point3f ypr;
  // Yaw, rotation about the y-axis
  ypr.x = angle2f(forward.x, forward.z);
  // Pitch, rotation about the x-axis
  ypr.y = angle2f(forward.z, forward.y);
  // Create reference up-vector (y-axis)
  cv::Matx31f yAxis = cv::Matx31f(0.0f, 1.0f, 0.0f);
  // Rotate reference up-vector about x-axis (pitch)
  cv::Matx33f rotX = cv::Matx33f(1.0f,       0.0f,        0.0f,
                                 0.0f, cos(ypr.y), -sin(ypr.y),
                                 0.0f, sin(ypr.y),  cos(ypr.y));
  yAxis = rotX * yAxis;
  // Rotate reference up-vector about y-axis (yaw)
  cv::Matx33f rotY = cv::Matx33f( cos(ypr.x), 0.0f,  sin(ypr.x),
                                        0.0f, 1.0f,        0.0f,
                                 -sin(ypr.x), 0.0f,  cos(ypr.x));
  yAxis = rotY * yAxis;
  // Get rotated reference up-vector as cv::Point3f
  cv::Point3f upReference;
  upReference.x = yAxis(1);
  upReference.y = yAxis(2);
  upReference.z = yAxis(3);
  // Roll, rotation about the z-axis
  ypr.z = angle3f(upReference, up);
  // Done
  return ypr;
}*/
