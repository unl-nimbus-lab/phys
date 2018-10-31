/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>
#include <ros/package.h>

// We are working with OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>

// Local headers
#include <thesis/math2d.h>
#include <thesis/math3d.h>
#include <thesis/timestring.h>
#include <thesis/object_recognizer.h>

// We want to subscribe to multiple topics with one callback
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// We want to subscribe to messages of these types
#include <std_msgs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// We want to call these services
#include <thesis/DatabaseGetAll.h>

// We are going to publish messages of this type
#include <thesis/ObjectClass.h>
#include <thesis/ObjectInstance.h>
#include <thesis/ObjectClassArray.h>
#include <thesis/ObjectInstanceArray.h>
#include <tf/transform_datatypes.h>

// FPS counter for debugging purposes
#include <thesis/fps_calculator.h>

// C++ std libraries
#include <fstream>

using namespace sensor_msgs;
using namespace message_filters;

// Debug image window
#define CAMERA_DEBUG_IMAGE_WINDOW "Camera Debug Image"
#define MIPMAP_DEBUG_IMAGE_WINDOW "Mipmap Debug Image"

// Default image encoding strings
#define BGR8_ENC  sensor_msgs::image_encodings::BGR8
#define MONO8_ENC sensor_msgs::image_encodings::MONO8
#define DEPTH_ENC sensor_msgs::image_encodings::TYPE_32FC1

// Camera orientation seen from the camera POV
static const cv::Point3f YPR_CAMERA = getYPR(cv::Point3f(0.0f, 0.0f, 1.0f),
                                             cv::Point3f(0.0f, 1.0f, 0.0f));
                                             
static const geometry_msgs::Quaternion CAMERA_ORIENTATION_MSG
  = tf::createQuaternionMsgFromRollPitchYaw(YPR_CAMERA.z,
                                            YPR_CAMERA.y,
                                            YPR_CAMERA.x);

// Config parameters
std::string camera_frame;

bool        debug,
            logging;

int         mipmap_level,
            max_objects_per_frame,
            max_nof_keypoints;
            
double      openni_timeout,
            knn_1to2_ratio;

// Logfile
std::ofstream logfile,
              fpsfile;

// We try to get the image size of a connected OpenNI camera (if available)
bool openni_once = false;

// FPS counter for debugging purposes
FPSCalculator fps_calculator;

// Recognized-Objects Publisher
ros::Publisher object_pose_publisher,
               camera_pose_publisher,
               object_meta_publisher;

// Reusable service clients
ros::ServiceClient db_get_all_client;

struct Finding
{
  // The type of object we found
  std::string type_id;
  
  // The pixels that mark the corners of the object in image coordinate space
  std::vector<cv::Point2f> points;
  
  // How sure we are, that we successfully recognized this object
  // (ratio of matching keypoints)
  double confidence;
  
  // Constructor
  Finding(std::string id, std::vector<cv::Point2f> p, double c)
    : type_id(id), points(p), confidence(c)
  {
    //
  };
};

struct Sample
{
  ObjectRecognizer::ImageInfo image_info,
                              mipmap_info;
};

// Map a sample image and its mipmap to the corresponding ID
std::map<std::string, Sample> database_processed;

// Object recognizer
ObjectRecognizer object_recognizer;

// Camera model used to convert pixels to camera coordinates
image_geometry::PinholeCameraModel camera_model;

// Remember when we have successfully recognized an object and where
std::vector<Finding> tracking_objects;

inline void create_mipmap(const cv::Mat& image, cv::Mat& mipmap, unsigned int n)
{
  mipmap = image.clone();
  for(unsigned int i = 0; i < n; i++)
  {
    cv::pyrDown(mipmap, mipmap);
  }
}

bool reset(int mipmaps)
{
  ROS_INFO("Perception::reset(int mipmaps=%i):", mipmaps);
  // Set mipmap level
  if(mipmaps < 0)
  {
    ROS_ERROR("  Mipmap level (%i) out of bounds.", mipmaps);
    std::cout << std::endl;
    return false;
  }
  mipmap_level = mipmaps;
  // Create image database
  ROS_INFO("  Processing database...");
  database_processed.clear();
  thesis::DatabaseGetAll db_get_all_service;
  if(db_get_all_client.call(db_get_all_service))
  {
    for(size_t i = 0; i < db_get_all_service.response.object_classes.size(); i++)
    {
      // Convert ROS images to OpenCV images
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(db_get_all_service.response.object_classes[i].image, MONO8_ENC);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("  cv_bridge exception: %s", e.what());
        std::cout << std::endl;
        return false;
      }
      ROS_INFO("  Loading sample '%s'.", db_get_all_service.response.object_classes[i].type_id.c_str());
      // Process sample image (compute keypoints and descriptors)
      object_recognizer.setMaxKeypoints(max_nof_keypoints);
      ObjectRecognizer::ImageInfo image_info;
      object_recognizer.getImageInfo(cv_ptr->image, image_info);
      database_processed[db_get_all_service.response.object_classes[i].type_id].image_info = image_info;
      // Create mipmap image
      cv::Mat sample_mipmap;
      create_mipmap(cv_ptr->image, sample_mipmap, mipmap_level);
      // Process mipmap image (compute keypoints and descriptors)
      object_recognizer.setMaxKeypoints(max_nof_keypoints / (mipmap_level+1));
      ObjectRecognizer::ImageInfo mipmap_info;
      object_recognizer.getImageInfo(sample_mipmap, mipmap_info);
      database_processed[db_get_all_service.response.object_classes[i].type_id].mipmap_info = mipmap_info;
    }
    ROS_INFO("  ...done.");
    std::cout << std::endl;
    return true;
  }
  else
  {
    ROS_ERROR("  Failed to call service 'thesis_database/get_all'.");
    std::cout << std::endl;
    return false;
  }
}

inline geometry_msgs::Quaternion quaternion_from_plane(cv::Point3f w, cv::Point3f h)
{
  // Get the surface normal of the plane defined by w and h
  // If we can see an object, it is obviously facing the camera (thus the negation)
  cv::Point3f n = surfaceNormal3f(w, h);
  
  // Convert direction vector (surface normal) to Euler angles
  //cv::Point3f ypr = getYPR(n, w);
  
  // Convert Euler angles to quaternion
  //if(ypr.x < 0) ypr.x += M_PI;
  //if(ypr.y < 0) ypr.y += M_PI;
  //if(ypr.z < 0) ypr.z += M_PI;
  
  //tf::Quaternion roll = tf::createQuaternionFromRPY(0, 0, -ypr.z);
  //tf::quaternionTFToMsg(roll, msg);
  
  tf::Vector3 axis_vector(n.x, n.y, n.z);
  
  tf::Vector3 up_vector(0.0, 0.0, 1.0);
  
  tf::Vector3 right_vector = axis_vector.cross(up_vector);
  right_vector.normalized();
  
  tf::Quaternion quat(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
  quat.normalize();
  
  geometry_msgs::Quaternion orientation;
  tf::quaternionTFToMsg(quat, orientation);
  
  return orientation;
}

inline bool draw_rectangle(const std::vector<cv::Point2f>& corners,
                           const cv::Scalar& color,
                           cv::Mat& debug_image)
{
  if(corners.size() == 4)
  {
    line(debug_image, corners[0], corners[1], color);
    line(debug_image, corners[1], corners[2], color);
    line(debug_image, corners[2], corners[3], color);
    line(debug_image, corners[3], corners[0], color);
    return true;
  }
  else
  {
    return false;
  }
}

inline float get_depth(const cv::Mat1f& depth_image, const cv::Point2f& point)
{
  if(point.x < depth_image.cols && point.y < depth_image.rows
     && point.x >= 0 && point.y >= 0)
  {
    float depth = depth_image[(int)point.y][(int)point.x];
    return depth;
  }
  else
  {
    return NAN;
  }
}

inline void publish_object(const Finding& finding,
                           const cv::Mat& mono8_image,
                           const cv::Mat1f& depth_image,
                           thesis::ObjectInstance& msg_object_pose,
                           thesis::ObjectInstance& msg_camera_pose,
                           thesis::ObjectClass& msg_object_meta)
{
  // Get current time once, in order to use it for all messages
  ros::Time current_time = ros::Time::now();
  // Fill camera pose
  msg_camera_pose.type_id                       = finding.type_id;
  msg_camera_pose.confidence                    = finding.confidence;
  msg_camera_pose.pose_stamped.header.stamp     = current_time;
  msg_camera_pose.pose_stamped.header.frame_id  = camera_frame;
  msg_camera_pose.pose_stamped.pose.position.x  = 0;
  msg_camera_pose.pose_stamped.pose.position.y  = 0;
  msg_camera_pose.pose_stamped.pose.position.z  = 0;
  msg_camera_pose.pose_stamped.pose.orientation = CAMERA_ORIENTATION_MSG;
  // Fill object pose
  msg_object_pose.type_id                   = finding.type_id;
  msg_object_pose.confidence                = finding.confidence;
  msg_object_pose.pose_stamped.header.stamp = current_time;
  // Fill object metadata
  msg_object_meta.type_id    = finding.type_id;
  msg_object_meta.confidence = finding.confidence;
  // Get center of the object in camera image coordinate space
  cv::Point2f finding_centroid = centroid2f(finding.points);
  float finding_centroid_depth = get_depth(depth_image, finding_centroid);
  // Get object points in camera coordinate space
  std::vector<cv::Point3f> camera_coordinates;
  for(size_t i = 0; i < finding.points.size(); i++)
  {
    // Vector from centroid to corner
    cv::Point2f center2corner = finding_centroid - finding.points[i];
    // Get pixel halfway between corner and centroid
    cv::Point2f midway = finding_centroid + (0.5f * center2corner);
    // Get depth value for that pixel
    float midway_depth = get_depth(depth_image, midway);
    // Distance from centroid to midway in z-direction
    float corner2midway_depth = finding_centroid_depth - midway_depth;
    // Compute depth for actual corner
    float corner_depth = finding_centroid_depth + (2.0f * corner2midway_depth);
    // Check if we got a valid depth value for this pixel
    if(isnan(corner_depth) || corner_depth == 0)
    {
      camera_coordinates.push_back(cv::Point3f(NAN, NAN, NAN));
    }
    else
    {
      // Project object point to camera coordinate space
      cv::Point3f camera_coordinate = camera_model.projectPixelTo3dRay(finding.points[i]);
      // Use value from depth image to get actual 3D position in camera space
      camera_coordinate *= corner_depth;
      // Add result to our vector holding successfully transformed points
      camera_coordinates.push_back(camera_coordinate);
    }
  }
  // Get object position in camera coordinate space
  if(!isnan(finding_centroid_depth) && finding_centroid_depth != 0)
  {
    // Project object centroid to camera coordinate space
    cv::Point3f camera_coordinate = camera_model.projectPixelTo3dRay(finding_centroid);
    // Use value from depth image to get actual 3D position in camera space
    camera_coordinate *= finding_centroid_depth;
    // Fill object pose message
    msg_object_pose.pose_stamped.header.frame_id = camera_frame;
    msg_object_pose.pose_stamped.pose.position.x = camera_coordinate.x;
    msg_object_pose.pose_stamped.pose.position.y = camera_coordinate.y;
    msg_object_pose.pose_stamped.pose.position.z = camera_coordinate.z;
  }
  else
  {
    // No depth value available.
    // Fill object pose with placeholders instead of throwing it away.
    // We still need to publish it, so others can use a synchronized subscriber.
    msg_object_pose.pose_stamped.header.frame_id = "";
    msg_object_pose.pose_stamped.pose.position.x = NAN;
    msg_object_pose.pose_stamped.pose.position.y = NAN;
    msg_object_pose.pose_stamped.pose.position.z = NAN;
  }
  // Calculate 2 vectors of a triangle on the (planar) object
  cv::Point3f w, h;
  if(!isnan(camera_coordinates[0]) && !isnan(camera_coordinates[1]) && !isnan(camera_coordinates[3]))
  {
    w = camera_coordinates[1] - camera_coordinates[0];
    h = camera_coordinates[3] - camera_coordinates[0];
  }
  else if(!isnan(camera_coordinates[1]) && !isnan(camera_coordinates[2]) && !isnan(camera_coordinates[0]))
  {
    w = camera_coordinates[1] - camera_coordinates[0];
    h = camera_coordinates[2] - camera_coordinates[1];
  }
  else if(!isnan(camera_coordinates[2]) && !isnan(camera_coordinates[3]) && !isnan(camera_coordinates[1]))
  {
    w = camera_coordinates[2] - camera_coordinates[3];
    h = camera_coordinates[2] - camera_coordinates[1];
  }
  else if(!isnan(camera_coordinates[3]) && !isnan(camera_coordinates[0]) && !isnan(camera_coordinates[2]))
  {
    w = camera_coordinates[2] - camera_coordinates[3];
    h = camera_coordinates[3] - camera_coordinates[0];
  }
  else
  {
    w = cv::Point3f(NAN, NAN, NAN);
    h = cv::Point3f(NAN, NAN, NAN);
  }
  // Get object orientation & dimensions in camera coordinate space
  if(!isnan(w) && !isnan(h))
  {
    ROS_DEBUG("Perception: !isnan(w) && !isnan(h) == true");
    // Compute object orientation
    msg_object_pose.pose_stamped.pose.orientation = quaternion_from_plane(w, h);
    // Compute object dimensions
    msg_object_meta.width  = mag3f(w);
    msg_object_meta.height = mag3f(h);
  }
  else
  {
    ROS_DEBUG("Perception: !isnan(w) && !isnan(h) == false");
    // Fill object pose message
    msg_object_pose.pose_stamped.pose.orientation.x = NAN;
    msg_object_pose.pose_stamped.pose.orientation.y = NAN;
    msg_object_pose.pose_stamped.pose.orientation.z = NAN;
    msg_object_pose.pose_stamped.pose.orientation.w = NAN;
    // Fill object metadata message
    msg_object_meta.width  = NAN;
    msg_object_meta.height = NAN;
  }
}

inline void recognize(const std::string& sample_id,
                      ObjectRecognizer::ImageInfo& sample_image_info,
                      const cv::Mat& camera_image,
                      ObjectRecognizer::ImageInfo& camera_image_info,
                      const unsigned int max_loops,
                      std::vector<Finding>& findings,
                      cv::Mat& debug_image)
{
  std::vector<cv::Point2f> object_points;
  double confidence = 0.0;
  // Search for an object until we don't find any more occurrences
  for(unsigned int loops = 0; loops <= max_loops; loops++)
  {
    if(object_recognizer.recognize(sample_image_info,
                                   camera_image_info,
                                   object_points,
                                   confidence,
                                   knn_1to2_ratio))
    {
      // Remove keypoints belonging to this candidate from image info,
      // in order to search for other candidates of the same type
      ObjectRecognizer::ImageInfo inside_mask;
      object_recognizer.filterImageInfo(camera_image_info, object_points, &inside_mask, &camera_image_info);
      // 
      if(inside_mask.keypoints.size() >= 4)
      {
        // Visualize result
        draw_rectangle(object_points, GREEN, debug_image);
        // We want to search for this candidate in the original camera image for higher precision
        findings.push_back(Finding(sample_id, object_points, confidence));
        cv::drawKeypoints(debug_image, inside_mask.keypoints, debug_image, YELLOW);
        // Start next search with an empty vector again
        object_points.clear();
      }
      else
      {
        // Draw result one last time in order to visualize false positives
        draw_rectangle(object_points, RED, debug_image);
        // We won't find any more occurrences
        break;
      }
    }
    else
    {
      // Draw result one last time in order to visualize false positives
      draw_rectangle(object_points, RED, debug_image);
      // We won't find any more occurrences
      break;
    }
  }
}

void callback_simple(const cv::Mat& camera_image,
                     cv::Mat& camera_debug_image,
                     std::vector<Finding>& findings)
{
  // Process camera image
  object_recognizer.setMaxKeypoints(max_nof_keypoints);
  ObjectRecognizer::ImageInfo cam_img_info;
  object_recognizer.getImageInfo(camera_image, cam_img_info);
  // Draw keypoints to debug image
  cv::drawKeypoints(camera_debug_image, cam_img_info.keypoints, camera_debug_image, YELLOW);
  // Recognize objects
  std::map<std::string, Sample>::iterator db_iter = database_processed.begin();
  for(; db_iter != database_processed.end(); db_iter++)
  {
    recognize(db_iter->first,
              db_iter->second.image_info,
              camera_image,
              cam_img_info,
              max_objects_per_frame,
              findings,
              camera_debug_image);
  }
  // Show debug image
  if(debug)
  {
    cv::imshow(CAMERA_DEBUG_IMAGE_WINDOW, camera_debug_image);
    cv::waitKey(3);
  }
}

void callback_mipmapping(const cv::Mat& camera_image,
                         cv::Mat& camera_debug_image,
                         std::vector<Finding>& findings)
{
  // Create mipmap of camera image
  cv::Mat cam_img_mipmap;
  create_mipmap(camera_image, cam_img_mipmap, mipmap_level);
  // Create copy to draw stuff on (for debugging purposes)
  cv::Mat mipmap_debug_image;
  create_mipmap(camera_debug_image, mipmap_debug_image, mipmap_level);
  // Process the mipmap of the camera image
  object_recognizer.setMaxKeypoints(max_nof_keypoints / (mipmap_level+1));
  ObjectRecognizer::ImageInfo cam_img_mipmap_info;
  object_recognizer.getImageInfo(cam_img_mipmap, cam_img_mipmap_info);
  // Draw keypoints to debug image
  cv::drawKeypoints(mipmap_debug_image, cam_img_mipmap_info.keypoints, mipmap_debug_image, BLUE);
  // Recognize sample mipmaps on camera mipmap
  std::vector<Finding> mipmap_findings;
  std::map<std::string, Sample>::iterator db_iter = database_processed.begin();
  for(; db_iter != database_processed.end(); db_iter++)
  {
    recognize(db_iter->first,
              db_iter->second.mipmap_info,
              camera_image,
              cam_img_mipmap_info,
              max_objects_per_frame,
              mipmap_findings,
              mipmap_debug_image);
  }
  // We did not find anything on the mipmap camera image,
  // and we didn't find anything last time.
  // Don't look any further.
  if(mipmap_findings.size() < 1 && tracking_objects.size() < 1)
  {
    // Show debug image
    if(debug)
    {
      cv::imshow(CAMERA_DEBUG_IMAGE_WINDOW, camera_debug_image);
      cv::imshow(MIPMAP_DEBUG_IMAGE_WINDOW, mipmap_debug_image);
      cv::waitKey(3);
    }
    //
    return;
  }
  // Create a vector holding
  // - all objects found in mipmap camera image
  //   (they are added first, making perception more stable during movement)
  // - all objects successfully found last frame
  // They are the candidates we want to search in the original camera image
  std::vector<Finding> temp_tracking;
  double scale = pow(2, mipmap_level);
  for(size_t i = 0; i < mipmap_findings.size(); i++)
  {
    // Scale points found on the mipmap image back to original size
    mipmap_findings[i].points[0] *= scale,
    mipmap_findings[i].points[1] *= scale,
    mipmap_findings[i].points[2] *= scale,
    mipmap_findings[i].points[3] *= scale;
    temp_tracking.push_back(mipmap_findings[i]);
  }
  temp_tracking.insert(temp_tracking.end(), tracking_objects.begin(), tracking_objects.end());
  // Clear tracking vector,
  // so we can fill it with objects recognized this frame again
  tracking_objects.clear();
  // Compute keypoints and descriptors for the original camera image
  object_recognizer.setMaxKeypoints(max_nof_keypoints);
  ObjectRecognizer::ImageInfo camera_image_info;
  object_recognizer.getImageInfo(camera_image, camera_image_info);
  cv::drawKeypoints(camera_debug_image, camera_image_info.keypoints, camera_debug_image, BLUE);
  // Recognize objects
  for(std::vector<Finding>::iterator it = temp_tracking.begin(); it != temp_tracking.end(); it++)
  {
    std::vector<cv::Point2f> object_points;
    double confidence;
    // Draw the region we are going to examine further
    draw_rectangle(it->points, YELLOW, camera_debug_image);
    // Process only the part of the camera image
    // belonging to the area defined by the points we found on the mipmap image
    ObjectRecognizer::ImageInfo inside_mask,
                                outside_mask;
    object_recognizer.filterImageInfo(camera_image_info,
                                      it->points,
                                      &inside_mask,
                                      &outside_mask);
    // Draw the keypoints belonging to the region we are going to examine
    cv::drawKeypoints(camera_debug_image, inside_mask.keypoints, camera_debug_image, YELLOW);
    // Apply recognizer to the previously processed region of the camera image
    if(object_recognizer.recognize(database_processed[it->type_id].image_info,
                                   inside_mask,
                                   object_points,
                                   confidence,
                                   knn_1to2_ratio))
    {
      Finding finding(it->type_id, object_points, confidence);
      // We are going to publish this finding later
      findings.push_back(finding);
      // Also remember finding as a candidate for next frame
      tracking_objects.push_back(finding);
      // Visualize result
      draw_rectangle(object_points, GREEN, camera_debug_image);
      // Do not check these keypoints / descriptors anymore,
      // they belong to an already successfully recognized object
      camera_image_info.keypoints   = outside_mask.keypoints;
      camera_image_info.descriptors = outside_mask.descriptors;
    }
    else
    {
      // Visualize result for unsuccessful passes in a different color
      draw_rectangle(object_points, RED, camera_debug_image);
    }
  }
  // Show debug image
  if(debug)
  {
    cv::imshow(CAMERA_DEBUG_IMAGE_WINDOW, camera_debug_image);
    cv::imshow(MIPMAP_DEBUG_IMAGE_WINDOW, mipmap_debug_image);
    cv::waitKey(3);
  }
}

void callback_openni(const sensor_msgs::Image::ConstPtr& rgb_input,
                     const sensor_msgs::Image::ConstPtr& depth_input,
                     const sensor_msgs::CameraInfo::ConstPtr& cam_info_input)
{
  // Update FPS calculator
  fps_calculator.update();
  
  // Convert ROS image messages to OpenCV images
  cv_bridge::CvImagePtr cv_ptr_bgr8,
                        cv_ptr_mono8,
                        cv_ptr_depth;
  try
  {
    cv_ptr_bgr8  = cv_bridge::toCvCopy(rgb_input,   BGR8_ENC);
    cv_ptr_mono8 = cv_bridge::toCvCopy(rgb_input,   MONO8_ENC);
    cv_ptr_depth = cv_bridge::toCvCopy(depth_input, DEPTH_ENC);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Perception::callback_openni(): ");
    ROS_ERROR("  cv_bridge exception: %s", e.what());
    std::cout << std::endl;
    return;
  }
  // Depth values are stored as 32bit float
  cv::Mat1f depth_image = cv::Mat1f(cv_ptr_depth->image);
  // Print FPS to debug image
  if(debug)
  {
    // Log FPS
    if(logging && fpsfile.is_open())
    {
      fpsfile << fps_calculator.get_fps() << "\n";
    }
    // Create string
    std::stringstream stream;
    stream << "FPS: " << fps_calculator.get_fps();
    std::string fps_string = stream.str();
    // Setup font
    double   font_scale     = 2;
    int      font_face      = cv::FONT_HERSHEY_PLAIN,
             font_thickness = 2,
             baseline       = 0;
    cv::Size text_size      = cv::getTextSize(fps_string, font_face, font_scale, font_thickness, &baseline);
    baseline += font_thickness;
    // Text position
    cv::Point text_org(10, text_size.height + 10);
    cv::Point text_shadow_org(text_org.x + 1, text_org.y + 1);
    // Draw text
    cv::putText(cv_ptr_bgr8->image, fps_string, text_shadow_org, font_face, font_scale, BLACK, font_thickness, 8);
    cv::putText(cv_ptr_bgr8->image, fps_string, text_org,        font_face, font_scale, WHITE, font_thickness, 8);
  }
  //
  std::vector<Finding> findings;
  if(mipmap_level > 0)
  {
    callback_mipmapping(cv_ptr_mono8->image, cv_ptr_bgr8->image, findings);
  }
  else
  {
    callback_simple(cv_ptr_mono8->image, cv_ptr_bgr8->image, findings);
  }
  // These messages will be published
  thesis::ObjectInstanceArray msgs_object_pose,
                              msgs_camera_pose;
  thesis::ObjectClassArray    msgs_object_meta;
  // Update current camera model in order to project 2D pixel to 3D ray
  camera_model.fromCameraInfo(cam_info_input);
  // Publish objects
  for(size_t i = 0; i < findings.size(); i++)
  {
    // Create empty messages
    thesis::ObjectInstance msg_object_pose,
                           msg_camera_pose;
    thesis::ObjectClass    msg_object_meta;
    // Get values for object messages from findings
    publish_object(
      findings[i],
      cv_ptr_mono8->image,
      depth_image,
      msg_object_pose,
      msg_camera_pose,
      msg_object_meta
    );
    //
    msgs_object_pose.array.push_back(msg_object_pose);
    msgs_camera_pose.array.push_back(msg_camera_pose);
    msgs_object_meta.array.push_back(msg_object_meta);
  }
  // Publish messages
  object_pose_publisher.publish(msgs_object_pose);
  camera_pose_publisher.publish(msgs_camera_pose);
  object_meta_publisher.publish(msgs_object_meta);
  // Log findings
  if(logging && logfile.is_open())
  {
    std::map<std::string, Sample>::iterator db_iter = database_processed.begin();
    for(; db_iter != database_processed.end(); db_iter++)
    {
      bool type_found = false;
      for(size_t i = 0; i < findings.size(); i++)
      {
        if(findings[i].type_id.compare(db_iter->first) == 0)
        {
          type_found = true;
        }
      }
      if(type_found)
      {
        logfile << "1 ";
      }
      else
      {
        logfile << "0 ";
      }
    }
    logfile << "\n";
  }
}

void callback_database_update(const std_msgs::Empty::ConstPtr& input)
{
  // Rebuild the database
  reset(mipmap_level);
}

void callback_openni_once(const sensor_msgs::CameraInfo::ConstPtr& input)
{
  object_recognizer.setMaxImageSize(std::max(input->width, input->height));
  openni_once = true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_recognition");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  // Get global parameters
  std::string rgb_image_topic,
              depth_image_topic,
              camera_info_topic;
  
  nh.getParam("/thesis/rgb_image_topic",   rgb_image_topic);
  nh.getParam("/thesis/depth_image_topic", depth_image_topic);
  nh.getParam("/thesis/camera_info_topic", camera_info_topic);
  nh.getParam("/thesis/camera_frame",      camera_frame);
  nh.getParam("/thesis/openni_timeout",    openni_timeout);
  
  ROS_INFO("Perception (global parameters): ");
  ROS_INFO("  RGB image topic:   %s.", rgb_image_topic.c_str());
  ROS_INFO("  Depth image topic: %s.", depth_image_topic.c_str());
  ROS_INFO("  Camera info topic: %s.", camera_info_topic.c_str());
  ROS_INFO("  Camera frame:      %s.", camera_frame.c_str());
  ROS_INFO("  OpenNI timeout:    %f.", openni_timeout);
  std::cout << std::endl;
  
  // Get local parameters
  nh_private.param("debug",                 debug,                 false);
  nh_private.param("logging",               logging,               false);
  nh_private.param("mipmap_level",          mipmap_level,          0);
  nh_private.param("max_objects_per_frame", max_objects_per_frame, 1);
  nh_private.param("max_nof_keypoints",     max_nof_keypoints,     64);
  nh_private.param("knn_1to2_ratio",        knn_1to2_ratio,        0.9);
  
  ROS_INFO("Perception (local parameters): ");
  ROS_INFO("  Debug mode:                 %s.", debug   ? "true" : "false");
  ROS_INFO("  Logging:                    %s.", logging ? "true" : "false");
  ROS_INFO("  Mipmap level:               %i.", mipmap_level);
  ROS_INFO("  Max objects per frame:      %i.", max_objects_per_frame);
  ROS_INFO("  Max number of keypoints:    %i.", max_nof_keypoints);
  ROS_INFO("  kNN 1st to 2nd ratio:       %f.", knn_1to2_ratio);
  std::cout << std::endl;
  
  // Create debug image windows
  if(debug)
  {
    cv::namedWindow(CAMERA_DEBUG_IMAGE_WINDOW);
    if(mipmap_level > 0)
    {
      cv::namedWindow(MIPMAP_DEBUG_IMAGE_WINDOW);
    }
  }
  
  // Try to get OpenNI camera image size
  ros::Subscriber camera_info_once_subscriber = nh.subscribe(camera_info_topic, 1, callback_openni_once);
  ros::Time wait_time = ros::Time::now();
  while(!openni_once && ros::ok())
  {
    // Spin
    ros::spinOnce();
    // Stop if wait time is up
    if(ros::Time::now().toSec() - wait_time.toSec() > openni_timeout)
    {
      break;
    }
  }
  camera_info_once_subscriber.shutdown();
  
  // Initialize reusable service clients
  ros::service::waitForService("thesis_database/get_all", -1);
  db_get_all_client = nh.serviceClient<thesis::DatabaseGetAll>("thesis_database/get_all");
  
  // Create image database
  reset(mipmap_level);
  
  // Open logfiles
  if(logging)
  {
    // Create logfile names
    std::stringstream logpath, fpspath;
    // Path to this package
    logpath << ros::package::getPath("thesis");
    logpath << "/log/perceptions_";
    fpspath << ros::package::getPath("thesis");
    fpspath << "/log/fps_";
    // Current time and date
    std::string current_time = std::string(timestring::get_timestring());
    logpath << current_time;
    fpspath << current_time;
    // File extension
    logpath << ".log";
    fpspath << ".log";
    // Open perceptions logfile
    ROS_INFO("Perception: ");
    logfile.open(logpath.str().c_str(), std::ofstream::out | std::ofstream::app);
    if(logfile.is_open())
    {
      // Create header for logfile
      logfile << "# Has an object of type XY been seen during a frame? \n";
      logfile << "#   0 if not \n";
      logfile << "#   1 if yes \n";
      logfile << "# These objects exist in the database: \n";
      logfile << "# ";
      std::map<std::string, Sample>::iterator db_iter = database_processed.begin();
      for(; db_iter != database_processed.end(); db_iter++)
      {
        logfile << db_iter->first << " ";
      }
      logfile << "\n";
      // Debug output
      ROS_INFO("  Successfully opened logfile '%s'.", logpath.str().c_str());
    }
    else
    {
      ROS_INFO("  Unable to open logfile '%s'.", logpath.str().c_str());
    }
    // Open fps logfile
    fpsfile.open(fpspath.str().c_str(), std::ofstream::out | std::ofstream::app);
    if(fpsfile.is_open())
    {
      ROS_INFO("  Successfully opened logfile '%s'.", fpspath.str().c_str());
    }
    else
    {
      ROS_INFO("  Unable to open logfile '%s'.", fpspath.str().c_str());
    }
    std::cout << std::endl;
  }

  // Subscribe to relevant OpenNI topics
  Subscriber<Image>      rgb_subscriber(nh, rgb_image_topic, 1);
  Subscriber<Image>      depth_subscriber(nh, depth_image_topic, 1);
  Subscriber<CameraInfo> cam_info_subscriber(nh, camera_info_topic, 1);

  // Use one time-sychronized callback for all OpenNI subscriptions
  typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> synchronizer(
    SyncPolicy(10),
    rgb_subscriber,
    depth_subscriber,
    cam_info_subscriber
  );
  synchronizer.registerCallback(boost::bind(&callback_openni, _1, _2, _3));
  
  // Subscribe to updates about the database
  ros::Subscriber database_update_subscriber = nh.subscribe(
    "thesis_database/updates",
    1,
    callback_database_update
  );
  
  // Publish recognized objects
  object_pose_publisher = nh_private.advertise<thesis::ObjectInstanceArray>("object_pose", 1000);
  camera_pose_publisher = nh_private.advertise<thesis::ObjectInstanceArray>("camera_pose", 1000);
  object_meta_publisher = nh_private.advertise<thesis::ObjectClassArray>("object_dimension", 1000);
  
  // Spin
  ros::spin();
  
  // Free memory
  if(debug)
  {
    cv::destroyWindow(CAMERA_DEBUG_IMAGE_WINDOW);
    if(mipmap_level > 0)
    {
      cv::destroyWindow(MIPMAP_DEBUG_IMAGE_WINDOW);
    }
  }
  
  // Close logfiles
  if(logging && logfile.is_open())
  {
    logfile.flush();
    logfile.close();
  }
  if(logging && fpsfile.is_open())
  {
    fpsfile.flush();
    fpsfile.close();
  }
  
  // Exit with success
  return 0;
}
