/**
 * @author Carsten Könemann
 */

#include <thesis/object_recognizer.h>

#include <thesis/graham_scanner.h>
#include <thesis/math2d.h>

ObjectRecognizer::ObjectRecognizer()
{
  maxImageSize = 1280;
  maxKeypoints = 128;
}

ObjectRecognizer::~ObjectRecognizer()
{
  // Default destructor
}

void ObjectRecognizer::setMaxImageSize(const int maxImageSize)
{
  if(this->maxImageSize < maxImageSize)
  {
    #ifdef  USE_SIFT_GPU
      SiftGPUFeatureDetector* detector = SiftGPUFeatureDetector::getInstance();
      //
      if(detector->initialized())
      {
        detector->setMaxImageSize(maxImageSize);
      }
    #endif
    #ifndef USE_SIFT_GPU
      // no need to set max image size in OpenCV software mode
    #endif
    this->maxImageSize = maxImageSize;
  }
}

void ObjectRecognizer::setMaxKeypoints(const int maxKeypoints)
{
  if(this->maxKeypoints != maxKeypoints)
  {
    #ifdef  USE_SIFT_GPU
      SiftGPUFeatureDetector* detector = SiftGPUFeatureDetector::getInstance();
      //
      if(detector->initialized())
      {
        detector->setMaxKeypoints(maxKeypoints);
      }
    #endif
    #ifndef USE_SIFT_GPU
      feature_detector = cv::SiftFeatureDetector(maxKeypoints);
    #endif
    this->maxKeypoints = maxKeypoints;
  }
}

void ObjectRecognizer::filterImageInfo(const ImageInfo& input,
                                       const std::vector<cv::Point2f>& mask,
                                       ImageInfo* inside_mask,
                                       ImageInfo* outside_mask)
{
  ImageInfo temp_inside_mask,
            temp_outside_mask;
  // Filter input
  for(size_t i = 0; i < input.keypoints.size(); i++)
  {
    if(insideConvexPolygon(mask, input.keypoints[i].pt))
    {
      if(inside_mask)
      {
        temp_inside_mask.keypoints.push_back(input.keypoints[i]);
        #ifdef  USE_SIFT_GPU
          temp_inside_mask.descriptors.insert(temp_inside_mask.descriptors.end(),
                                              input.descriptors.begin()+i*128,
                                              input.descriptors.begin()+i*128+128);
        #endif
        #ifndef USE_SIFT_GPU
          temp_inside_mask.descriptors.push_back(input.descriptors.row(i));
        #endif
      }
    }
    else
    {
      if(outside_mask)
      {
        temp_outside_mask.keypoints.push_back(input.keypoints[i]);
        #ifdef  USE_SIFT_GPU
          temp_outside_mask.descriptors.insert(temp_outside_mask.descriptors.end(),
                                               input.descriptors.begin()+i*128,
                                               input.descriptors.begin()+i*128+128);
        #endif
        #ifndef USE_SIFT_GPU
          temp_outside_mask.descriptors.push_back(input.descriptors.row(i));
        #endif
      }
    }
  }
  // Output
  if(inside_mask)
  {
    inside_mask->width       = input.width;
    inside_mask->height      = input.height;
    inside_mask->keypoints   = temp_inside_mask.keypoints;
    inside_mask->descriptors = temp_inside_mask.descriptors;
  }
  if(outside_mask)
  {
    outside_mask->width       = input.width;
    outside_mask->height      = input.height;
    outside_mask->keypoints   = temp_outside_mask.keypoints;
    outside_mask->descriptors = temp_outside_mask.descriptors;
  }
}

void ObjectRecognizer::getImageInfo(const cv::Mat& image,
                                    ImageInfo& image_info)
{
  // Remember image size
  image_info.width  = image.cols;
  image_info.height = image.rows;
  // Detect keypoints & descriptors
  #ifdef  USE_SIFT_GPU
    SiftGPUFeatureDetector* detector = SiftGPUFeatureDetector::getInstance();
    // Initialize feature detector (if not initialized yet)
    if(!detector->initialized())
    {
      detector->init(maxKeypoints, maxImageSize);
    }
    // Detect keypoints & compute descriptors
    detector->detect(image, image_info.keypoints, image_info.descriptors);
  #endif
  #ifndef USE_SIFT_GPU
    // Detect keypoints
    feature_detector.detect(image, image_info.keypoints);
    // Compute descriptors
    if(!image_info.keypoints.empty())
    {
      descriptor_extractor.compute(image, image_info.keypoints, image_info.descriptors);
    }
  #endif
}

bool ObjectRecognizer::recognize(ImageInfo& sample_info,
                                 ImageInfo& cam_img_info,
                                 std::vector<cv::Point2f>& object_points,
                                 double& confidence,
                                 const double knn_1to2_ratio)
{
  // Otherwise an OpenCV assertion would fail for images without keypoints
  #ifdef  USE_SIFT_GPU
    if(sample_info.descriptors.size() < 2 || cam_img_info.descriptors.size() < 2)
  #endif
  #ifndef USE_SIFT_GPU
    if(sample_info.descriptors.rows   < 2 || cam_img_info.descriptors.rows   < 2)
  #endif
  {
    return false;
  }
  // Compute matches
  std::vector<cv::DMatch> matches_filtered;
  #ifdef  USE_SIFT_GPU
    SiftGPUDescriptorMatcher* matcher = SiftGPUDescriptorMatcher::getInstance();
    // Initialize matcher (if not initialized yet)
    if(!matcher->initialized())
    {
      matcher->init();
    }
    // Match descriptors
    // SiftGPU already implements filtering
    // - by max distance between two matches
    // - by ratio of nearest and second nearest neighbour distance
    // - by checking if two descriptors are a mutual best match
    matcher->match(
      sample_info.descriptors,
      cam_img_info.descriptors,
      matches_filtered,
      knn_1to2_ratio,
      knn_1to2_ratio
    );
  #endif
  #ifndef USE_SIFT_GPU
    // Match descriptors (find the k=2 nearest neighbours)
    std::vector<std::vector<cv::DMatch> > matches;
    flann_matcher.knnMatch(sample_info.descriptors, cam_img_info.descriptors, matches, 2);
    // Filter matches:
    // By ratio of nearest and second nearest neighbour distance
    for(size_t i = 0; i < matches.size(); i++)
    {
      double ratio = matches[i][0].distance / matches[i][1].distance;
      if(ratio < knn_1to2_ratio)
      {
        matches_filtered.push_back(matches[i][0]);
      }
    }
  #endif
  // Locate objects
  if(matches_filtered.size() >= 4)
  {
    std::vector<cv::Point2f> object_in_scene,
                             scene;
    for(size_t i = 0; i < matches_filtered.size(); i++)
    {
      object_in_scene.push_back(sample_info.keypoints[matches_filtered[i].queryIdx].pt);
      scene.push_back(cam_img_info.keypoints[matches_filtered[i].trainIdx].pt);
    }
    // Compute homography
    cv::Mat homography = cv::findHomography(object_in_scene, scene, CV_RANSAC);
    // Compute perspective transform
    std::vector<cv::Point2f> object_corners = std::vector<cv::Point2f>(4),
                             scene_corners  = std::vector<cv::Point2f>(4);
    object_corners[0] = cvPoint(                0,                  0);
    object_corners[1] = cvPoint(sample_info.width,                  0);
    object_corners[2] = cvPoint(sample_info.width, sample_info.height);
    object_corners[3] = cvPoint(                0, sample_info.height);
    cv::perspectiveTransform(object_corners, scene_corners, homography);
    // Add object points to output before filtering false positives
    // (we still might want to visualize them for debugging purposes)
    object_points = scene_corners;
    // Filter false positives:
    // Check if object corners in scene are twisted
    #define p0 scene_corners[0]
    #define p1 scene_corners[1]
    #define p2 scene_corners[2]
    #define p3 scene_corners[3]
    cv::Point2f intersection;
    if(intersectLineSegments(p0, p1, p2, p3, intersection)
    || intersectLineSegments(p0, p3, p1, p2, intersection))
    {
      return false;
    }
    // Filter false positives:
    // Check if set of scene points is convex
    // (i.e. all points are part of the convex hull)
    std::vector<cv::Point2f> convex_hull;
    GrahamScanner::grahamScan(scene_corners, convex_hull);
    if(convex_hull.size() < scene_corners.size())
    {
      return false;
    }
    // Compute ratio of matched keypoints
    #ifdef  USE_SIFT_GPU
      confidence = (float) matches_filtered.size() / (float) sample_info.descriptors.size();
    #endif
    #ifndef USE_SIFT_GPU
      confidence = (float) matches_filtered.size() / (float) sample_info.descriptors.rows;
    #endif
    // Success
    return true;
  }
  else
  {
    return false;
  }
}
