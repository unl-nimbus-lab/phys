/**
 *
 */
#ifdef USE_SIFT_GPU

#include <thesis/siftgpu_feature_detector.h>

#include <ros/ros.h>

#include <GL/gl.h>

SiftGPUFeatureDetector* SiftGPUFeatureDetector::theInstance     = NULL;
SiftGPU*                SiftGPUFeatureDetector::featureDetector = NULL;

SiftGPUFeatureDetector::SiftGPUFeatureDetector()
{
  // Default constructor
}

SiftGPUFeatureDetector::~SiftGPUFeatureDetector()
{
  // The singleton instance
  delete theInstance;
  // SiftGPU instance
  delete featureDetector;
  // SiftGPU image data
  free(data);
}

bool SiftGPUFeatureDetector::init(const int max_nof_keypoints,
                                  const int max_image_size)
{
  data = NULL;
  data_size = 0;
  
  featureDetector = new SiftGPU();
  
  //
  char method[]           = {"-glsl"};

  //
  char subpixel_key[]     = {"-s"};
  char subpixel_val[]     = {"0"};
  
  //
  char max_feat_key[]     = {"-tc2"};
  char max_feat_val[16];
  snprintf(max_feat_val, 16, "%d", max_nof_keypoints);

  //
  char first_octave_key[] = {"-fo"};
  char first_octave_val[] = {"0"};
  
  // nothing but errors
  char verbosity_key[]    = {"-v"};
  char verbosity_val[]    = {"0"};
  
  // 
  char pyramid_key[]      = {"-p"};
  char pyramid_val[16];
  snprintf(pyramid_val, 16, "%dx%d", max_image_size, max_image_size);
  
  char* argv[] =
  {
    method,
    subpixel_key,
    subpixel_val,
    max_feat_key,
    max_feat_val,
    first_octave_key,
    first_octave_val,
    verbosity_key,
    verbosity_val,
    pyramid_key,
    pyramid_val
  };
  
  featureDetector->ParseParam(11, argv);

  if(featureDetector->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
  {
    ROS_ERROR("SiftGPU Feature Detector: Can't create OpenGL context.");
    featureDetector = NULL;
    return false;
  }
  
  //
  ROS_INFO("SiftGPU Feature Detector: Initialized.");
  ROS_INFO("  Max number of keypoints: %i.", max_nof_keypoints);
  ROS_INFO("  Max image size:          %s.", pyramid_val);
  
  //
  return true;
}

bool SiftGPUFeatureDetector::initialized()
{
  return (featureDetector != NULL);
}

SiftGPUFeatureDetector* SiftGPUFeatureDetector::getInstance()
{
  if(theInstance == NULL)
  {
    ROS_INFO("SiftGPU Feature Detector: Creating instance.");
    theInstance = new SiftGPUFeatureDetector();
  }
  return theInstance;
}

void SiftGPUFeatureDetector::cvMatToSiftGPU(const cv::Mat& image,
                                            unsigned char* siftImage)
{
  cv::Mat tmp;
  image.convertTo(tmp, CV_8U);
  for(int y = 0; y < tmp.rows; ++y)
  {
    for(int x = 0; x < tmp.cols; ++x)
    {
      siftImage[y * tmp.cols + x] = tmp.at<unsigned char>(y, x);
    }
  }
}

void SiftGPUFeatureDetector::setMaxImageSize(const int max_image_size)
{
  //
  char pyramid_key[] = {"-p"};
  char pyramid_val[16];
  snprintf(pyramid_val, 16, "%dx%d", max_image_size, max_image_size);
  //
  char* argv[] = { pyramid_key, pyramid_val };
  //
  featureDetector->ParseParam(2, argv);
}

void SiftGPUFeatureDetector::setMaxKeypoints(const int max_nof_keypoints)
{
  //
  char max_feat_key[] = {"-tc2"};
  char max_feat_val[16];
  snprintf(max_feat_val, 16, "%d", max_nof_keypoints);
  //
  char* argv[] = { max_feat_key, max_feat_val };
  //
  featureDetector->ParseParam(2, argv);
}

bool SiftGPUFeatureDetector::compute(const cv::Mat& image,
                                     const std::vector<cv::KeyPoint>& keypoints,
                                     std::vector<float>& descriptors)
{
  //
  if(featureDetector == NULL)
  {
    ROS_WARN("SiftGPU Feature Detector has not been initialized.");
    return false;
  }
  
  //
  std::vector<SiftGPU::SiftKeypoint> siftkeys;
  
  // Convert input keypoints to SiftGPU structure
  for(size_t i = 0; i < keypoints.size(); i++)
  {
    SiftGPU::SiftKeypoint key;
    key.x = keypoints[i].pt.x;
    key.y = keypoints[i].pt.y;
    // 6x scale is the conversion from pixels
    // (according to Changchang Wu, the author of SiftGPU)
    key.s = keypoints[i].size / 6.0;
    key.o = keypoints[i].angle;
    siftkeys.push_back(key);
  }
  
  // Allocate memory to store image data
  // No need to allocate memory again for multiple images of the same size
  int current_image_size = image.rows * image.cols;
  if(current_image_size > data_size)
  {
    data_size = current_image_size;
    free(data);
    data = (unsigned char*) malloc(data_size);
  }
  
  // Convert OpenCV image to SiftGPU image
  cvMatToSiftGPU(image, data);
  
  // Specify the keypoints for next image to siftgpu
  featureDetector->SetKeypointList(siftkeys.size(), &siftkeys[0]);
  
  //
  if(featureDetector->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE))
  {
    //
    descriptors.resize(128 * siftkeys.size());
    featureDetector->GetFeatureVector(NULL, &descriptors[0]);
  }
  else
  {
    ROS_WARN("SiftGPU Feature Detector: SiftGPU->RunSIFT() failed.");
    return false;
  }
  
  return true;
}

bool SiftGPUFeatureDetector::detect(const cv::Mat& image,
	                                  std::vector<cv::KeyPoint>& keypoints,
	                                  std::vector<float>& descriptors)
{
  keypoints.clear();
  
  //
  if(featureDetector == NULL)
  {
    ROS_WARN("SiftGPU Feature Detector has not been initialized.");
    return false;
  }
  
  // Allocate memory to store image data
  // No need to allocate memory again for multiple images of the same size
  int current_image_size = image.rows * image.cols;
  if(current_image_size > data_size)
  {
    data_size = current_image_size;
    free(data);
    data = (unsigned char*) malloc(data_size);
  }
  
  // Convert OpenCV image to SiftGPU image
  cvMatToSiftGPU(image, data);

  //
  int num_features = 0;
  
  if(featureDetector->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE))
  {
    //
    num_features = featureDetector->GetFeatureNum();
    ROS_DEBUG("SiftGPU Feature Detector: #features found: %i", num_features);
  }
  else
  {
    ROS_WARN("SiftGPU Feature Detector: SiftGPU->RunSIFT() failed.");
    return false;
  }

  //
  std::vector<SiftGPU::SiftKeypoint> siftkeys(num_features);
  descriptors.resize(128 * num_features);
  
  //
  featureDetector->GetFeatureVector(&siftkeys[0], &descriptors[0]);

  // Convert SiftGPU keypoints to OpenCV structure
  for(int i = 0; i < num_features; ++i)
  {
    // 6x scale is the conversion to pixels
    // (according to Changchang Wu, the author of SiftGPU)
    cv::KeyPoint key(siftkeys[i].x, siftkeys[i].y, 6.0 * siftkeys[i].s, siftkeys[i].o);
    keypoints.push_back(key);
  }
  
  //
  return true;
}

#endif // USE_SIFT_GPU
