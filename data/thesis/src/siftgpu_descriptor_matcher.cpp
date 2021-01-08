/**
 *
 */
#ifdef USE_SIFT_GPU

#include <thesis/siftgpu_descriptor_matcher.h>

#include <ros/ros.h>

SiftGPUDescriptorMatcher* SiftGPUDescriptorMatcher::theInstance       = NULL;
SiftMatchGPU*             SiftGPUDescriptorMatcher::descriptorMatcher = NULL;

SiftGPUDescriptorMatcher::SiftGPUDescriptorMatcher()
{
  // Default constructor
}

SiftGPUDescriptorMatcher::~SiftGPUDescriptorMatcher()
{
  // The singleton instance
  delete theInstance;
  // SiftGPU matcher instance
  delete descriptorMatcher;
}

bool SiftGPUDescriptorMatcher::init()
{
  // Create SiftMatchGPU instance
  descriptorMatcher = CreateNewSiftMatchGPU(4096);
  // Verify OpenGL context
  if(!descriptorMatcher->VerifyContextGL())
  {
    // Failure
    ROS_ERROR("SiftGPU Descriptor Matcher: Can't verify OpenGL context.");
    descriptorMatcher = NULL;
    return false;
  }
  // Success
  ROS_INFO("SiftGPU Descriptor Matcher: Initialized successfully.");
  return true;
}

bool SiftGPUDescriptorMatcher::initialized()
{
  return (descriptorMatcher != NULL);
}

SiftGPUDescriptorMatcher* SiftGPUDescriptorMatcher::getInstance()
{
  if(theInstance == NULL)
  {
    ROS_INFO("SiftGPU Descriptor Matcher: Creating instance.");
    theInstance = new SiftGPUDescriptorMatcher();
  }
  return theInstance;
}

bool SiftGPUDescriptorMatcher::match(const std::vector<float>& descriptors1,
                                     const std::vector<float>& descriptors2,
                                     std::vector<cv::DMatch>& matches,
                                     const double dist_max,
                                     const double knn_1to2_ratio,
                                     const int mutual_best_match)
{
  matches.clear();
  //
  if(descriptorMatcher == NULL)
  {
    ROS_WARN("SiftGPU Descriptor Matcher has not been initialized.");
    return false;
  }
  //
  int num1 = descriptors1.size() / 128,
      num2 = descriptors2.size() / 128;
  //
  descriptorMatcher->SetDescriptors(0, num1, &descriptors1[0]);
  descriptorMatcher->SetDescriptors(1, num2, &descriptors2[0]);
  //
  int (*match_buf)[2] = new int[num1][2];
  int nof_matches = descriptorMatcher->GetSiftMatch
  (
    num1,
    match_buf,
    dist_max,
    knn_1to2_ratio,
    mutual_best_match
  );
  ROS_DEBUG("SiftGPU Descriptor Matcher: #matches found: %i", nof_matches);
  //
  cv::DMatch match;
  int counter = 0;
  for(int i = 0; i < nof_matches; i++)
  {
    //
    match.queryIdx = match_buf[i][0];
    match.trainIdx = match_buf[i][1];
    // Only use matches with indices != 0
    // (OpenGL context problem may occur)
    if(match.queryIdx == 0 || match.trainIdx == 0)
    {
      counter++;
    }
    // 
    if(counter > 0.5 * (double) nof_matches)
    {
      ROS_DEBUG("SiftGPU Descriptor Matcher: Matches bad due to context error.");
      ROS_DEBUG("Counter:           %i.", counter);
      ROS_DEBUG("NOF Matches:       %i.", nof_matches);
      ROS_DEBUG("0.5 * NOF Matches: %f.", 0.5 * (double) nof_matches);
      matches.clear();
      return false;
    }
    // 
    float sum = 0;
    for(int j = 0; j < 128; j++)
    {
      float a = descriptors1[match.queryIdx * 128 + j]
              - descriptors2[match.trainIdx * 128 + j];
      sum += a * a;
    }
    match.distance = sqrt(sum);
    //
    matches.push_back(match);
  }
  //
  delete[] match_buf;
  //
  return true;
}

#endif // USE_SIFT_GPU
