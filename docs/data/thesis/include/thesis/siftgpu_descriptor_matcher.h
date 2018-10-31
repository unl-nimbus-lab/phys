/**
 *
 */
#ifndef __SIFTGPU_DESCRIPTOR_MATCHER__
#define __SIFTGPU_DESCRIPTOR_MATCHER__

#ifdef USE_SIFT_GPU

#include <opencv2/opencv.hpp>
#include <SiftGPU/SiftGPU.h>

/**
 *
 */
class SiftGPUDescriptorMatcher
{
  public:
    /**
     *
     */
    ~SiftGPUDescriptorMatcher();
  
    /**
     *
     */
    bool init();

    /**
     *
     */
    bool initialized();

    /**
     *
     */
    static SiftGPUDescriptorMatcher* getInstance();

    /**
	   * Is used for matching two sets of descriptors.
	   *
	   * @param descriptors1 The first descriptor.
	   * @param descriptors2 The second descriptor.
	   * @param matches      Is used to store the matches.
	   *
	   * @return false if there was an error during the matching process,
	   *         true  otherwise.
	   */
	  bool match(const std::vector<float>& descriptors1,
	             const std::vector<float>& descriptors2,
	             std::vector<cv::DMatch>& matches,
	             const double dist_max=0.7,
	             const double knn_1to2_ratio=0.9,
	             const int mutual_best_match=1);
  
  protected:
    /**
     *
     */
    SiftGPUDescriptorMatcher();
    
    /**
     *
     */
    static SiftGPUDescriptorMatcher* theInstance;
    
    /**
     *
     */
    static SiftMatchGPU* descriptorMatcher;
};

#endif // USE_SIFT_GPU
#endif // __SIFTGPU_DESCRIPTOR_MATCHER__
