/**
 *
 */
#ifndef __SIFTGPU_FEATURE_DETECTOR__
#define __SIFTGPU_FEATURE_DETECTOR__

#ifdef USE_SIFT_GPU

#include <opencv2/opencv.hpp>
#include <SiftGPU/SiftGPU.h>

/**
 *
 */
class SiftGPUFeatureDetector
{
  public:
    /**
     *
     */
    ~SiftGPUFeatureDetector();
  
    /**
     *
     */
    bool init(const int max_nof_keypoints=4096,
              const int max_image_size=1280);

    /**
     *
     */
    bool initialized();

    /**
     *
     */
    static SiftGPUFeatureDetector* getInstance();

    /**
     *
     */
    void cvMatToSiftGPU(const cv::Mat& image, unsigned char* siftImage);
    
    /**
     *
     */
    void setMaxImageSize(const int max_image_size);
    
    /**
     *
     */
    void setMaxKeypoints(const int max_nof_keypoints);
    
    /**
     * Compute descriptors for existing keypoints.
     *
     * @param image       The image.
	   * @param keypoints   The existing keypoints.
	   * @param descriptors The computed descriptors (output).
     */
    bool compute(const cv::Mat& image,
                 const std::vector<cv::KeyPoint>& keypoints,
                 std::vector<float>& descriptors);

    /**
	   * Detect keypoints and compute their descriptors.
	   *
	   * @param image       The image.
	   * @param keypoints   The detected keypoints (output).
	   * @param descriptors The computed descriptors (output).
	   */
	  bool detect(const cv::Mat& image,
	              std::vector<cv::KeyPoint>& keypoints,
	              std::vector<float>& descriptors);
  
  protected:
    /**
     *
     */
    SiftGPUFeatureDetector();
    
    /**
     *
     */
    static SiftGPUFeatureDetector* theInstance;
    
    /**
     *
     */
    static SiftGPU* featureDetector;
    
    /**
	   * 
	   */
	  unsigned char* data;
	  
	  /**
	   * 
	   */
	  int data_size;
	  
	  /**
	   *
	   */
	  int max_nof_keypoints;
};

#endif // USE_SIFT_GPU
#endif // __SIFTGPU_FEATURE_DETECTOR__
