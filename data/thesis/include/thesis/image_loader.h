/**
 * @author Carsten Könemann
 */
#ifndef __IMAGE_LOADER__
#define __IMAGE_LOADER__

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

class ImageLoader
{
  public:
    // Default constructor
    ImageLoader();
    // Default destructor
    ~ImageLoader();
    
    // Read a single sample image
    bool load_file(const boost::filesystem::path& path,
                   cv::Mat& out_image,
                   std::string& out_filename);
    
    // Read all sample images from a directory
    bool load_directory(const boost::filesystem::path& path,
                        std::vector<cv::Mat>& out_images,
                        std::vector<std::string>& out_filenames);
    
    // Load image from URL,
    // automatically determine if URL is a file or a directory
    bool load_url(const boost::filesystem::path& path,
                  std::vector<cv::Mat>& out_images,
                  std::vector<std::string>& out_filenames);

  protected:
    bool load(const std::string& path,
              cv::Mat& out_image);
};

#endif //__IMAGE_LOADER__
