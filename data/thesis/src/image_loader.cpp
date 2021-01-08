/**
 * @author Carsten Könemann
 */

#include <thesis/image_loader.h>

#include <ros/ros.h>

ImageLoader::ImageLoader()
{
  // Default constructor
}

ImageLoader::~ImageLoader()
{
  // Default destructor
}

bool ImageLoader::load(const std::string& path,
                       cv::Mat& out_image)
{
  out_image = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
  if(!out_image.empty())
  {
    ROS_INFO("Loading '%s'.", path.c_str());
    return true;
  }
  else
  {
    ROS_WARN("Unable to load image '%s'.", path.c_str());
    return false;
  }
}

bool ImageLoader::load_file(const boost::filesystem::path& path,
                            cv::Mat& out_image,
                            std::string& out_filename)
{
  if(boost::filesystem::exists(path))
  {
    if(!boost::filesystem::is_directory(path))
    {
      std::string filename = boost::filesystem::basename(path.filename());
      if(filename.length() > 0)
      {
        cv::Mat image;
        if(load(path.string(), image))
        {
          out_image = image;
          out_filename = filename;
        }
      }
      return true;
    }
    else
    {
      ROS_WARN("'%s' is not a file.", path.string().c_str());
      return false;
    }
  }
  else
  {
    ROS_WARN("File '%s' not found.", path.string().c_str());
    return false;
  }
}

bool ImageLoader::load_directory(const boost::filesystem::path& path,
                                 std::vector<cv::Mat>& out_images,
                                 std::vector<std::string>& out_filenames)
{
  if(boost::filesystem::exists(path))
  {
    if(boost::filesystem::is_directory(path))
    {
      // Check all elements contained in given directory
      boost::filesystem::directory_iterator end_itr;
      for(boost::filesystem::directory_iterator itr(path); itr != end_itr; itr++)
      {
        // Only load try loading files, not directories
        if(!boost::filesystem::is_directory(itr->status()))
        {
          std::string filename = boost::filesystem::basename(itr->path().filename());
          if(filename.length() > 0)
          {
            cv::Mat image;
            if(load(itr->path().string(), image))
            {
              out_images.push_back(image);
              out_filenames.push_back(filename);
            }
          }
        }
      }
      return true;
    }
    else
    {
      ROS_WARN("'%s' is not a directory.", path.string().c_str());
      return false;
    }
  }
  else
  {
    ROS_WARN("Directory '%s' not found.", path.string().c_str());
    return false;
  }
}

bool ImageLoader::load_url(const boost::filesystem::path& path,
                           std::vector<cv::Mat>& out_images,
                           std::vector<std::string>& out_filenames)
{
  if(boost::filesystem::exists(path))
  {
    if(boost::filesystem::is_directory(path))
    {
      return load_directory(path, out_images, out_filenames);
    }
    else
    {
      cv::Mat image;
      std::string filename;
      if(load_file(path, image, filename))
      {
        out_images.push_back(image);
        out_filenames.push_back(filename);
        return true;
      }
      else
      {
        ROS_WARN("Unable to load URL '%s'.", path.string().c_str());
        return false;
      }
    }
  }
  else
  {
    ROS_WARN("URL '%s' not found.", path.string().c_str());
    return false;
  }
}
