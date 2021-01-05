/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Local headers
#include <thesis/database.h>
#include <thesis/image_loader.h>

// Since we use OpenCV to load image files,
// we need to convert them to ROS messages
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// We are going to try getting the resolution of an OpenNI camera (if available)
#include <sensor_msgs/CameraInfo.h>

// My message types
#include <thesis/ObjectClass.h>
#include <thesis/ObjectClassArray.h>

// Empty message to inform subscribing nodes about changes
#include <std_msgs/Empty.h>

// This node provides these services
#include <thesis/DatabaseAddFile.h>
#include <thesis/DatabaseAddImg.h>
#include <thesis/DatabaseAddURL.h>
#include <thesis/DatabaseGetAll.h>
#include <thesis/DatabaseGetByID.h>

// Config parameters
std::string camera_info_topic,
            image_path;
            
cv::Size    min_image_size,
            max_image_size;

bool        debug;

double      openni_timeout;

int         memory_size;

// We try to get the image size of a connected OpenNI camera (if available)
bool openni_once = false;
cv::Size openni_image_size;

// 
ImageLoader image_loader;

Database    database;

// We are going to inform subscribing nodes about changes
ros::Publisher update_publisher;


/**
 * Adders.
 */

bool callback_add_urls(thesis::DatabaseAddURL::Request& request,
                       thesis::DatabaseAddURL::Response& result)
{
  for(size_t i = 0; i < request.urls.size(); i++)
  {
    std::vector<cv::Mat> images;
    std::vector<std::string> filenames;
    if(image_loader.load_url(request.urls[i], images, filenames))
    {
      for(size_t j = 0; j < images.size(); j++)
      {
        database.add_image(images[j], filenames[j], min_image_size, max_image_size);
      }
    }
  }
  //
  std_msgs::Empty msg;
  update_publisher.publish(msg);
  //
  return true;
}

bool callback_add_files(thesis::DatabaseAddFile::Request& request,
                        thesis::DatabaseAddFile::Response& result)
{
  for(size_t i = 0; i < request.urls.size(); i++)
  {
    cv::Mat image;
    std::string filename;
    if(image_loader.load_file(request.urls[i], image, filename))
    {
      if(request.names.size() >= i && request.names[i].length() > 0)
      {
        database.add_image(image, request.names[i], min_image_size, max_image_size);
      }
      else
      {
        ROS_INFO("Database::callback_add_files(): ");
        ROS_INFO("  Image name is empty. Using filename ('%s') instead.", filename.c_str());
        database.add_image(image, filename, min_image_size, max_image_size);
      }
    }
  }
  //
  std_msgs::Empty msg;
  update_publisher.publish(msg);
  //
  return true;
}

bool callback_add_images(thesis::DatabaseAddImg::Request& request,
                         thesis::DatabaseAddImg::Response& result)
{
  for(size_t i = 0; i < request.images.size(); i++)
  {
    database.add_image(request.images[i], request.names[i], min_image_size, max_image_size);
  }
  //
  std_msgs::Empty msg;
  update_publisher.publish(msg);
  //
  return true;
}


/**
 * Getters.
 */

bool callback_get_all(thesis::DatabaseGetAll::Request& request,
                      thesis::DatabaseGetAll::Response& result)
{
  result.object_classes = database.getAll();
  return true;
}

bool callback_get_by_type(thesis::DatabaseGetByID::Request& request,
                          thesis::DatabaseGetByID::Response& result)
{
  thesis::ObjectClass temp;
  if(database.getByID(request.id, temp))
  {
    result.object_class = temp;
    return true;
  }
  // For debugging purposes
  else if(request.id == "Identity")
  {
    thesis::ObjectClass identity;
    identity.type_id = "Identity";
    identity.width   = 0.1;
    identity.height  = 0.2;
    result.object_class = identity;
    return true;
  }
  else
  {
    return false;
  }
}


/**
 * Non-service callbacks.
 */

void callback_openni_once(const sensor_msgs::CameraInfo::ConstPtr& input)
{
  openni_image_size.width  = input->width;
  openni_image_size.height = input->height;
  openni_once = true;
}

void callback_object_dimension(const thesis::ObjectClassArray::ConstPtr& input)
{
  for(size_t i = 0; i < input->array.size(); i++)
  {
    if(!database.update(input->array[i]) && debug)
    {
      if(debug)
      {
        ROS_WARN("Database::callback_object_dimension(): ");
        ROS_WARN("  Unable to update values for '%s'.", input->array[i].type_id.c_str());
        std::cout << std::endl;
      }
    }
    else
    {
      if(debug)
      {
        ROS_INFO("Database::callback_object_dimension(): ");
        ROS_INFO("  Updating values for '%s': ", input->array[i].type_id.c_str());
        ROS_INFO("    Confidence: %f.", input->array[i].confidence);
        ROS_INFO("    Width:      %f.", input->array[i].width);
        ROS_INFO("    Height:     %f.", input->array[i].height);
        std::cout << std::endl;
      }
    }
  }
}


/**
 * Main loop.
 */

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_database");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  // Get global parameters
  nh.getParam("/thesis/camera_info_topic", camera_info_topic);
  nh.getParam("/thesis/openni_timeout",    openni_timeout);
  nh.getParam("/thesis/memory_size",       memory_size);
  
  ROS_INFO("Database (global parameters): ");
  ROS_INFO("  Camera info topic: %s.", camera_info_topic.c_str());
  ROS_INFO("  OpenNI timeout:    %f.", openni_timeout);
  ROS_INFO("  Memory size:       %i.", memory_size);
  std::cout << std::endl;
  
  // Get local parameters
  nh_private.param("debug",            debug,                 false);
  nh_private.param("image_path",       image_path,            std::string("img"));
  nh_private.param("min_image_width",  min_image_size.width,  1);
  nh_private.param("min_image_height", min_image_size.height, 1);
  nh_private.param("max_image_width",  max_image_size.width,  1280);
  nh_private.param("max_image_height", max_image_size.height, 1024);
  
  ROS_INFO("Database (local parameters): ");
  ROS_INFO("  Debug mode:        %s.", debug ? "true" : "false");
  ROS_INFO("  Sample image path: %s.", image_path.c_str());
  ROS_INFO("  Min image width:   %i.", min_image_size.width);
  ROS_INFO("  Min image height:  %i.", min_image_size.height);
  std::cout << std::endl;
  
  // Try to get OpenNI camera image size
  ros::Subscriber camera_info_subscriber = nh.subscribe(camera_info_topic, 1, callback_openni_once);
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
  camera_info_subscriber.shutdown();
  
  // Set image size limits
  if(openni_once)
  {
    ROS_ASSERT_MSG(openni_image_size.width > 0 && openni_image_size.height > 0,
                   "Got bad image size from OpenNI camera.");
    max_image_size.width  = openni_image_size.width;
    max_image_size.height = openni_image_size.height;
    ROS_INFO("Database: ");
    ROS_INFO("  Initializing image resolution limits:");
    ROS_INFO("  Got the following image resolution from OpenNI camera: %ix%i",
             openni_image_size.width, openni_image_size.height);
    std::cout << std::endl;
  }
  else
  {
    ROS_WARN("Database: ");
    ROS_WARN("  Initializing image resolution limits:");
    ROS_WARN("  Unable to get image resolution from OpenNI camera.");
    ROS_WARN("  Using default or given max resolution (%ix%i) instead.",
             max_image_size.width, max_image_size.height);
    std::cout << std::endl;
  }
  
  // Create sample database
  database = Database(memory_size, debug);
  std::vector<cv::Mat> images;
  std::vector<std::string> filenames;
  image_loader.load_directory(image_path, images, filenames);
  std::cout << std::endl; // after debug output from load_directory()
  for(size_t i = 0; i < images.size(); i++)
  {
    database.add_image(images[i], filenames[i], min_image_size, max_image_size);
  }
  
  // Advertise services
  ros::ServiceServer srv_add_urls    = nh_private.advertiseService("add_urls",    callback_add_urls);
  ros::ServiceServer srv_add_files   = nh_private.advertiseService("add_files",   callback_add_files);
  ros::ServiceServer srv_add_images  = nh_private.advertiseService("add_images",  callback_add_images);
  ros::ServiceServer srv_get_all     = nh_private.advertiseService("get_all",     callback_get_all);
  ros::ServiceServer srv_get_by_type = nh_private.advertiseService("get_by_type", callback_get_by_type);
  
  // Subscribe to updates on object metadata (e.g. perceived dimensions)
  ros::Subscriber object_meta_subscriber = nh.subscribe("thesis_recognition/object_dimension", 1, callback_object_dimension);
  
  // We are going to inform subscribing nodes about changes
  update_publisher = nh_private.advertise<std_msgs::Empty>("updates", 10);
  
  // Spin
  ros::spin();
  // Exit
  return 0;
}
