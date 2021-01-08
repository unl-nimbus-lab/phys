/**
 * @author Carsten Könemann
 */

#include <thesis/database.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

thesis::ObjectClass Database::ObjectQueue::combined()
{
  thesis::ObjectClass o;
  double confidence_sum = 0.0;
  
  // Combine values, weighted by confidence
  std::deque<thesis::ObjectClass>::iterator iter = deque.begin();
  for(; iter != deque.end(); iter++)
  {
    o.confidence   += iter->confidence * iter->confidence;
    o.width        += iter->confidence * iter->width;
    o.height       += iter->confidence * iter->height;
    // Remember total of all weightings
    confidence_sum += iter->confidence;
  }
  
  // Average values
  o.confidence /= confidence_sum;
  o.width      /= confidence_sum;
  o.height     /= confidence_sum;
  
  if(debug)
  {
    ROS_INFO("Database::ObjectQueue::combined(): ");
    ROS_INFO("  confidence_sum = %f", confidence_sum);
    ROS_INFO("  confidence     = %f", o.confidence);
    ROS_INFO("  width          = %f", o.width);
    ROS_INFO("  height         = %f", o.height);
    std::cout << std::endl;
  }
  
  return o;
}

void Database::ObjectQueue::add(thesis::ObjectClass o)
{
  // enqueue
  deque.push_back(o);
  // dequeue
  if((int) deque.size() > memory_size)
  {
    deque.pop_front();
  }
}

bool Database::exists(std::string id)
{
  try
  {
    database.at(id);
  }
  catch(const std::out_of_range& oor)
  {
    return false;
  }
  return true;
}

bool Database::update(const thesis::ObjectClass& input)
{
  if(exists(input.type_id))
  {
    if(!isnan(input.width) && !isnan(input.height))
    {
      database[input.type_id].values.add(input);
      return true;
    }
    else if(debug)
    {
      ROS_WARN("Database: Do not add NaN values for '%s' to database.", input.type_id.c_str());
      return false;
    }
  }
  else if(debug)
  {
    ROS_WARN("Database: No object '%s' found to update.", input.type_id.c_str());
    return false;
  }
  // else
  return false;
}

bool Database::getByID(std::string id, thesis::ObjectClass& out)
{
  if(exists(id))
  {
    // Get entry
    Entry e = database[id];
    // Get average values
    out = e.values.combined();
    // Add "static" values
    out.type_id = id;
    out.image   = e.image;
    // Success
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<thesis::ObjectClass> Database::getAll()
{
  std::vector<thesis::ObjectClass> out;
  //
  std::map<std::string, Entry>::iterator iter = database.begin();
  for(; iter != database.end(); iter++)
  {
    thesis::ObjectClass temp;
    if(getByID(iter->first, temp))
    {
      out.push_back(temp);
    }
  }
  //
  return out;
}

bool Database::add_image
(
  cv::Mat&    image,
  std::string name,
  cv::Size    min_image_size,
  cv::Size    max_image_size
)
{
  // Check if image name is valid
  if(!(name.length() > 0))
  {
    ROS_WARN("Error while trying to add an image to the database:");
    ROS_WARN("  Image name empty.");
    ROS_WARN("  Don't add it to the database.");
    std::cout << std::endl;
    return false;
  }
  // Check if an image of this name already exists in the database
  if(exists(name))
  {
    ROS_WARN("Error while trying to add an image to the database:");
    ROS_WARN("  Image '%s' already exists in the database.", name.c_str());
    ROS_WARN("  Don't add it to the database again.");
    std::cout << std::endl;
    return false;
  }
  // Check if image is large enough
  if(image.cols < min_image_size.width || image.rows < min_image_size.height)
  {
    ROS_WARN("Error while trying to add an image to the database:");
    ROS_WARN("  Image '%s' is smaller than %i x %i.",
      name.c_str(),
      min_image_size.width,
      min_image_size.height
    );
    ROS_WARN("  Don't add it to database.");
    std::cout << std::endl;
    return false;
  }
  // Determine sample image orientation and rotate max_image_size accordingly
  if(max_image_size.width < max_image_size.height)
  {
    max_image_size
      = (image.cols <  image.rows)
      ? max_image_size
      : cv::Size(max_image_size.height, max_image_size.width);
  }
  else
  {
    max_image_size
      = (image.cols >= image.rows)
      ? max_image_size
      : cv::Size(max_image_size.height, max_image_size.width);
  }
  // Resize image (if too large)
  cv::Mat image_resized;
  if(image.cols > max_image_size.width || image.rows > max_image_size.height)
  {
    cv::Size re_size;
    // Determine aspect ratio...
    float aspect_ratio_width
      = (float) max_image_size.width  / (float) image.cols;
    float aspect_ratio_height
      = (float) max_image_size.height / (float) image.rows;
    // ...determine biggest possible new size with the same aspect ratio...
    if(aspect_ratio_width < aspect_ratio_height)
    {
      re_size = cv::Size(image.cols * aspect_ratio_width,
                         image.rows * aspect_ratio_width);
    }
    else
    {
      re_size = cv::Size(image.cols * aspect_ratio_height,
                         image.rows * aspect_ratio_height);
    }
    // Debug output
    ROS_INFO("While trying to add an image to the database:");
    ROS_INFO("  Image '%s' is bigger than %i x %i.",
      name.c_str(),
      max_image_size.width,
      max_image_size.height
    );
    ROS_INFO("  Resizing it to %i x %i.", re_size.width, re_size.height);
    std::cout << std::endl;
    // ...and resize it accordingly
    cv::resize(image, image_resized, re_size);
  }
  else
  {
    image_resized = image;
  }
  // We already made sure an entry for this type doesn't already exist (see above):
  // Trying to access an entry that not exists will initialize an empty entry.
  Entry* entry = &database[name];
  // Initialize ObjectQueue of the newly created entry to actually hold values
  entry->values = ObjectQueue(memory_size);
  // Convert OpenCV image to ROS image message
  cv_bridge::CvImage cv_image;
  cv_image.encoding = sensor_msgs::image_encodings::MONO8;
  cv_image.image = image_resized;
  cv_image.toImageMsg(entry->image);
  // Success
  return true;
}

bool Database::add_image
(
  sensor_msgs::Image& image,
  std::string         name,
  cv::Size            min_image_size,
  cv::Size            max_image_size
)
{
  // Convert ROS image message to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  // Add OpenCV image
  return add_image(cv_ptr->image, name, min_image_size, max_image_size);
}
