/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>
#include <ros/package.h>

// Local headers
#include <thesis/math3d.h>
#include <thesis/timestring.h>
#include <thesis/semantic_map.h>

// We are working with TF
#include <tf/transform_listener.h>

// We want to subscribe to messages of this types
#include <thesis/ObjectInstance.h>
#include <thesis/ObjectInstanceArray.h>

// We want to call these services
#include <thesis/DatabaseGetByID.h>

// This node provides these services
#include <thesis/MappingGetAll.h>
#include <thesis/MappingGetByID.h>
#include <thesis/MappingGetByPosition.h>
#include <thesis/MappingGetVisible.h>

// C++ std libraries
#include <fstream>

using namespace geometry_msgs;

// Config parameters
std::string camera_frame,
            map_frame;

double      tf_timeout,
            age_threshold,
            fov_near,
            fov_far;

int         memory_size,
            min_confirmations,
            fov_width,
            fov_height;

bool        debug,
            logging;

// Transform listener
tf::TransformListener* transform_listener;

// Reusable service clients
ros::ServiceClient db_get_by_type_client;

// The actual semantic map, storing recognized objects with map coordinates
SemanticMap semantic_map;

// Time since last cleanup
ros::Time last_cleanup;

// Logfile
std::ofstream logfile;

void map_2_camera(const PoseStamped& pose_map, PoseStamped& pose_camera)
{
  try
  {
    // Make sure transformation exists at this point in time
    if(transform_listener->waitForTransform(
      map_frame,
      camera_frame,
      pose_map.header.stamp,
      ros::Duration(tf_timeout)
    ))
    {
      // Transform map pose to camera frame
      transform_listener->transformPose(camera_frame, pose_map, pose_camera);
      pose_camera.header.stamp = ros::Time::now();
    }
  }
  catch(tf::TransformException e)
  {
    ROS_ERROR("Mapping::map_2_camera(): ");
    ROS_ERROR("  tf::TransformException caught.");
    ROS_ERROR("  %s", e.what());
  }
}

void camera_2_map(const PoseStamped& pose_camera, PoseStamped& pose_map)
{
  try
  {
    // Make sure transformation exists at this point in time
    if(transform_listener->waitForTransform(
      camera_frame,
      map_frame,
      pose_camera.header.stamp,
      ros::Duration(tf_timeout)
    ))
    {
      // Transform recognized camera pose to map frame
      transform_listener->transformPose(map_frame, pose_camera, pose_map);
      pose_map.header.stamp = ros::Time(0);
    }
  }
  catch(tf::TransformException e)
  {
    ROS_ERROR("Mapping::camera_2_map(): ");
    ROS_ERROR("  tf::TransformException caught.");
    ROS_ERROR("  %s", e.what());
  }
}

cv::Point3f get_current_camera_position()
{
  //
  geometry_msgs::PoseStamped pose_camera;
  pose_camera.header.stamp    = ros::Time::now();
  pose_camera.header.frame_id = camera_frame;
  pose_camera.pose.position.x = 0;
  pose_camera.pose.position.y = 0;
  pose_camera.pose.position.z = 0;
  tf::quaternionTFToMsg(IDENTITY_QUATERNION, pose_camera.pose.orientation);
  //
  geometry_msgs::PoseStamped pose_map;
  camera_2_map(pose_camera, pose_map);
  //
  return ros2cv3f(pose_map.pose.position);
}

inline bool is_visible(geometry_msgs::Point p)
{
  // Calculate angles between point and camera direction
  float angle_width  = angle3f(cv::Point3f(p.x,  0.0f, p.z), cv::Point3f(0.0f, 0.0f, 1.0f)),
        angle_heigth = angle3f(cv::Point3f(0.0f, p.y,  p.z), cv::Point3f(0.0f, 0.0f, 1.0f));
  // Check if point is inside camera frustum
  //ROS_INFO("FAR:    %f, %f", p.z, fov_far);
  //ROS_INFO("NEAR:   %f, %f", p.z, fov_near);
  //ROS_INFO("WIDTH:  %f, %f", fabs(angle_width)*RAD,  fov_width/2.0f);
  //ROS_INFO("HEIGHT: %f, %f", fabs(angle_heigth)*RAD, fov_height/2.0f);
  return (fabs(angle_width)*RAD <= fov_width/2.0f && fabs(angle_heigth)*RAD <= fov_height/2.0f && p.z >= fov_near && p.z <= fov_far);
}

inline float get_shorter_side(std::string type)
{
  thesis::DatabaseGetByID db_get_by_type_service;
  db_get_by_type_service.request.id = type;
  if(db_get_by_type_client.call(db_get_by_type_service))
  {
    // Get dimensions
    float width  = db_get_by_type_service.response.object_class.width,
          height = db_get_by_type_service.response.object_class.height;
    // Return shorter side
    return std::min(width, height);
  }
  else
  {
    // Error
    ROS_WARN("Mapping::get_shorter_side(%s): ", type.c_str());
    ROS_WARN("  Failed to call service 'thesis_database/get_by_type'.");
    std::cout << std::endl;
    return NAN;
  }
}

inline void get_currently_visible(std::vector<thesis::ObjectInstance>& visible)
{
  visible.clear();
  // Get all objects
  std::vector<thesis::ObjectInstance> all_objects;
  semantic_map.setCurrentPosition(get_current_camera_position());
  semantic_map.getAll(all_objects);
  // Check which are currently visible
  for(size_t i = 0; i < all_objects.size(); i++)
  {
    // Transform object pose to camera coordinates
    // (which makes checking for visibility a lot easier)
    geometry_msgs::PoseStamped pose_camera;
    map_2_camera(all_objects[i].pose_stamped, pose_camera);
    // Check if object is currently visible
    if(is_visible(pose_camera.pose.position))
    {
      visible.push_back(all_objects[i]);
    }
  }
}

bool object_callback(const thesis::ObjectInstance& input, boost::uuids::uuid& id)
{
  // 
  thesis::ObjectInstance transformed;
  transformed.type_id    = input.type_id;
  transformed.confidence = input.confidence;
  // If available, transform recognized object pose to map frame
  if(!isnan(input.pose_stamped.pose.position) && !(input.pose_stamped.pose.position.x == 0 &&
                                                   input.pose_stamped.pose.position.y == 0 &&
                                                   input.pose_stamped.pose.position.z == 0))
  {
    // Compute min distance for object to be considered a new object
    float min_distance = get_shorter_side(input.type_id);
    //
    if(!isnan(min_distance) && min_distance > 0.0f)
    {
      // Debug output
      if(debug)
      {
        ROS_INFO("Mapping::object_callback(%s): ", input.type_id.c_str());
        ROS_INFO("  min_distance: %f.", min_distance);
        std::cout << std::endl;
      }
      // Transform object to map space
      camera_2_map(input.pose_stamped, transformed.pose_stamped);
      
      if(!isnan(input.pose_stamped.pose.position) && !(transformed.pose_stamped.pose.position.x == 0 &&
                                                       transformed.pose_stamped.pose.position.y == 0 &&
                                                       transformed.pose_stamped.pose.position.z == 0))
      {
        // Add object to semantic map
        return semantic_map.add(transformed, id, min_distance);
      }
      else
      {
        // Error
        if(debug)
        {
          ROS_INFO("Mapping::object_callback(%s): ", input.type_id.c_str());
          ROS_INFO("  Got bad transformed position values (at least one is NaN or 0).");
          std::cout << std::endl;
        }
        // 'true' might seem weird,
        // but it must remain consistent with SemanticMap::add()
        return true;
      }
    }
    else
    {
      // Error
      if(debug)
      {
        ROS_INFO("Mapping::object_callback(%s): ", input.type_id.c_str());
        ROS_INFO("  Database does not know dimensions yet.");
        ROS_INFO("  Don't add object to semantic map.");
        std::cout << std::endl;
      }
      // 'true' might seem weird,
      // but it must remain consistent with SemanticMap::add()
      return true;
    }
  }
  else
  {
    // Error
    if(debug)
    {
      ROS_INFO("Mapping::object_callback(%s): ", input.type_id.c_str());
      ROS_INFO("  Got bad object position values (at least one is NaN or 0).");
      std::cout << std::endl;
    }
    // 'true' might seem weird,
    // but it must remain consistent with SemanticMap::add()
    return true;
  }
}

void object_array_callback(const thesis::ObjectInstanceArray::ConstPtr& input)
{
  // Attempt cleanup if delay is up
  if((ros::Time::now() - last_cleanup).toSec() > age_threshold)
  {
    semantic_map.cleanup(age_threshold, (int) min_confirmations);
    last_cleanup = ros::Time::now();
  }
  // Get currently visible objects
  std::vector<thesis::ObjectInstance> visible;
  get_currently_visible(visible);
  size_t nof_visible = visible.size();
  // Add objects to semantic map
  for(size_t i = 0; i < input->array.size(); i++)
  {
    boost::uuids::uuid id;
    // 'false' if an object was updated instead of adding a new one
    if(!object_callback(input->array[i], id))
    {
      // Remove updated objects from currently visible objects
      std::vector<thesis::ObjectInstance>::iterator vec_iter = visible.begin();
      while(vec_iter != visible.end())
      {
        if(uuid_msgs::fromMsg(vec_iter->uuid) == id)
        {
          vec_iter = visible.erase(vec_iter);
          // We shouldn't find an object with the same UUID again
          break;
        }
        else
        {
          vec_iter++;
        }
      }
    }
  }
  // Flag not updated visible objects for removal
  for(size_t i = 0; i < visible.size(); i++)
  {
    semantic_map.flag(
      visible[i].type_id,
      uuid_msgs::fromMsg(visible[i].uuid),
      age_threshold
    );
  }
  // Logging
  if(logging && logfile.is_open())
  {
    // Get currently stored objects
    std::vector<thesis::ObjectInstance> all_objects;
    semantic_map.setCurrentPosition(get_current_camera_position());
    semantic_map.getAll(all_objects);
    // Log their poses
    for(size_t i = 0; i < all_objects.size(); i++)
    {
      // UUID
      logfile << all_objects[i].uuid << " ";
      // Position
      logfile << all_objects[i].pose_stamped.pose.position.x << " ";
      logfile << all_objects[i].pose_stamped.pose.position.y << " ";
      logfile << all_objects[i].pose_stamped.pose.position.z << " ";
      // Quaternion message to TF quaternion
      tf::Quaternion quaternion_tf;
      tf::quaternionMsgToTF(
        all_objects[i].pose_stamped.pose.orientation,
        quaternion_tf
      );
      // Quaternion to roll, pitch & yaw
      tf::Matrix3x3 matTemp(quaternion_tf);
      double roll, pitch, yaw;
      matTemp.getRPY(roll, pitch, yaw);
      // Orientation
      logfile << yaw   << " ";
      logfile << pitch << " ";
      logfile << roll  << "\n";
    }
  }
  // Debug output
  if(debug)
  {
    ROS_INFO("Mapping::object_array_callback(): ");
    ROS_INFO("  %lo visible objects.", nof_visible);
    ROS_INFO("  %lo of them were updated,", nof_visible - visible.size());
    ROS_INFO("  %lo were flagged for removal.", visible.size());
    std::cout << std::endl;
  }
}

bool get_all(thesis::MappingGetAll::Request& request,
             thesis::MappingGetAll::Response& result)
{
  semantic_map.setCurrentPosition(get_current_camera_position());
  semantic_map.getAll(result.objects);
  return true;
}

bool get_by_type(thesis::MappingGetByID::Request& request,
                 thesis::MappingGetByID::Response& result)
{
  semantic_map.setCurrentPosition(get_current_camera_position());
  semantic_map.getByID(request.id, result.objects);
  return true;
}

bool get_by_position(thesis::MappingGetByPosition::Request& request,
                     thesis::MappingGetByPosition::Response& result)
{
  semantic_map.setCurrentPosition(get_current_camera_position());
  semantic_map.getByPosition(ros2cv3f(request.position), result.objects);
  return true;
}

bool get_visible(thesis::MappingGetVisible::Request& request,
                 thesis::MappingGetVisible::Response& result)
{
  // Just a fail-safe, get_currently_visible() should be doing this anyway
  semantic_map.setCurrentPosition(get_current_camera_position());
  // Get currently visible objects
  get_currently_visible(result.objects);
  // Success
  return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  // Get global parameters
  nh.getParam("/thesis/camera_frame", camera_frame);
  nh.getParam("/thesis/map_frame",    map_frame);
  nh.getParam("/thesis/tf_timeout",   tf_timeout);
  nh.getParam("/thesis/memory_size",  memory_size);
  
  ROS_INFO("Mapping (global parameters): ");
  ROS_INFO("  Camera frame: %s.", camera_frame.c_str());
  ROS_INFO("  Map frame:    %s.", map_frame.c_str());
  ROS_INFO("  TF timeout:   %f.", tf_timeout);
  ROS_INFO("  Memory size:  %i.", memory_size);
  std::cout << std::endl;
  
  // Get local parameters
  nh_private.param("debug",             debug,             false);
  nh_private.param("logging",           logging,           false);
  nh_private.param("age_threshold",     age_threshold,     1.0);
  nh_private.param("min_confirmations", min_confirmations, 0);
  nh_private.param("fov_width",         fov_width,         90);
  nh_private.param("fov_height",        fov_height,        90);
  nh_private.param("fov_near",          fov_near,          0.0);
  nh_private.param("fov_far",           fov_far,           5.0);
  
  ROS_INFO("Mapping (local parameters): ");
  ROS_INFO("  Debug mode:        %s.", debug   ? "true" : "false");
  ROS_INFO("  Logging:           %s.", logging ? "true" : "false");
  ROS_INFO("  Age threshold:     %f.", age_threshold);
  ROS_INFO("  Min confirmations: %i.", min_confirmations);
  ROS_INFO("  FOV width:         %i.", fov_width);
  ROS_INFO("  FOV height:        %i.", fov_height);
  ROS_INFO("  FOV near:          %f.", fov_near);
  ROS_INFO("  FOV far:           %f.", fov_far);
  std::cout << std::endl;
  
  // Create transform listener
  transform_listener = new tf::TransformListener();
  
  // Initialize reusable service clients
  ros::service::waitForService("thesis_database/get_by_type", -1);
  db_get_by_type_client = nh.serviceClient<thesis::DatabaseGetByID>("thesis_database/get_by_type");
  
  // Subscribe to relevant topics
  ros::Subscriber object_subscriber = nh.subscribe("thesis_recognition/object_pose", 1, object_array_callback);
  
  // Advertise services
  ros::ServiceServer srv_all         = nh_private.advertiseService("all", get_all);
  ros::ServiceServer srv_by_type     = nh_private.advertiseService("by_type", get_by_type);
  ros::ServiceServer srv_by_position = nh_private.advertiseService("by_position", get_by_position);
  ros::ServiceServer srv_visible     = nh_private.advertiseService("visible", get_visible);
  
  // Initialize semantic map
  semantic_map = SemanticMap(memory_size, debug);
  last_cleanup = ros::Time::now();
  
  // Open logfile
  if(logging)
  {
    // Create logfile name
    std::stringstream logpath;
    // Path to this package
    logpath << ros::package::getPath("thesis");
    logpath << "/log/3D-poses_";
    // Current time and date
    logpath << timestring::get_timestring();
    // File extension
    logpath << ".log";
    // Open file
    logfile.open(logpath.str().c_str(), std::ofstream::out | std::ofstream::app);
    if(logfile.is_open())
    {
      logfile << "# 3D poses of objects stored in the semantic map.\n";
      logfile << "# UUID xPos yPos zPos yaw pitch roll\n";
      // Debug output
      ROS_INFO("Mapping: ");
      ROS_INFO("  Successfully opened logfile '%s'.", logpath.str().c_str());
      std::cout << std::endl;
    }
    else
    {
      ROS_INFO("Mapping: ");
      ROS_INFO("  Unable to open logfile '%s'.", logpath.str().c_str());
      std::cout << std::endl;
    }
  }
  
  // Spin
  ros::spin();
  
  // Free memory
  delete transform_listener;
  
  // Close logfile
  if(logging && logfile.is_open())
  {
    logfile.flush();
    logfile.close();
  }
  
  // Exit
  return 0;
}
