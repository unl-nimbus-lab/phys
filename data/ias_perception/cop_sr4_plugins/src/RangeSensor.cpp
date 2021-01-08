#include "RangeSensor.h"


#include <ros/ros.h>


extern volatile bool g_stopall;

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>



#define XML_PROPERTY_TOPIC_SR4 "Topic"
#define XML_PROPERTY_POINT_CLOUD2 "PointCloud2"
#define XML_PROPERTY_INTEGRATE "Integrate"

#define XML_PROPERTY_FOCALLENGTH "focal_length"
#define XML_PROPERTY_DISTPARAM "dist_param"
#define XML_PROPERTY_PIXWIDTH "sx"
#define XML_PROPERTY_PIXHEIGHT "sy"
#define XML_PROPERTY_CX "cx"
#define XML_PROPERTY_CY "cy"
#define XML_PROPERTY_WIDTH "width"
#define XML_PROPERTY_HEIGHT "height"

#define XML_PROPERTY_LINKPARAM_NAMESPACE "param_ns"

using namespace cop;

RangeSensor::RangeSensor() :
    m_grabbing(false),
    m_bintegration(true),
    m_subscribePointCloud2(false),
    m_self_filter(NULL)
{
  printf("Successfully initialized plugin class RangeSensor\n");
  m_max_cameraImages = 1;
}


RangeSensor::~RangeSensor()
{
  Stop();
  if(m_self_filter != NULL)
    delete m_self_filter;
}

/**
*  This can be overwritten to get the data necessary to reconstruct a saved reading
*/
void RangeSensor::SetData(cop::XMLTag* tag)
{
  Sensor::SetData(tag);
  printf("Set data\n");
  m_stTopic = tag->GetProperty(XML_PROPERTY_TOPIC_SR4, "cloud_sr");
  m_subscribePointCloud2  = tag->GetPropertyInt(XML_PROPERTY_POINT_CLOUD2, 0) != 0;
  m_bintegration = tag->GetPropertyInt(XML_PROPERTY_INTEGRATE, 1) == 1;
  m_calibration.resize(8);
  m_calibration[0] = tag->GetPropertyDouble(XML_PROPERTY_FOCALLENGTH, 0.0105631);
  m_calibration[1] = tag->GetPropertyDouble(XML_PROPERTY_DISTPARAM , -8623.59);
  m_calibration[2] = tag->GetPropertyDouble(XML_PROPERTY_PIXWIDTH,  3.99851e-05);
  m_calibration[3] = tag->GetPropertyDouble(XML_PROPERTY_PIXHEIGHT, 4e-05);
  m_calibration[4] = tag->GetPropertyDouble(XML_PROPERTY_CX, 87.8255);
  m_calibration[5] = tag->GetPropertyDouble(XML_PROPERTY_CY, 71.9162);
  m_calibration[6] = tag->GetPropertyDouble(XML_PROPERTY_WIDTH, 176.0);
  m_calibration[7] = tag->GetPropertyDouble(XML_PROPERTY_HEIGHT, 144.0);

  m_paramNamespace = tag->GetProperty(XML_PROPERTY_LINKPARAM_NAMESPACE , "rosie_arm_navigation_tilt_laser_self_filter");
  ros::NodeHandle nh(m_paramNamespace);
  m_self_filter = new filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ> >(nh);

}

cop::Reading*	RangeSensor::GetReading(const long &Frame)
{
  if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
  {
    if(m_grabbing)
    {
      while(!g_stopall && m_grabbing && (m_images.size() == 0))
      {
        printf("waiting for the camera %s to start grabbing\n", GetSensorID().c_str());
        sleep(1);
      }
      printf("Got a new image: %d\n", (int)m_images.size());
    }
    if(m_images.size() == 0)
    {
      printf("unexpected error\n");
      throw "Asking for images from a camera that has no images";
    }
  }
  if(Frame == -1 || (Frame - m_deletedOffset < 0 && (unsigned)(Frame - m_deletedOffset) >= m_images.size()))
  {
    return GetReading_Lock(m_images.size() -1);
    /*return m_images[m_images.size() -1];*/
  }
  return GetReading_Lock(Frame - m_deletedOffset);
}

bool RangeSensor::CanSee(cop::RelPose &pose) const
{
  double row, col;
  if(this->m_relPose != NULL)
  {
    if(this->m_relPose->m_uniqueID != pose.m_uniqueID)
    {
      Matrix m = pose.GetMatrix(this->m_relPose->m_uniqueID);
      ProjectPoint3DToSensor(m.element(0,3), m.element(1,3), m.element(2,3), row, col);
      printf("Project %f %f %f -> %f %f (< %f && < %f) \n", m.element(0,3), m.element(1,3), m.element(2,3), row, col, m_calibration[6], m_calibration[7]);
      if(row >= 0 && row < m_calibration[7] && col >= 0 && col < m_calibration[6])
      {
        return true;
      }
    }
    else
    {
      return true;
    }
  }
  return false;
}

Reading* RangeSensor::ApplySelfFilter(Reading* read)
{
  /*
  printf("In self filter\n");
  m_self_filter->setSensorFrame(m_relPose->m_mapstring);


  robot_self_filter::SelfMask* mask = m_self_filter->getSelfMask ();

  std::vector<geometry_msgs::Point32>::iterator it = ((SwissRangerReading*)read)->m_image.points.begin();
  mask->assumeFrame(((SwissRangerReading*)read)->m_image.header.frame_id,((SwissRangerReading*)read)->m_image.header.stamp);
  int i = 0, j = 0;
  for (; it != ((SwissRangerReading*)read)->m_image.points.end(); i++)
  {
    btVector3 pt ((*it).x, (*it).y, (*it).z );
    int result = mask->getMaskIntersection(pt);
    if(result == robot_self_filter::OUTSIDE)
    {
      printf("Removeing element %d: result was %d\n", j++, result);
      it = ((SwissRangerReading*)read)->m_image.points.erase(it);
    }
    else
    {
      it++;
    }
    if(it == ((SwissRangerReading*)read)->m_image.points.end())
      break;
  }*/
  return read;
}


void RangeSensor::CallBack(const sensor_msgs::PointCloudConstPtr& cloud)
{
  SwissRangerReading* reading;
  if(m_images.size() > 0)
  {
    if(m_bintegration)
    {
      SwissRangerReading* last_reading = (SwissRangerReading*)GetReading_Lock(m_images.size() - 1);
      reading = new SwissRangerReading(cloud, last_reading, GetRelPose());
      last_reading->Free();
    }
    else
      reading = new SwissRangerReading(cloud, NULL, NULL);
  }
  else
  {
    reading = new SwissRangerReading(cloud, NULL, NULL);
  }

  PushBack(reading);
  while(m_images.size() > m_max_cameraImages)
  {
    if(DeleteReading())
      continue;
    else
    {
      printf("SR: Could not delete an image!");
      break;
    }
  }
}


void RangeSensor::CallBackPCL2(const sensor_msgs::PointCloud2ConstPtr& cloud2)
{
  SwissRangerReading* reading;
  sensor_msgs::PointCloud cloud;
  if(!sensor_msgs::convertPointCloud2ToPointCloud(*cloud2, cloud))
  {
    ROS_ERROR("Conversion of PointCloud to PointCloud2 failed\n");
    throw("Error in point cloud coversion\n");
    return;
  }
  if(m_images.size() > 0)
  {
    if(m_bintegration)
    {
      SwissRangerReading* last_reading = (SwissRangerReading*)GetReading_Lock(m_images.size() - 1);
      reading = new SwissRangerReading(cloud, last_reading, GetRelPose());
      last_reading->Free();
    }
    else
      reading = new SwissRangerReading(cloud, NULL, NULL);
  }
  else
  {
    reading = new SwissRangerReading(cloud, NULL, NULL);
  }

  PushBack(reading);
  while(m_images.size() > m_max_cameraImages)
  {
    if(DeleteReading())
      continue;
    else
    {
      printf("SR: Could not delete an image!");
      break;
    }
  }
}

std::pair<std::string, std::vector<double> > RangeSensor::GetUnformatedCalibrationValues() const
{
  return std::pair<std::string, std::vector<double> >("HALCONCALIB", m_calibration);
}


/**
*    Start
*   @brief overwrite to start up the data reading, is called at least once after creation
*/
bool	RangeSensor::Start()
{
  ros::NodeHandle nh;
  printf("Subscribe to topic %s \n", m_stTopic.c_str());
  if(m_subscribePointCloud2)
  {
    m_cloudSub = nh.subscribe (m_stTopic, 3, &RangeSensor::CallBackPCL2, this);
  }
  else
  {
    m_cloudSub = nh.subscribe (m_stTopic, 3, &RangeSensor::CallBack, this);
  }
  m_grabbing = true;
  return true;
}
/**
*    Start
*   @brief overwrite to stop call this in the destructor if necessary, will be used to shut down cameras
*/
bool	RangeSensor::Stop()
{
  Sensor::Stop();
  m_grabbing = false;
  return true;
  /*TODO unsunscribe*/
}

cop::XMLTag* RangeSensor::Save()
{
  cop::XMLTag* tag = new cop::XMLTag(GetName());
  cop::Sensor::SaveTo(tag);
  tag->AddProperty(XML_PROPERTY_TOPIC, m_stTopic);
  tag->AddProperty(XML_PROPERTY_POINT_CLOUD2, m_subscribePointCloud2 ? 1 : 0);
  return tag;
}
