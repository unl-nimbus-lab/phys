#ifndef SWISSRANGERREADING_H
#define SWISSRANGERREADING_H

#define XML_NODE_SWISSRANGER   "SwissRangerRemoteSensor"
#define XML_PROPERTY_TOPIC     "Topic"

#include <sensor_msgs/PointCloud.h>
//#include <point_cloud_mapping/cloud_io.h>
#include <sstream>

#include <XMLTag.h>

extern bool s_initedPCDPublisher;
extern ros::Publisher s_sensorPublisher;


class SwissRangerReading : public cop::Reading
{
public:
  SwissRangerReading(const sensor_msgs::PointCloudConstPtr& pcdcloud, SwissRangerReading* integrate_with, cop::RelPose* parent) :
      Reading(ReadingType_PointCloud),
      m_integrationcounter(0)
  {
      if(parent != NULL)
        SetPose(parent);
      if(integrate_with != NULL && m_relPose != NULL && integrate_with->m_relPose != NULL)
      {
        double  dist = m_relPose->DistanceTo(integrate_with->m_relPose->m_uniqueID);
        if(dist < 0.0005)
        {
          IntegratePointCloud( *pcdcloud, integrate_with->m_image, integrate_with->m_integrationcounter);
        }
        else
        {
          m_image = (*pcdcloud);
        }
      }
      else
      {
        m_image = (*pcdcloud);
      }
      /*if(s_initedPCDPublisher)
        s_sensorPublisher.publish(m_image);*/
  }

SwissRangerReading(const sensor_msgs::PointCloud& pcdcloud, SwissRangerReading* integrate_with, cop::RelPose* parent) :
      Reading(ReadingType_PointCloud),
      m_integrationcounter(0)
  {
      if(parent != NULL)
        SetPose(parent);
      if(integrate_with != NULL && m_relPose != NULL && integrate_with->m_relPose != NULL)
      {
        double  dist = m_relPose->DistanceTo(integrate_with->m_relPose->m_uniqueID);
        if(dist < 0.0005)
        {
          IntegratePointCloud( pcdcloud, integrate_with->m_image, integrate_with->m_integrationcounter);
        }
        else
        {
          m_image = (pcdcloud);
        }
      }
      else
      {
        m_image = (pcdcloud);
      }
      /*if(s_initedPCDPublisher)
        s_sensorPublisher.publish(m_image);*/
  }

  SwissRangerReading() :
      Reading(ReadingType_PointCloud),
      m_integrationcounter(0)
  {
  }
#define min_l(A, B) (((A) < (B)) ? (A) : (B))
  void IntegratePointCloud (const sensor_msgs::PointCloud& p_new, const sensor_msgs::PointCloud p_old, int count_old)
  {
    /** lets assume an ordered point cloud
        the origin

    */
    double weight_new = (10.0 - min_l(count_old, 9.0)) / 10.0;
    double weight_old = 1.0  -  weight_new;
    //printf("weight new %f, weight old %f\n", weight_new, weight_old);
    if(p_new.points.size() == p_old.points.size() && p_new.channels.size() > 0 &&
       p_old.channels.size() > 0 && p_new.channels[0].values.size() ==
         p_old.channels[0].values.size())
    {
      m_image.header = p_new.header;
      m_image.points.resize(p_new.points.size());
      m_image.channels.resize(1);
      m_image.channels[0].values.resize( p_new.channels[0].values.size());
      for(size_t i = 0; i < p_new.points.size(); i++)
      {
        if( p_new.points[i].z != p_new.points[i].z ||  p_old.points[i].z !=  p_old.points[i].z)
        {
          if(p_new.points[i].z == p_new.points[i].z)
          {
            m_image.points[i].x = p_new.points[i].x;
            m_image.points[i].y = p_new.points[i].y;
            m_image.points[i].z = p_new.points[i].z;
            m_image.channels[0].values[i] = weight_new * p_new.channels[0].values[i] + weight_old * p_old.channels[0].values[i];
          }
          else if(p_old.points[i].z == p_old.points[i].z)
          {
            m_image.points[i].x = p_old.points[i].x;
            m_image.points[i].y = p_old.points[i].y;
            m_image.points[i].z = p_old.points[i].z;
            m_image.channels[0].values[i] = weight_new * p_new.channels[0].values[i] + weight_old * p_old.channels[0].values[i];
          }
          else
          {
            m_image.points[i].x = 0.0;
            m_image.points[i].y = 0.0;
            m_image.points[i].z = 0.0;
            m_image.channels[0].values[i] = weight_new * p_new.channels[0].values[i] + weight_old * p_old.channels[0].values[i];
          }
        }
        else
        {
          m_image.points[i].x = weight_new * p_new.points[i].x + weight_old * p_old.points[i].x;
          m_image.points[i].y = weight_new * p_new.points[i].y + weight_old * p_old.points[i].y;
          m_image.points[i].z = weight_new * p_new.points[i].z + weight_old * p_old.points[i].z;
          m_image.channels[0].values[i] = weight_new * p_new.channels[0].values[i] + weight_old * p_old.channels[0].values[i];
        }
      }
      m_integrationcounter = count_old + 1;
    }
    else
    {
      m_integrationcounter = 1;
    }
  }
  ~SwissRangerReading()
  {
  }

 /**
   *   Save the image in a file and put a string in the xml
   */
  virtual cop::XMLTag* Save()
  {
    using namespace cop;
    /*TODO: Save to file and save filename to an XMLTag*/
    XMLTag* tag = new XMLTag("PointCloud");

    std::ostringstream os;
    os << "pcd_"<< tag->date() << ".pcd";
    tag->AddProperty("FileName", os.str());
    //cloud_io::savePCDFile (os.str().c_str(), m_image);
    //TODO

    return  tag;
  }
  /**
   *   Copy the image
   */
  virtual cop::Reading* Clone()
  {
    /*TODO: Copy m_pcd and create new reading*/
    return  NULL;
  }
  sensor_msgs::PointCloud m_image;
  int m_integrationcounter;
};

#endif /**SWISSRANGERREADING_H*/
