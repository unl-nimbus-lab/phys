#ifndef RANGESENSOR_H
#define RANGESENSOR_H
#include "pluginlib/class_list_macros.h"

#include "Sensor.h"
#include "XMLTag.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "SwissRangerReading.h"

#define XML_NODE_RANGESENSOR "RangeSensor"


#include <robot_self_filter/self_see_filter.h>


namespace cop
{
  class RangeSensor : public cop::Sensor
  {
  public:
    RangeSensor();
    ~RangeSensor();

    /**
    *  This can be overwritten to get the data necessary to reconstruct a saved reading
    */
    virtual void SetData(cop::XMLTag* tag);
    /**
    *  Get Type of the camera by its Name
    */
    virtual std::string GetName() const{return XML_NODE_RANGESENSOR;};

    /**
    * GetReading
    * @param Frame frame number, to specify an offset or a specific file
    * @throws char* with an error message in case of failure
    */
    virtual cop::Reading*	GetReading(const long &Frame = -1);

    /**
    * CanSee
    * Checks if a pose is inside the view of this sensor
    * @param pose pose that has to be looked at
    */
    virtual bool	CanSee(cop::RelPose &pose) const;

    void CallBack(const sensor_msgs::PointCloudConstPtr& cloud);

    void CallBackPCL2(const sensor_msgs::PointCloud2ConstPtr& cloud2);

    virtual std::pair<std::string, std::vector<double> > GetUnformatedCalibrationValues() const;


    /**
    *    Start
    *   @brief overwrite to start up the data reading, is called at least once after creation
    */
    virtual bool	Start();
    /**
    *    Start
    *   @brief overwrite to stop call this in the destructor if necessary, will be used to shut down cameras
    */
    virtual bool	Stop();
    /**
     *   @return the pose of this sensor
     */
    cop::RelPose* GetRelPose(){return m_relPose;}

    virtual cop::XMLTag* Save();
    /**
    *   Can this Sensor be used like a camera, (incl. Calibration, Showing, usw.)
    */
    virtual bool IsCamera(){return false;}

    std::vector<double> m_calibration;


    virtual Reading* ApplySelfFilter(Reading* read);
  private:
      std::string m_stTopic;
      ros::Subscriber m_cloudSub;
      bool m_grabbing;
      bool  m_bintegration;
      bool m_subscribePointCloud2;
      filters::SelfFilter<pcl::PointCloud < pcl::PointXYZ> > *m_self_filter;
      std::string m_paramNamespace;
  };
}
#endif
