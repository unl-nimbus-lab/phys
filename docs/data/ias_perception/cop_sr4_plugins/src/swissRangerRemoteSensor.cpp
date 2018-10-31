#include "pluginlib/class_list_macros.h"

#include "RangeSensor.h"
#include "SegmentPrototype.h"


class SwissRangerRemoteSensor : public cop::RangeSensor
{
public:
  virtual std::string GetName() const{return XML_NODE_SWISSRANGER;};
};
/**  Sensor Plugin*/
PLUGINLIB_REGISTER_CLASS(SwissRangerRemoteSensor, SwissRangerRemoteSensor, cop::Sensor)
PLUGINLIB_REGISTER_CLASS(RangeSensor, cop::RangeSensor, cop::Sensor)
/** Cluster Detector Plugin */
//PLUGINLIB_REGISTER_CLASS(ClusterDetector, cop::ClusterDetector, cop::LocateAlgorithm)
/** Cluster Descriptor Plugin */
PLUGINLIB_REGISTER_CLASS(SegmentPrototype, cop::SegmentPrototype, cop::Descriptor)

