#ifndef _ROS_SERVICE_viconGrabPose_h
#define _ROS_SERVICE_viconGrabPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace vicon_bridge
{

static const char VICONGRABPOSE[] = "vicon_bridge/viconGrabPose";

  class viconGrabPoseRequest : public ros::Msg
  {
    public:
      const char* subject_name;
      const char* segment_name;
      int32_t n_measurements;

    viconGrabPoseRequest():
      subject_name(""),
      segment_name(""),
      n_measurements(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_subject_name = strlen(this->subject_name);
      memcpy(outbuffer + offset, &length_subject_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->subject_name, length_subject_name);
      offset += length_subject_name;
      uint32_t length_segment_name = strlen(this->segment_name);
      memcpy(outbuffer + offset, &length_segment_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->segment_name, length_segment_name);
      offset += length_segment_name;
      union {
        int32_t real;
        uint32_t base;
      } u_n_measurements;
      u_n_measurements.real = this->n_measurements;
      *(outbuffer + offset + 0) = (u_n_measurements.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_n_measurements.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_n_measurements.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_n_measurements.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->n_measurements);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_subject_name;
      memcpy(&length_subject_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_subject_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_subject_name-1]=0;
      this->subject_name = (char *)(inbuffer + offset-1);
      offset += length_subject_name;
      uint32_t length_segment_name;
      memcpy(&length_segment_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_segment_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_segment_name-1]=0;
      this->segment_name = (char *)(inbuffer + offset-1);
      offset += length_segment_name;
      union {
        int32_t real;
        uint32_t base;
      } u_n_measurements;
      u_n_measurements.base = 0;
      u_n_measurements.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_n_measurements.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_n_measurements.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_n_measurements.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->n_measurements = u_n_measurements.real;
      offset += sizeof(this->n_measurements);
     return offset;
    }

    const char * getType(){ return VICONGRABPOSE; };
    const char * getMD5(){ return "4045133337c2e7a711effc5b44dfbbb6"; };

  };

  class viconGrabPoseResponse : public ros::Msg
  {
    public:
      bool success;
      geometry_msgs::PoseStamped pose;

    viconGrabPoseResponse():
      success(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return VICONGRABPOSE; };
    const char * getMD5(){ return "236213ed6979c1ab1c49bd1bc04ace9e"; };

  };

  class viconGrabPose {
    public:
    typedef viconGrabPoseRequest Request;
    typedef viconGrabPoseResponse Response;
  };

}
#endif
