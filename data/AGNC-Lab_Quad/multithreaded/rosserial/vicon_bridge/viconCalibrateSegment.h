#ifndef _ROS_SERVICE_viconCalibrateSegment_h
#define _ROS_SERVICE_viconCalibrateSegment_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace vicon_bridge
{

static const char VICONCALIBRATESEGMENT[] = "vicon_bridge/viconCalibrateSegment";

  class viconCalibrateSegmentRequest : public ros::Msg
  {
    public:
      const char* subject_name;
      const char* segment_name;
      double z_offset;
      int32_t n_measurements;

    viconCalibrateSegmentRequest():
      subject_name(""),
      segment_name(""),
      z_offset(0),
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
        double real;
        uint64_t base;
      } u_z_offset;
      u_z_offset.real = this->z_offset;
      *(outbuffer + offset + 0) = (u_z_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_offset.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_z_offset.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_z_offset.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_z_offset.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_z_offset.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->z_offset);
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
        double real;
        uint64_t base;
      } u_z_offset;
      u_z_offset.base = 0;
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_z_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->z_offset = u_z_offset.real;
      offset += sizeof(this->z_offset);
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

    const char * getType(){ return VICONCALIBRATESEGMENT; };
    const char * getMD5(){ return "f57831d02c84e74975c7663933fe42d8"; };

  };

  class viconCalibrateSegmentResponse : public ros::Msg
  {
    public:
      bool success;
      const char* status;
      geometry_msgs::PoseStamped pose;

    viconCalibrateSegmentResponse():
      success(0),
      status(""),
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
      uint32_t length_status = strlen(this->status);
      memcpy(outbuffer + offset, &length_status, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
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
      uint32_t length_status;
      memcpy(&length_status, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return VICONCALIBRATESEGMENT; };
    const char * getMD5(){ return "fd8b451f9e0c65ec25938e0acbd102d7"; };

  };

  class viconCalibrateSegment {
    public:
    typedef viconCalibrateSegmentRequest Request;
    typedef viconCalibrateSegmentResponse Response;
  };

}
#endif
