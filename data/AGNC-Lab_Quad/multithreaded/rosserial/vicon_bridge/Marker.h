#ifndef _ROS_vicon_bridge_Marker_h
#define _ROS_vicon_bridge_Marker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace vicon_bridge
{

  class Marker : public ros::Msg
  {
    public:
      const char* marker_name;
      const char* subject_name;
      const char* segment_name;
      geometry_msgs::Point translation;
      bool occluded;

    Marker():
      marker_name(""),
      subject_name(""),
      segment_name(""),
      translation(),
      occluded(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_marker_name = strlen(this->marker_name);
      memcpy(outbuffer + offset, &length_marker_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->marker_name, length_marker_name);
      offset += length_marker_name;
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
      offset += this->translation.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_occluded;
      u_occluded.real = this->occluded;
      *(outbuffer + offset + 0) = (u_occluded.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->occluded);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_marker_name;
      memcpy(&length_marker_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_marker_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_marker_name-1]=0;
      this->marker_name = (char *)(inbuffer + offset-1);
      offset += length_marker_name;
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
      offset += this->translation.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_occluded;
      u_occluded.base = 0;
      u_occluded.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->occluded = u_occluded.real;
      offset += sizeof(this->occluded);
     return offset;
    }

    const char * getType(){ return "vicon_bridge/Marker"; };
    const char * getMD5(){ return "da6f93747c24b19a71932ae4b040f489"; };

  };

}
#endif