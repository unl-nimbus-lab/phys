#ifndef _ROS_vicon_bridge_Markers_h
#define _ROS_vicon_bridge_Markers_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "vicon_bridge/Marker.h"

namespace vicon_bridge
{

  class Markers : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t frame_number;
      uint8_t markers_length;
      vicon_bridge::Marker st_markers;
      vicon_bridge::Marker * markers;

    Markers():
      header(),
      frame_number(0),
      markers_length(0), markers(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->frame_number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->frame_number >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->frame_number >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->frame_number >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frame_number);
      *(outbuffer + offset++) = markers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->frame_number =  ((uint32_t) (*(inbuffer + offset)));
      this->frame_number |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->frame_number |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->frame_number |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->frame_number);
      uint8_t markers_lengthT = *(inbuffer + offset++);
      if(markers_lengthT > markers_length)
        this->markers = (vicon_bridge::Marker*)realloc(this->markers, markers_lengthT * sizeof(vicon_bridge::Marker));
      offset += 3;
      markers_length = markers_lengthT;
      for( uint8_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(vicon_bridge::Marker));
      }
     return offset;
    }

    const char * getType(){ return "vicon_bridge/Markers"; };
    const char * getMD5(){ return "579f0637989aa8139ce6bf98cf7aabda"; };

  };

}
#endif