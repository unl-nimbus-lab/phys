#ifndef _ROS_hector_uav_msgs_RawRC_h
#define _ROS_hector_uav_msgs_RawRC_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class RawRC : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t status;
      uint8_t channel_length;
      uint16_t st_channel;
      uint16_t * channel;

    RawRC():
      header(),
      status(0),
      channel_length(0), channel(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset++) = channel_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channel_length; i++){
      *(outbuffer + offset + 0) = (this->channel[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->channel[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->channel[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      uint8_t channel_lengthT = *(inbuffer + offset++);
      if(channel_lengthT > channel_length)
        this->channel = (uint16_t*)realloc(this->channel, channel_lengthT * sizeof(uint16_t));
      offset += 3;
      channel_length = channel_lengthT;
      for( uint8_t i = 0; i < channel_length; i++){
      this->st_channel =  ((uint16_t) (*(inbuffer + offset)));
      this->st_channel |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_channel);
        memcpy( &(this->channel[i]), &(this->st_channel), sizeof(uint16_t));
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/RawRC"; };
    const char * getMD5(){ return "f1584488325f747abea0b77036f70e2c"; };

  };

}
#endif