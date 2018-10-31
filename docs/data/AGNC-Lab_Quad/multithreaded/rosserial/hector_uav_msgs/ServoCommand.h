#ifndef _ROS_hector_uav_msgs_ServoCommand_h
#define _ROS_hector_uav_msgs_ServoCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class ServoCommand : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t value_length;
      uint16_t st_value;
      uint16_t * value;

    ServoCommand():
      header(),
      value_length(0), value(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = value_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < value_length; i++){
      *(outbuffer + offset + 0) = (this->value[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->value[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t value_lengthT = *(inbuffer + offset++);
      if(value_lengthT > value_length)
        this->value = (uint16_t*)realloc(this->value, value_lengthT * sizeof(uint16_t));
      offset += 3;
      value_length = value_lengthT;
      for( uint8_t i = 0; i < value_length; i++){
      this->st_value =  ((uint16_t) (*(inbuffer + offset)));
      this->st_value |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_value);
        memcpy( &(this->value[i]), &(this->st_value), sizeof(uint16_t));
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/ServoCommand"; };
    const char * getMD5(){ return "d60ef35d4e3412dc6686b189be2523d0"; };

  };

}
#endif