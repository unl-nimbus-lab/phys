#ifndef _ROS_hector_uav_msgs_RawMagnetic_h
#define _ROS_hector_uav_msgs_RawMagnetic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class RawMagnetic : public ros::Msg
  {
    public:
      std_msgs::Header header;
      double channel[3];

    RawMagnetic():
      header(),
      channel()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_channeli;
      u_channeli.real = this->channel[i];
      *(outbuffer + offset + 0) = (u_channeli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_channeli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_channeli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_channeli.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_channeli.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_channeli.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_channeli.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_channeli.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->channel[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_channeli;
      u_channeli.base = 0;
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_channeli.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->channel[i] = u_channeli.real;
      offset += sizeof(this->channel[i]);
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/RawMagnetic"; };
    const char * getMD5(){ return "babd510868ac7b486e2097c79e1384c9"; };

  };

}
#endif