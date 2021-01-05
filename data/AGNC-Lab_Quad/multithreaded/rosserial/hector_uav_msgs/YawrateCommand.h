#ifndef _ROS_hector_uav_msgs_YawrateCommand_h
#define _ROS_hector_uav_msgs_YawrateCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class YawrateCommand : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float turnrate;

    YawrateCommand():
      header(),
      turnrate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_turnrate;
      u_turnrate.real = this->turnrate;
      *(outbuffer + offset + 0) = (u_turnrate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_turnrate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_turnrate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_turnrate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->turnrate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_turnrate;
      u_turnrate.base = 0;
      u_turnrate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_turnrate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_turnrate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_turnrate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->turnrate = u_turnrate.real;
      offset += sizeof(this->turnrate);
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/YawrateCommand"; };
    const char * getMD5(){ return "d8d2a0a1e3daa0fc410bf30a154fa6b6"; };

  };

}
#endif