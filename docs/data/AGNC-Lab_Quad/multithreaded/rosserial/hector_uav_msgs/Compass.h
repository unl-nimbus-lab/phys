#ifndef _ROS_hector_uav_msgs_Compass_h
#define _ROS_hector_uav_msgs_Compass_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class Compass : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float magnetic_heading;
      float declination;

    Compass():
      header(),
      magnetic_heading(0),
      declination(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_magnetic_heading;
      u_magnetic_heading.real = this->magnetic_heading;
      *(outbuffer + offset + 0) = (u_magnetic_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_magnetic_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_magnetic_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_magnetic_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->magnetic_heading);
      union {
        float real;
        uint32_t base;
      } u_declination;
      u_declination.real = this->declination;
      *(outbuffer + offset + 0) = (u_declination.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_declination.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_declination.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_declination.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->declination);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_magnetic_heading;
      u_magnetic_heading.base = 0;
      u_magnetic_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_magnetic_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_magnetic_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_magnetic_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->magnetic_heading = u_magnetic_heading.real;
      offset += sizeof(this->magnetic_heading);
      union {
        float real;
        uint32_t base;
      } u_declination;
      u_declination.base = 0;
      u_declination.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_declination.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_declination.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_declination.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->declination = u_declination.real;
      offset += sizeof(this->declination);
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/Compass"; };
    const char * getMD5(){ return "69b5db73a2f794a5a815baf6b84a4be5"; };

  };

}
#endif