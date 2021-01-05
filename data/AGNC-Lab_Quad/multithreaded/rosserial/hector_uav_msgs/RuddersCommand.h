#ifndef _ROS_hector_uav_msgs_RuddersCommand_h
#define _ROS_hector_uav_msgs_RuddersCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class RuddersCommand : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float aileron;
      float elevator;
      float rudder;

    RuddersCommand():
      header(),
      aileron(0),
      elevator(0),
      rudder(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_aileron;
      u_aileron.real = this->aileron;
      *(outbuffer + offset + 0) = (u_aileron.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_aileron.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_aileron.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_aileron.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->aileron);
      union {
        float real;
        uint32_t base;
      } u_elevator;
      u_elevator.real = this->elevator;
      *(outbuffer + offset + 0) = (u_elevator.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevator.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevator.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevator.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->elevator);
      union {
        float real;
        uint32_t base;
      } u_rudder;
      u_rudder.real = this->rudder;
      *(outbuffer + offset + 0) = (u_rudder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rudder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rudder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rudder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rudder);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_aileron;
      u_aileron.base = 0;
      u_aileron.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_aileron.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_aileron.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_aileron.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->aileron = u_aileron.real;
      offset += sizeof(this->aileron);
      union {
        float real;
        uint32_t base;
      } u_elevator;
      u_elevator.base = 0;
      u_elevator.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevator.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevator.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevator.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->elevator = u_elevator.real;
      offset += sizeof(this->elevator);
      union {
        float real;
        uint32_t base;
      } u_rudder;
      u_rudder.base = 0;
      u_rudder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rudder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rudder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rudder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rudder = u_rudder.real;
      offset += sizeof(this->rudder);
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/RuddersCommand"; };
    const char * getMD5(){ return "2e136cb8cfffc2233e404b320c27bca6"; };

  };

}
#endif