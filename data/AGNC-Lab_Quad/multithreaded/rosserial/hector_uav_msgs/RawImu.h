#ifndef _ROS_hector_uav_msgs_RawImu_h
#define _ROS_hector_uav_msgs_RawImu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class RawImu : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int16_t angular_velocity[3];
      int16_t linear_acceleration[3];

    RawImu():
      header(),
      angular_velocity(),
      linear_acceleration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_angular_velocityi;
      u_angular_velocityi.real = this->angular_velocity[i];
      *(outbuffer + offset + 0) = (u_angular_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocityi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angular_velocity[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_linear_accelerationi;
      u_linear_accelerationi.real = this->linear_acceleration[i];
      *(outbuffer + offset + 0) = (u_linear_accelerationi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_accelerationi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->linear_acceleration[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_angular_velocityi;
      u_angular_velocityi.base = 0;
      u_angular_velocityi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocityi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angular_velocity[i] = u_angular_velocityi.real;
      offset += sizeof(this->angular_velocity[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_linear_accelerationi;
      u_linear_accelerationi.base = 0;
      u_linear_accelerationi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_accelerationi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->linear_acceleration[i] = u_linear_accelerationi.real;
      offset += sizeof(this->linear_acceleration[i]);
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/RawImu"; };
    const char * getMD5(){ return "398f651a68070a719c7938171d0fcc45"; };

  };

}
#endif