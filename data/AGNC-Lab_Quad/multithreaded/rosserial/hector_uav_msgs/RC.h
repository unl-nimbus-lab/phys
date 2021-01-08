#ifndef _ROS_hector_uav_msgs_RC_h
#define _ROS_hector_uav_msgs_RC_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class RC : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t status;
      bool valid;
      uint8_t axis_length;
      float st_axis;
      float * axis;
      uint8_t axis_function_length;
      uint8_t st_axis_function;
      uint8_t * axis_function;
      uint8_t swit_length;
      int8_t st_swit;
      int8_t * swit;
      uint8_t swit_function_length;
      uint8_t st_swit_function;
      uint8_t * swit_function;
      enum { ROLL =  1 };
      enum { PITCH =  2 };
      enum { YAW =  3 };
      enum { STEER =  4 };
      enum { HEIGHT =  5 };
      enum { THRUST =  6 };
      enum { BRAKE =  7 };

    RC():
      header(),
      status(0),
      valid(0),
      axis_length(0), axis(NULL),
      axis_function_length(0), axis_function(NULL),
      swit_length(0), swit(NULL),
      swit_function_length(0), swit_function(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      union {
        bool real;
        uint8_t base;
      } u_valid;
      u_valid.real = this->valid;
      *(outbuffer + offset + 0) = (u_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valid);
      *(outbuffer + offset++) = axis_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < axis_length; i++){
      union {
        float real;
        uint32_t base;
      } u_axisi;
      u_axisi.real = this->axis[i];
      *(outbuffer + offset + 0) = (u_axisi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_axisi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_axisi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_axisi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->axis[i]);
      }
      *(outbuffer + offset++) = axis_function_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < axis_function_length; i++){
      *(outbuffer + offset + 0) = (this->axis_function[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->axis_function[i]);
      }
      *(outbuffer + offset++) = swit_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < swit_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_switi;
      u_switi.real = this->swit[i];
      *(outbuffer + offset + 0) = (u_switi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->swit[i]);
      }
      *(outbuffer + offset++) = swit_function_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < swit_function_length; i++){
      *(outbuffer + offset + 0) = (this->swit_function[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->swit_function[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      union {
        bool real;
        uint8_t base;
      } u_valid;
      u_valid.base = 0;
      u_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->valid = u_valid.real;
      offset += sizeof(this->valid);
      uint8_t axis_lengthT = *(inbuffer + offset++);
      if(axis_lengthT > axis_length)
        this->axis = (float*)realloc(this->axis, axis_lengthT * sizeof(float));
      offset += 3;
      axis_length = axis_lengthT;
      for( uint8_t i = 0; i < axis_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_axis;
      u_st_axis.base = 0;
      u_st_axis.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_axis.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_axis.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_axis.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_axis = u_st_axis.real;
      offset += sizeof(this->st_axis);
        memcpy( &(this->axis[i]), &(this->st_axis), sizeof(float));
      }
      uint8_t axis_function_lengthT = *(inbuffer + offset++);
      if(axis_function_lengthT > axis_function_length)
        this->axis_function = (uint8_t*)realloc(this->axis_function, axis_function_lengthT * sizeof(uint8_t));
      offset += 3;
      axis_function_length = axis_function_lengthT;
      for( uint8_t i = 0; i < axis_function_length; i++){
      this->st_axis_function =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_axis_function);
        memcpy( &(this->axis_function[i]), &(this->st_axis_function), sizeof(uint8_t));
      }
      uint8_t swit_lengthT = *(inbuffer + offset++);
      if(swit_lengthT > swit_length)
        this->swit = (int8_t*)realloc(this->swit, swit_lengthT * sizeof(int8_t));
      offset += 3;
      swit_length = swit_lengthT;
      for( uint8_t i = 0; i < swit_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_swit;
      u_st_swit.base = 0;
      u_st_swit.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_swit = u_st_swit.real;
      offset += sizeof(this->st_swit);
        memcpy( &(this->swit[i]), &(this->st_swit), sizeof(int8_t));
      }
      uint8_t swit_function_lengthT = *(inbuffer + offset++);
      if(swit_function_lengthT > swit_function_length)
        this->swit_function = (uint8_t*)realloc(this->swit_function, swit_function_lengthT * sizeof(uint8_t));
      offset += 3;
      swit_function_length = swit_function_lengthT;
      for( uint8_t i = 0; i < swit_function_length; i++){
      this->st_swit_function =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_swit_function);
        memcpy( &(this->swit_function[i]), &(this->st_swit_function), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/RC"; };
    const char * getMD5(){ return "2691c2fe8c5ab2323146bdd8dd2e449e"; };

  };

}
#endif