#ifndef _ROS_crazyflie_driver_GenericLogData_h
#define _ROS_crazyflie_driver_GenericLogData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace crazyflie_driver
{

  class GenericLogData : public ros::Msg
  {
    public:
      uint8_t values_length;
      double st_values;
      double * values;

    GenericLogData():
      values_length(0), values(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = values_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        double real;
        uint64_t base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valuesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_valuesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_valuesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_valuesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_valuesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t values_lengthT = *(inbuffer + offset++);
      if(values_lengthT > values_length)
        this->values = (double*)realloc(this->values, values_lengthT * sizeof(double));
      offset += 3;
      values_length = values_lengthT;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_values;
      u_st_values.base = 0;
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_values.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_values = u_st_values.real;
      offset += sizeof(this->st_values);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "crazyflie_driver/GenericLogData"; };
    const char * getMD5(){ return "b9163d7c678dcd669317e43e46b63d96"; };

  };

}
#endif