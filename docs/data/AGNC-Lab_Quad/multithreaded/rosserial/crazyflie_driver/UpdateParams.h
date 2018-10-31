#ifndef _ROS_SERVICE_UpdateParams_h
#define _ROS_SERVICE_UpdateParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace crazyflie_driver
{

static const char UPDATEPARAMS[] = "crazyflie_driver/UpdateParams";

  class UpdateParamsRequest : public ros::Msg
  {
    public:
      uint8_t params_length;
      char* st_params;
      char* * params;

    UpdateParamsRequest():
      params_length(0), params(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = params_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < params_length; i++){
      uint32_t length_paramsi = strlen(this->params[i]);
      memcpy(outbuffer + offset, &length_paramsi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->params[i], length_paramsi);
      offset += length_paramsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t params_lengthT = *(inbuffer + offset++);
      if(params_lengthT > params_length)
        this->params = (char**)realloc(this->params, params_lengthT * sizeof(char*));
      offset += 3;
      params_length = params_lengthT;
      for( uint8_t i = 0; i < params_length; i++){
      uint32_t length_st_params;
      memcpy(&length_st_params, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_params; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_params-1]=0;
      this->st_params = (char *)(inbuffer + offset-1);
      offset += length_st_params;
        memcpy( &(this->params[i]), &(this->st_params), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return UPDATEPARAMS; };
    const char * getMD5(){ return "279e0627eb574ffe8968384218434f5f"; };

  };

  class UpdateParamsResponse : public ros::Msg
  {
    public:

    UpdateParamsResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return UPDATEPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class UpdateParams {
    public:
    typedef UpdateParamsRequest Request;
    typedef UpdateParamsResponse Response;
  };

}
#endif
