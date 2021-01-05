#ifndef _ROS_crazyflie_driver_LogBlock_h
#define _ROS_crazyflie_driver_LogBlock_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace crazyflie_driver
{

  class LogBlock : public ros::Msg
  {
    public:
      const char* topic_name;
      int16_t frequency;
      uint8_t variables_length;
      char* st_variables;
      char* * variables;

    LogBlock():
      topic_name(""),
      frequency(0),
      variables_length(0), variables(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_topic_name = strlen(this->topic_name);
      memcpy(outbuffer + offset, &length_topic_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->topic_name, length_topic_name);
      offset += length_topic_name;
      union {
        int16_t real;
        uint16_t base;
      } u_frequency;
      u_frequency.real = this->frequency;
      *(outbuffer + offset + 0) = (u_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frequency.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->frequency);
      *(outbuffer + offset++) = variables_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < variables_length; i++){
      uint32_t length_variablesi = strlen(this->variables[i]);
      memcpy(outbuffer + offset, &length_variablesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->variables[i], length_variablesi);
      offset += length_variablesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_topic_name;
      memcpy(&length_topic_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic_name-1]=0;
      this->topic_name = (char *)(inbuffer + offset-1);
      offset += length_topic_name;
      union {
        int16_t real;
        uint16_t base;
      } u_frequency;
      u_frequency.base = 0;
      u_frequency.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frequency.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->frequency = u_frequency.real;
      offset += sizeof(this->frequency);
      uint8_t variables_lengthT = *(inbuffer + offset++);
      if(variables_lengthT > variables_length)
        this->variables = (char**)realloc(this->variables, variables_lengthT * sizeof(char*));
      offset += 3;
      variables_length = variables_lengthT;
      for( uint8_t i = 0; i < variables_length; i++){
      uint32_t length_st_variables;
      memcpy(&length_st_variables, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_variables; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_variables-1]=0;
      this->st_variables = (char *)(inbuffer + offset-1);
      offset += length_st_variables;
        memcpy( &(this->variables[i]), &(this->st_variables), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "crazyflie_driver/LogBlock"; };
    const char * getMD5(){ return "d9325f33ff3a1ffc6b6c0323a9f9b181"; };

  };

}
#endif