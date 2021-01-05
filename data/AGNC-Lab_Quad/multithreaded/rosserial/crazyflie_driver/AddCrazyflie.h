#ifndef _ROS_SERVICE_AddCrazyflie_h
#define _ROS_SERVICE_AddCrazyflie_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "crazyflie_driver/LogBlock.h"

namespace crazyflie_driver
{

static const char ADDCRAZYFLIE[] = "crazyflie_driver/AddCrazyflie";

  class AddCrazyflieRequest : public ros::Msg
  {
    public:
      const char* uri;
      const char* tf_prefix;
      float roll_trim;
      float pitch_trim;
      bool enable_logging;
      bool enable_parameters;
      uint8_t log_blocks_length;
      crazyflie_driver::LogBlock st_log_blocks;
      crazyflie_driver::LogBlock * log_blocks;

    AddCrazyflieRequest():
      uri(""),
      tf_prefix(""),
      roll_trim(0),
      pitch_trim(0),
      enable_logging(0),
      enable_parameters(0),
      log_blocks_length(0), log_blocks(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_uri = strlen(this->uri);
      memcpy(outbuffer + offset, &length_uri, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->uri, length_uri);
      offset += length_uri;
      uint32_t length_tf_prefix = strlen(this->tf_prefix);
      memcpy(outbuffer + offset, &length_tf_prefix, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->tf_prefix, length_tf_prefix);
      offset += length_tf_prefix;
      union {
        float real;
        uint32_t base;
      } u_roll_trim;
      u_roll_trim.real = this->roll_trim;
      *(outbuffer + offset + 0) = (u_roll_trim.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_trim.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_trim.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_trim.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_trim);
      union {
        float real;
        uint32_t base;
      } u_pitch_trim;
      u_pitch_trim.real = this->pitch_trim;
      *(outbuffer + offset + 0) = (u_pitch_trim.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_trim.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_trim.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_trim.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_trim);
      union {
        bool real;
        uint8_t base;
      } u_enable_logging;
      u_enable_logging.real = this->enable_logging;
      *(outbuffer + offset + 0) = (u_enable_logging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable_logging);
      union {
        bool real;
        uint8_t base;
      } u_enable_parameters;
      u_enable_parameters.real = this->enable_parameters;
      *(outbuffer + offset + 0) = (u_enable_parameters.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable_parameters);
      *(outbuffer + offset++) = log_blocks_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < log_blocks_length; i++){
      offset += this->log_blocks[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_uri;
      memcpy(&length_uri, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uri-1]=0;
      this->uri = (char *)(inbuffer + offset-1);
      offset += length_uri;
      uint32_t length_tf_prefix;
      memcpy(&length_tf_prefix, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tf_prefix; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tf_prefix-1]=0;
      this->tf_prefix = (char *)(inbuffer + offset-1);
      offset += length_tf_prefix;
      union {
        float real;
        uint32_t base;
      } u_roll_trim;
      u_roll_trim.base = 0;
      u_roll_trim.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_trim.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_trim.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_trim.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_trim = u_roll_trim.real;
      offset += sizeof(this->roll_trim);
      union {
        float real;
        uint32_t base;
      } u_pitch_trim;
      u_pitch_trim.base = 0;
      u_pitch_trim.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_trim.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_trim.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_trim.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch_trim = u_pitch_trim.real;
      offset += sizeof(this->pitch_trim);
      union {
        bool real;
        uint8_t base;
      } u_enable_logging;
      u_enable_logging.base = 0;
      u_enable_logging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable_logging = u_enable_logging.real;
      offset += sizeof(this->enable_logging);
      union {
        bool real;
        uint8_t base;
      } u_enable_parameters;
      u_enable_parameters.base = 0;
      u_enable_parameters.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable_parameters = u_enable_parameters.real;
      offset += sizeof(this->enable_parameters);
      uint8_t log_blocks_lengthT = *(inbuffer + offset++);
      if(log_blocks_lengthT > log_blocks_length)
        this->log_blocks = (crazyflie_driver::LogBlock*)realloc(this->log_blocks, log_blocks_lengthT * sizeof(crazyflie_driver::LogBlock));
      offset += 3;
      log_blocks_length = log_blocks_lengthT;
      for( uint8_t i = 0; i < log_blocks_length; i++){
      offset += this->st_log_blocks.deserialize(inbuffer + offset);
        memcpy( &(this->log_blocks[i]), &(this->st_log_blocks), sizeof(crazyflie_driver::LogBlock));
      }
     return offset;
    }

    const char * getType(){ return ADDCRAZYFLIE; };
    const char * getMD5(){ return "281517a9f976c165030a3b33c63a9478"; };

  };

  class AddCrazyflieResponse : public ros::Msg
  {
    public:

    AddCrazyflieResponse()
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

    const char * getType(){ return ADDCRAZYFLIE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AddCrazyflie {
    public:
    typedef AddCrazyflieRequest Request;
    typedef AddCrazyflieResponse Response;
  };

}
#endif
