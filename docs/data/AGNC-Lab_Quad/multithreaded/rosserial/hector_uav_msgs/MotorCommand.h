#ifndef _ROS_hector_uav_msgs_MotorCommand_h
#define _ROS_hector_uav_msgs_MotorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class MotorCommand : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t force_length;
      float st_force;
      float * force;
      uint8_t torque_length;
      float st_torque;
      float * torque;
      uint8_t frequency_length;
      float st_frequency;
      float * frequency;
      uint8_t voltage_length;
      float st_voltage;
      float * voltage;

    MotorCommand():
      header(),
      force_length(0), force(NULL),
      torque_length(0), torque(NULL),
      frequency_length(0), frequency(NULL),
      voltage_length(0), voltage(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = force_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < force_length; i++){
      union {
        float real;
        uint32_t base;
      } u_forcei;
      u_forcei.real = this->force[i];
      *(outbuffer + offset + 0) = (u_forcei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_forcei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_forcei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_forcei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force[i]);
      }
      *(outbuffer + offset++) = torque_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < torque_length; i++){
      union {
        float real;
        uint32_t base;
      } u_torquei;
      u_torquei.real = this->torque[i];
      *(outbuffer + offset + 0) = (u_torquei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torquei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torquei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torquei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torque[i]);
      }
      *(outbuffer + offset++) = frequency_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < frequency_length; i++){
      union {
        float real;
        uint32_t base;
      } u_frequencyi;
      u_frequencyi.real = this->frequency[i];
      *(outbuffer + offset + 0) = (u_frequencyi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frequencyi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frequencyi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frequencyi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frequency[i]);
      }
      *(outbuffer + offset++) = voltage_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < voltage_length; i++){
      union {
        float real;
        uint32_t base;
      } u_voltagei;
      u_voltagei.real = this->voltage[i];
      *(outbuffer + offset + 0) = (u_voltagei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltagei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltagei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltagei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t force_lengthT = *(inbuffer + offset++);
      if(force_lengthT > force_length)
        this->force = (float*)realloc(this->force, force_lengthT * sizeof(float));
      offset += 3;
      force_length = force_lengthT;
      for( uint8_t i = 0; i < force_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_force;
      u_st_force.base = 0;
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_force.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_force = u_st_force.real;
      offset += sizeof(this->st_force);
        memcpy( &(this->force[i]), &(this->st_force), sizeof(float));
      }
      uint8_t torque_lengthT = *(inbuffer + offset++);
      if(torque_lengthT > torque_length)
        this->torque = (float*)realloc(this->torque, torque_lengthT * sizeof(float));
      offset += 3;
      torque_length = torque_lengthT;
      for( uint8_t i = 0; i < torque_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_torque;
      u_st_torque.base = 0;
      u_st_torque.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_torque.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_torque.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_torque.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_torque = u_st_torque.real;
      offset += sizeof(this->st_torque);
        memcpy( &(this->torque[i]), &(this->st_torque), sizeof(float));
      }
      uint8_t frequency_lengthT = *(inbuffer + offset++);
      if(frequency_lengthT > frequency_length)
        this->frequency = (float*)realloc(this->frequency, frequency_lengthT * sizeof(float));
      offset += 3;
      frequency_length = frequency_lengthT;
      for( uint8_t i = 0; i < frequency_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_frequency;
      u_st_frequency.base = 0;
      u_st_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_frequency = u_st_frequency.real;
      offset += sizeof(this->st_frequency);
        memcpy( &(this->frequency[i]), &(this->st_frequency), sizeof(float));
      }
      uint8_t voltage_lengthT = *(inbuffer + offset++);
      if(voltage_lengthT > voltage_length)
        this->voltage = (float*)realloc(this->voltage, voltage_lengthT * sizeof(float));
      offset += 3;
      voltage_length = voltage_lengthT;
      for( uint8_t i = 0; i < voltage_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_voltage;
      u_st_voltage.base = 0;
      u_st_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_voltage = u_st_voltage.real;
      offset += sizeof(this->st_voltage);
        memcpy( &(this->voltage[i]), &(this->st_voltage), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/MotorCommand"; };
    const char * getMD5(){ return "ccd4d4d4606731d1c73409e9bfa55808"; };

  };

}
#endif