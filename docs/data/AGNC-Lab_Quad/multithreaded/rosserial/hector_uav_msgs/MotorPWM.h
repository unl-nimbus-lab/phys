#ifndef _ROS_hector_uav_msgs_MotorPWM_h
#define _ROS_hector_uav_msgs_MotorPWM_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class MotorPWM : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t pwm_length;
      uint8_t st_pwm;
      uint8_t * pwm;

    MotorPWM():
      header(),
      pwm_length(0), pwm(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = pwm_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < pwm_length; i++){
      *(outbuffer + offset + 0) = (this->pwm[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pwm[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t pwm_lengthT = *(inbuffer + offset++);
      if(pwm_lengthT > pwm_length)
        this->pwm = (uint8_t*)realloc(this->pwm, pwm_lengthT * sizeof(uint8_t));
      offset += 3;
      pwm_length = pwm_lengthT;
      for( uint8_t i = 0; i < pwm_length; i++){
      this->st_pwm =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_pwm);
        memcpy( &(this->pwm[i]), &(this->st_pwm), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/MotorPWM"; };
    const char * getMD5(){ return "42f78dd80f99e0208248b8a257b8a645"; };

  };

}
#endif