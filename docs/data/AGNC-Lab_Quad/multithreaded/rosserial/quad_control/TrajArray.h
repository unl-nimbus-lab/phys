#ifndef _ROS_quad_control_TrajArray_h
#define _ROS_quad_control_TrajArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"

namespace quad_control
{

  class TrajArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t velocity_length;
      geometry_msgs::Twist st_velocity;
      geometry_msgs::Twist * velocity;
      uint8_t acceleration_length;
      geometry_msgs::Wrench st_acceleration;
      geometry_msgs::Wrench * acceleration;
      uint8_t time_length;
      double st_time;
      double * time;
      uint8_t position_length;
      geometry_msgs::Pose st_position;
      geometry_msgs::Pose * position;

    TrajArray():
      header(),
      velocity_length(0), velocity(NULL),
      acceleration_length(0), acceleration(NULL),
      time_length(0), time(NULL),
      position_length(0), position(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = velocity_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < velocity_length; i++){
      offset += this->velocity[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = acceleration_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < acceleration_length; i++){
      offset += this->acceleration[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = time_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < time_length; i++){
      union {
        double real;
        uint64_t base;
      } u_timei;
      u_timei.real = this->time[i];
      *(outbuffer + offset + 0) = (u_timei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_timei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_timei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_timei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_timei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time[i]);
      }
      *(outbuffer + offset++) = position_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < position_length; i++){
      offset += this->position[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t velocity_lengthT = *(inbuffer + offset++);
      if(velocity_lengthT > velocity_length)
        this->velocity = (geometry_msgs::Twist*)realloc(this->velocity, velocity_lengthT * sizeof(geometry_msgs::Twist));
      offset += 3;
      velocity_length = velocity_lengthT;
      for( uint8_t i = 0; i < velocity_length; i++){
      offset += this->st_velocity.deserialize(inbuffer + offset);
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(geometry_msgs::Twist));
      }
      uint8_t acceleration_lengthT = *(inbuffer + offset++);
      if(acceleration_lengthT > acceleration_length)
        this->acceleration = (geometry_msgs::Wrench*)realloc(this->acceleration, acceleration_lengthT * sizeof(geometry_msgs::Wrench));
      offset += 3;
      acceleration_length = acceleration_lengthT;
      for( uint8_t i = 0; i < acceleration_length; i++){
      offset += this->st_acceleration.deserialize(inbuffer + offset);
        memcpy( &(this->acceleration[i]), &(this->st_acceleration), sizeof(geometry_msgs::Wrench));
      }
      uint8_t time_lengthT = *(inbuffer + offset++);
      if(time_lengthT > time_length)
        this->time = (double*)realloc(this->time, time_lengthT * sizeof(double));
      offset += 3;
      time_length = time_lengthT;
      for( uint8_t i = 0; i < time_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_time;
      u_st_time.base = 0;
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_time = u_st_time.real;
      offset += sizeof(this->st_time);
        memcpy( &(this->time[i]), &(this->st_time), sizeof(double));
      }
      uint8_t position_lengthT = *(inbuffer + offset++);
      if(position_lengthT > position_length)
        this->position = (geometry_msgs::Pose*)realloc(this->position, position_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      position_length = position_lengthT;
      for( uint8_t i = 0; i < position_length; i++){
      offset += this->st_position.deserialize(inbuffer + offset);
        memcpy( &(this->position[i]), &(this->st_position), sizeof(geometry_msgs::Pose));
      }
     return offset;
    }

    const char * getType(){ return "quad_control/TrajArray"; };
    const char * getMD5(){ return "99f24457a2c25ec6098a54c362a36262"; };

  };

}
#endif