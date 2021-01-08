#ifndef _ROS_SERVICE_Traj_h
#define _ROS_SERVICE_Traj_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "quad_control/trajData.h"

namespace quad_control
{

static const char TRAJ[] = "quad_control/Traj";

  class TrajRequest : public ros::Msg
  {
    public:
      float init_pos;
      float final_pos;
      float init_vel;
      float final_vel;
      float init_acc;
      float final_acc;
      float init_time;
      float final_time;
      float sampling_rate;

    TrajRequest():
      init_pos(0),
      final_pos(0),
      init_vel(0),
      final_vel(0),
      init_acc(0),
      final_acc(0),
      init_time(0),
      final_time(0),
      sampling_rate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_init_pos;
      u_init_pos.real = this->init_pos;
      *(outbuffer + offset + 0) = (u_init_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_init_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_init_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_init_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->init_pos);
      union {
        float real;
        uint32_t base;
      } u_final_pos;
      u_final_pos.real = this->final_pos;
      *(outbuffer + offset + 0) = (u_final_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_final_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_final_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_final_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->final_pos);
      union {
        float real;
        uint32_t base;
      } u_init_vel;
      u_init_vel.real = this->init_vel;
      *(outbuffer + offset + 0) = (u_init_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_init_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_init_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_init_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->init_vel);
      union {
        float real;
        uint32_t base;
      } u_final_vel;
      u_final_vel.real = this->final_vel;
      *(outbuffer + offset + 0) = (u_final_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_final_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_final_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_final_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->final_vel);
      union {
        float real;
        uint32_t base;
      } u_init_acc;
      u_init_acc.real = this->init_acc;
      *(outbuffer + offset + 0) = (u_init_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_init_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_init_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_init_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->init_acc);
      union {
        float real;
        uint32_t base;
      } u_final_acc;
      u_final_acc.real = this->final_acc;
      *(outbuffer + offset + 0) = (u_final_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_final_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_final_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_final_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->final_acc);
      union {
        float real;
        uint32_t base;
      } u_init_time;
      u_init_time.real = this->init_time;
      *(outbuffer + offset + 0) = (u_init_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_init_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_init_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_init_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->init_time);
      union {
        float real;
        uint32_t base;
      } u_final_time;
      u_final_time.real = this->final_time;
      *(outbuffer + offset + 0) = (u_final_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_final_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_final_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_final_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->final_time);
      union {
        float real;
        uint32_t base;
      } u_sampling_rate;
      u_sampling_rate.real = this->sampling_rate;
      *(outbuffer + offset + 0) = (u_sampling_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sampling_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sampling_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sampling_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sampling_rate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_init_pos;
      u_init_pos.base = 0;
      u_init_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_init_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_init_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_init_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->init_pos = u_init_pos.real;
      offset += sizeof(this->init_pos);
      union {
        float real;
        uint32_t base;
      } u_final_pos;
      u_final_pos.base = 0;
      u_final_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_final_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_final_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_final_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->final_pos = u_final_pos.real;
      offset += sizeof(this->final_pos);
      union {
        float real;
        uint32_t base;
      } u_init_vel;
      u_init_vel.base = 0;
      u_init_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_init_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_init_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_init_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->init_vel = u_init_vel.real;
      offset += sizeof(this->init_vel);
      union {
        float real;
        uint32_t base;
      } u_final_vel;
      u_final_vel.base = 0;
      u_final_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_final_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_final_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_final_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->final_vel = u_final_vel.real;
      offset += sizeof(this->final_vel);
      union {
        float real;
        uint32_t base;
      } u_init_acc;
      u_init_acc.base = 0;
      u_init_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_init_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_init_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_init_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->init_acc = u_init_acc.real;
      offset += sizeof(this->init_acc);
      union {
        float real;
        uint32_t base;
      } u_final_acc;
      u_final_acc.base = 0;
      u_final_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_final_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_final_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_final_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->final_acc = u_final_acc.real;
      offset += sizeof(this->final_acc);
      union {
        float real;
        uint32_t base;
      } u_init_time;
      u_init_time.base = 0;
      u_init_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_init_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_init_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_init_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->init_time = u_init_time.real;
      offset += sizeof(this->init_time);
      union {
        float real;
        uint32_t base;
      } u_final_time;
      u_final_time.base = 0;
      u_final_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_final_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_final_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_final_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->final_time = u_final_time.real;
      offset += sizeof(this->final_time);
      union {
        float real;
        uint32_t base;
      } u_sampling_rate;
      u_sampling_rate.base = 0;
      u_sampling_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sampling_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sampling_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sampling_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sampling_rate = u_sampling_rate.real;
      offset += sizeof(this->sampling_rate);
     return offset;
    }

    const char * getType(){ return TRAJ; };
    const char * getMD5(){ return "025e67c629c573d32982a58fcfdb24b6"; };

  };

  class TrajResponse : public ros::Msg
  {
    public:
      uint8_t trajectory_length;
      quad_control::trajData st_trajectory;
      quad_control::trajData * trajectory;

    TrajResponse():
      trajectory_length(0), trajectory(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = trajectory_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < trajectory_length; i++){
      offset += this->trajectory[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t trajectory_lengthT = *(inbuffer + offset++);
      if(trajectory_lengthT > trajectory_length)
        this->trajectory = (quad_control::trajData*)realloc(this->trajectory, trajectory_lengthT * sizeof(quad_control::trajData));
      offset += 3;
      trajectory_length = trajectory_lengthT;
      for( uint8_t i = 0; i < trajectory_length; i++){
      offset += this->st_trajectory.deserialize(inbuffer + offset);
        memcpy( &(this->trajectory[i]), &(this->st_trajectory), sizeof(quad_control::trajData));
      }
     return offset;
    }

    const char * getType(){ return TRAJ; };
    const char * getMD5(){ return "6800fddd4bf810897f1a5feb2ba6d4b2"; };

  };

  class Traj {
    public:
    typedef TrajRequest Request;
    typedef TrajResponse Response;
  };

}
#endif
