#ifndef _ROS_quad_control_trajDataArray_h
#define _ROS_quad_control_trajDataArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "quad_control/trajData.h"

namespace quad_control
{

  class trajDataArray : public ros::Msg
  {
    public:
      uint8_t trajectory_length;
      quad_control::trajData st_trajectory;
      quad_control::trajData * trajectory;

    trajDataArray():
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

    const char * getType(){ return "quad_control/trajDataArray"; };
    const char * getMD5(){ return "6800fddd4bf810897f1a5feb2ba6d4b2"; };

  };

}
#endif