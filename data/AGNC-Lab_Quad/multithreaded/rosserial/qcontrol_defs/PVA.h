#ifndef _ROS_qcontrol_defs_PVA_h
#define _ROS_qcontrol_defs_PVA_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"

namespace qcontrol_defs
{

  class PVA : public ros::Msg
  {
    public:
      ros::Time t;
      geometry_msgs::Pose pos;
      geometry_msgs::Twist vel;
      geometry_msgs::Accel acc;

    PVA():
      t(),
      pos(),
      vel(),
      acc()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->t.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->t.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->t.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t.sec);
      *(outbuffer + offset + 0) = (this->t.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->t.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->t.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t.nsec);
      offset += this->pos.serialize(outbuffer + offset);
      offset += this->vel.serialize(outbuffer + offset);
      offset += this->acc.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->t.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->t.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->t.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->t.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->t.sec);
      this->t.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->t.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->t.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->t.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->t.nsec);
      offset += this->pos.deserialize(inbuffer + offset);
      offset += this->vel.deserialize(inbuffer + offset);
      offset += this->acc.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "qcontrol_defs/PVA"; };
    const char * getMD5(){ return "989591cd271e7e535440ee0095394efd"; };

  };

}
#endif