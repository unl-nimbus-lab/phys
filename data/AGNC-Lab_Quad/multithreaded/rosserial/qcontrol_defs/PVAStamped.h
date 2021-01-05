#ifndef _ROS_qcontrol_defs_PVAStamped_h
#define _ROS_qcontrol_defs_PVAStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "qcontrol_defs/PVA.h"

namespace qcontrol_defs
{

  class PVAStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      qcontrol_defs::PVA pva;

    PVAStamped():
      header(),
      pva()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pva.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pva.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "qcontrol_defs/PVAStamped"; };
    const char * getMD5(){ return "3fc3179d946b21030e147ba69679c26d"; };

  };

}
#endif