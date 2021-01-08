#ifndef _ROS_qcontrol_defs_PVAArray_h
#define _ROS_qcontrol_defs_PVAArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "qcontrol_defs/PVA.h"

namespace qcontrol_defs
{

  class PVAArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t pvas_length;
      qcontrol_defs::PVA st_pvas;
      qcontrol_defs::PVA * pvas;

    PVAArray():
      header(),
      pvas_length(0), pvas(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = pvas_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < pvas_length; i++){
      offset += this->pvas[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t pvas_lengthT = *(inbuffer + offset++);
      if(pvas_lengthT > pvas_length)
        this->pvas = (qcontrol_defs::PVA*)realloc(this->pvas, pvas_lengthT * sizeof(qcontrol_defs::PVA));
      offset += 3;
      pvas_length = pvas_lengthT;
      for( uint8_t i = 0; i < pvas_length; i++){
      offset += this->st_pvas.deserialize(inbuffer + offset);
        memcpy( &(this->pvas[i]), &(this->st_pvas), sizeof(qcontrol_defs::PVA));
      }
     return offset;
    }

    const char * getType(){ return "qcontrol_defs/PVAArray"; };
    const char * getMD5(){ return "91f3a9f2851ea4b885812b0f3339c65e"; };

  };

}
#endif