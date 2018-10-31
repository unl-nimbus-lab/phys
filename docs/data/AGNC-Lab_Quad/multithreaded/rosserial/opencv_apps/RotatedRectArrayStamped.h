#ifndef _ROS_opencv_apps_RotatedRectArrayStamped_h
#define _ROS_opencv_apps_RotatedRectArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/RotatedRect.h"

namespace opencv_apps
{

  class RotatedRectArrayStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t rects_length;
      opencv_apps::RotatedRect st_rects;
      opencv_apps::RotatedRect * rects;

    RotatedRectArrayStamped():
      header(),
      rects_length(0), rects(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->rects_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rects_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rects_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rects_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rects_length);
      for( uint32_t i = 0; i < rects_length; i++){
      offset += this->rects[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t rects_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rects_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rects_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rects_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rects_length);
      if(rects_lengthT > rects_length)
        this->rects = (opencv_apps::RotatedRect*)realloc(this->rects, rects_lengthT * sizeof(opencv_apps::RotatedRect));
      rects_length = rects_lengthT;
      for( uint32_t i = 0; i < rects_length; i++){
      offset += this->st_rects.deserialize(inbuffer + offset);
        memcpy( &(this->rects[i]), &(this->st_rects), sizeof(opencv_apps::RotatedRect));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/RotatedRectArrayStamped"; };
    const char * getMD5(){ return "89a2d4a7db2d2945ca46c25a3bd8c7c5"; };

  };

}
#endif