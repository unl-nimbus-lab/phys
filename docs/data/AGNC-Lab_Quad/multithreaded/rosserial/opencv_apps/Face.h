#ifndef _ROS_opencv_apps_Face_h
#define _ROS_opencv_apps_Face_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Rect.h"

namespace opencv_apps
{

  class Face : public ros::Msg
  {
    public:
      opencv_apps::Rect face;
      uint32_t eyes_length;
      opencv_apps::Rect st_eyes;
      opencv_apps::Rect * eyes;

    Face():
      face(),
      eyes_length(0), eyes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->face.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->eyes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->eyes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->eyes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->eyes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->eyes_length);
      for( uint32_t i = 0; i < eyes_length; i++){
      offset += this->eyes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->face.deserialize(inbuffer + offset);
      uint32_t eyes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      eyes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      eyes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      eyes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->eyes_length);
      if(eyes_lengthT > eyes_length)
        this->eyes = (opencv_apps::Rect*)realloc(this->eyes, eyes_lengthT * sizeof(opencv_apps::Rect));
      eyes_length = eyes_lengthT;
      for( uint32_t i = 0; i < eyes_length; i++){
      offset += this->st_eyes.deserialize(inbuffer + offset);
        memcpy( &(this->eyes[i]), &(this->st_eyes), sizeof(opencv_apps::Rect));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/Face"; };
    const char * getMD5(){ return "0c2547d2eaf71219898bf5c25e36907e"; };

  };

}
#endif