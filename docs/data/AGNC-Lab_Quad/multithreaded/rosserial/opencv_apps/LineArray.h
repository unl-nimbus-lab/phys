#ifndef _ROS_opencv_apps_LineArray_h
#define _ROS_opencv_apps_LineArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Line.h"

namespace opencv_apps
{

  class LineArray : public ros::Msg
  {
    public:
      uint32_t lines_length;
      opencv_apps::Line st_lines;
      opencv_apps::Line * lines;

    LineArray():
      lines_length(0), lines(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->lines_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lines_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lines_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lines_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lines_length);
      for( uint32_t i = 0; i < lines_length; i++){
      offset += this->lines[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t lines_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      lines_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      lines_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      lines_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->lines_length);
      if(lines_lengthT > lines_length)
        this->lines = (opencv_apps::Line*)realloc(this->lines, lines_lengthT * sizeof(opencv_apps::Line));
      lines_length = lines_lengthT;
      for( uint32_t i = 0; i < lines_length; i++){
      offset += this->st_lines.deserialize(inbuffer + offset);
        memcpy( &(this->lines[i]), &(this->st_lines), sizeof(opencv_apps::Line));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/LineArray"; };
    const char * getMD5(){ return "2b5441933900cc71528395dda29124da"; };

  };

}
#endif