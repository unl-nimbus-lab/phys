#ifndef _ROS_opencv_apps_FlowArray_h
#define _ROS_opencv_apps_FlowArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Flow.h"

namespace opencv_apps
{

  class FlowArray : public ros::Msg
  {
    public:
      uint32_t flow_length;
      opencv_apps::Flow st_flow;
      opencv_apps::Flow * flow;

    FlowArray():
      flow_length(0), flow(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->flow_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flow_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->flow_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->flow_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flow_length);
      for( uint32_t i = 0; i < flow_length; i++){
      offset += this->flow[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t flow_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      flow_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      flow_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      flow_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->flow_length);
      if(flow_lengthT > flow_length)
        this->flow = (opencv_apps::Flow*)realloc(this->flow, flow_lengthT * sizeof(opencv_apps::Flow));
      flow_length = flow_lengthT;
      for( uint32_t i = 0; i < flow_length; i++){
      offset += this->st_flow.deserialize(inbuffer + offset);
        memcpy( &(this->flow[i]), &(this->st_flow), sizeof(opencv_apps::Flow));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/FlowArray"; };
    const char * getMD5(){ return "fe292dca56eb3673cd698ea9ef841962"; };

  };

}
#endif