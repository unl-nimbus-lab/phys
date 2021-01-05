#ifndef _ROS_sound_play_SoundRequestGoal_h
#define _ROS_sound_play_SoundRequestGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sound_play/SoundRequest.h"

namespace sound_play
{

  class SoundRequestGoal : public ros::Msg
  {
    public:
      sound_play::SoundRequest sound_request;

    SoundRequestGoal():
      sound_request()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->sound_request.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->sound_request.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "sound_play/SoundRequestGoal"; };
    const char * getMD5(){ return "43529ed05bef9015b67d8030a34f6f04"; };

  };

}
#endif