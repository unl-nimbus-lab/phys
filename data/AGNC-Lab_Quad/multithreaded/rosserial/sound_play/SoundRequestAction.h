#ifndef _ROS_sound_play_SoundRequestAction_h
#define _ROS_sound_play_SoundRequestAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sound_play/SoundRequestActionGoal.h"
#include "sound_play/SoundRequestActionResult.h"
#include "sound_play/SoundRequestActionFeedback.h"

namespace sound_play
{

  class SoundRequestAction : public ros::Msg
  {
    public:
      sound_play::SoundRequestActionGoal action_goal;
      sound_play::SoundRequestActionResult action_result;
      sound_play::SoundRequestActionFeedback action_feedback;

    SoundRequestAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "sound_play/SoundRequestAction"; };
    const char * getMD5(){ return "469561382c553cd0eaf8d2a4c8cef442"; };

  };

}
#endif