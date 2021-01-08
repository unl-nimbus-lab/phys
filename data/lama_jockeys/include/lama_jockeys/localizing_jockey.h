/* 
 * Base class for localizing jockeys
 *
 */

#ifndef _LAMA_JOCKEYS_LOCALIZING_JOCKEY_H_
#define _LAMA_JOCKEYS_LOCALIZING_JOCKEY_H_

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <lama_jockeys/jockey.h>
#include <lama_jockeys/LocalizeAction.h>
#include <lama_jockeys/LocalizeGoal.h>
#include <lama_jockeys/LocalizeFeedback.h>

namespace lama_jockeys {

typedef actionlib::SimpleActionServer<lama_jockeys::LocalizeAction> LocalizeServer;

class LocalizingJockey : public Jockey
{
  protected:

    LocalizingJockey(const std::string& name);

    virtual void onGetVertexDescriptor();
    virtual void onGetEdgesDescriptors();
    virtual void onLocalizeInVertex();
    virtual void onLocalizeEdge();
    virtual void onGetDissimilarity();
    virtual void onInterrupt();
    virtual void onContinue();

    void initAction();

    // NodeHandle instance must be created before this line. Otherwise strange
    // error may occur (this is done in Jockey).
    LocalizeServer server_;
    LocalizeResult result_;
    LocalizeFeedback feedback_;

    // In case of INTERRUPT and CONTINUE, the descritptor attribute
    // of current goal are irrelevant.
    // This information needs to be saved for use after a CONTINUE action.
    LocalizeGoal goal_;

  private:

    void goalCallback(const lama_jockeys::LocalizeGoalConstPtr& goal);
    void preemptCallback();

    // Change the visibility to avoid double calls.
    using Jockey::interrupt;
    using Jockey::resume;
};

} // namespace lama_jockeys

#endif // _LAMA_JOCKEYS_LOCALIZING_JOCKEY_H_
