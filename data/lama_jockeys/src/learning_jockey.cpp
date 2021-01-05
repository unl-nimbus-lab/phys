/* Base class for learning jockeys
 *
 */

#include <lama_jockeys/learning_jockey.h>

namespace lama_jockeys
{

LearningJockey::LearningJockey(const std::string& name) :
  Jockey(name),
  server_(nh_, name, false)
{
  server_.registerGoalCallback(boost::bind(&LearningJockey::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&LearningJockey::preemptCallback, this));

  server_.start();
  ROS_DEBUG("Action server '%s' started for Learning", jockey_name_.c_str());
}

void LearningJockey::goalCallback()
{
  lama_jockeys::LearnGoalConstPtr current_goal = server_.acceptNewGoal();
  goal_ = *current_goal;

  // Check that preempt has not been requested by the client.
  if (server_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("%s: Preempted", jockey_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  switch (goal_.action)
  {
    case lama_jockeys::LearnGoal::LEARN:
      ROS_DEBUG("Received action LEARN");
      initAction();
      onLearn();
      break;
    case lama_jockeys::LearnGoal::STOP:
      ROS_DEBUG("Received action STOP");
      onStop();
      break;
    case lama_jockeys::LearnGoal::INTERRUPT:
      ROS_DEBUG("Received action INTERRUPT");
      interrupt();
      onInterrupt();
      break;
    case lama_jockeys::LearnGoal::CONTINUE:
      ROS_DEBUG("Received action CONTINUE");
      resume();
      onContinue();
      break;
  }
}

void LearningJockey::preemptCallback()
{
  ROS_INFO_STREAM(jockey_name_ << ": Preempted");
  // set the action state to preempted
  server_.setPreempted();
}

void LearningJockey::initAction()
{
  Jockey::initAction();
  result_ = LearnResult();
}

void LearningJockey::onInterrupt()
{
  ROS_DEBUG_STREAM(jockey_name_ << ": learning interrupted");
}

void LearningJockey::onContinue()
{
  ROS_DEBUG_STREAM(jockey_name_ << ": learning resumed");
}

} // namespace lama_jockeys

