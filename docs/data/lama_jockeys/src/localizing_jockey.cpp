#include <lama_jockeys/localizing_jockey.h>

namespace lama_jockeys
{

LocalizingJockey::LocalizingJockey(const std::string& name) :
  Jockey(name),
  server_(nh_, name, boost::bind(&LocalizingJockey::goalCallback, this, _1), false)
{
  server_.registerPreemptCallback(boost::bind(&LocalizingJockey::preemptCallback, this));

  server_.start();
  ROS_DEBUG("Action server '%s' started for Localization", jockey_name_.c_str());
}

void LocalizingJockey::goalCallback(const lama_jockeys::LocalizeGoalConstPtr& goal)
{
  goal_.action = goal->action;

  // Check that preempt has not been requested by the client.
  if (server_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO_STREAM(jockey_name_ << ": Preempted");
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  switch (goal_.action)
  {
    case lama_jockeys::LocalizeGoal::GET_VERTEX_DESCRIPTOR:
      ROS_DEBUG("Received action GET_VERTEX_DESCRIPTOR");
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onGetVertexDescriptor();
      break;
    case lama_jockeys::LocalizeGoal::GET_EDGES_DESCRIPTORS:
      ROS_DEBUG("Received action GET_EDGES_DESCRIPTORS");
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onGetEdgesDescriptors();
      break;
    case lama_jockeys::LocalizeGoal::LOCALIZE_IN_VERTEX:
      ROS_DEBUG("Received action LOCALIZE_IN_VERTEX");
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onLocalizeInVertex();
      break;
    case lama_jockeys::LocalizeGoal::LOCALIZE_EDGE:
      ROS_DEBUG("Received action LOCALIZE_EDGE");
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onLocalizeEdge();
      break;
    case lama_jockeys::LocalizeGoal::GET_DISSIMILARITY:
      ROS_DEBUG("Received action GET_DISSIMILARITY");
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onGetDissimilarity();
      break;
    case lama_jockeys::LocalizeGoal::INTERRUPT:
      ROS_DEBUG("Received action INTERRUPT");
      interrupt();
      onInterrupt();
      break;
    case lama_jockeys::LocalizeGoal::CONTINUE:
      ROS_DEBUG("Received action CONTINUE");
      resume();
      onContinue();
      break;
  }
}

void LocalizingJockey::preemptCallback()
{
  ROS_INFO_STREAM(jockey_name_ << ": Preempted");
  // set the action state to preempted
  server_.setPreempted();
}

void LocalizingJockey::initAction()
{
  Jockey::initAction();
  result_ = LocalizeResult();
}

void LocalizingJockey::onGetVertexDescriptor()
{
  ROS_DEBUG("%s: onGetVertexDescriptor not supported", jockey_name_.c_str());
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LocalizingJockey::onGetEdgesDescriptors()
{
  ROS_DEBUG("%s: onGetEdgesDescriptors not supported", jockey_name_.c_str());
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LocalizingJockey::onLocalizeInVertex()
{
  ROS_DEBUG("%s: onLocalizeInVertex not supported", jockey_name_.c_str());
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LocalizingJockey::onLocalizeEdge()
{
  ROS_DEBUG("%s: onLocalizeEdge not supported", jockey_name_.c_str());
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LocalizingJockey::onGetDissimilarity()
{
  ROS_DEBUG("%s: onGetDissimilarity not supported", jockey_name_.c_str());
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LocalizingJockey::onInterrupt()
{
  ROS_DEBUG("%s: localizing goal with lama object %d interrupted", jockey_name_.c_str(), goal_.descriptor_link.object_id);
}

void LocalizingJockey::onContinue()
{
  ROS_DEBUG("%s: localizing goal with lama object %d resumed", jockey_name_.c_str(), goal_.descriptor_link.object_id);
}

} // namespace lama_jockeys

