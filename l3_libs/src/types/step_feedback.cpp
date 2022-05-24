#include <l3_libs/types/step_feedback.h>

namespace l3
{
void StepFeedback::reset()
{
  for (StepDataPair& p : step_data_map_)
    p.second->reset();
}

void StepFeedback::fromMsg(const l3_msgs::StepFeedback& msg)
{
  clear();

  for (const l3_msgs::StepFeedbackData& s : msg.step_feedback_data)
    step_data_map_[s.step_data.origin.idx].reset(new StepFeedbackData(s));
  idx_ = msg.idx;

  variantDataSetMsgToL3(msg.data, data);
}

void StepFeedback::toMsg(l3_msgs::StepFeedback& msg) const
{
  msg.step_feedback_data.clear();
  for (const StepDataPair& p : step_data_map_)
    msg.step_feedback_data.push_back(p.second->toMsg());

  msg.idx = idx_;

  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::StepFeedback StepFeedback::toMsg() const
{
  l3_msgs::StepFeedback msg;
  toMsg(msg);
  return msg;
}

void StepFeedback::clear()
{
  BaseStep<StepFeedbackData::Ptr>::clear();
  changeable = false;
  executing = false;
  finished = false;
}

bool StepFeedback::isChangeable() const
{
  if (step_data_map_.empty())
    return changeable;

  for (const StepDataPair& p : step_data_map_)
  {
    if (!p.second->changeable)
      return false;
  }
  return true;
}

bool StepFeedback::isExecuting() const
{
  if (step_data_map_.empty())
    return executing;

  for (const StepDataPair& p : step_data_map_)
  {
    if (p.second->executing)
      return true;
  }
  return false;
}

bool StepFeedback::isFinished() const
{
  if (step_data_map_.empty())
    return finished;

  for (const StepDataPair& p : step_data_map_)
  {
    if (!p.second->finished)
      return false;
  }
  return true;
}

void StepFeedback::setChangeable()
{
  if (step_data_map_.empty())
  {
    changeable = true;
    executing = false;
    finished = false;
  }

  for (StepDataPair& p : step_data_map_)
  {
    p.second->changeable = true;
    p.second->executing = false;
    p.second->finished = false;
  }
}

void StepFeedback::setExecuting()
{
  if (step_data_map_.empty())
  {
    changeable = false;
    executing = true;
    finished = false;
  }

  for (StepDataPair& p : step_data_map_)
  {
    p.second->changeable = false;
    p.second->executing = true;
    p.second->finished = false;
  }
}

void StepFeedback::setFinished()
{
  if (step_data_map_.empty())
  {
    changeable = false;
    executing = false;
    finished = true;
  }

  for (StepDataPair& p : step_data_map_)
  {
    p.second->changeable = false;
    p.second->executing = false;
    p.second->finished = true;
  }
}

ros::Time StepFeedback::firstExecutionStart() const
{
  ros::Time time;

  for (const StepDataPair& p : step_data_map_)
  {
    if (time.isZero() || time > p.second->execution_start)
      time = p.second->execution_start;
  }
  return time;
}

ros::Time StepFeedback::latestExecutionStart() const
{
  ros::Time time;

  for (const StepDataPair& p : step_data_map_)
  {
    if (time.isZero() || time < p.second->execution_start)
      time = p.second->execution_start;
  }
  return time;
}

ros::Time StepFeedback::firstExecutionEnd() const
{
  ros::Time time;

  for (const StepDataPair& p : step_data_map_)
  {
    if (time.isZero() || time > p.second->execution_end)
      time = p.second->execution_end;
  }
  return time;
}

ros::Time StepFeedback::latestExecutionEnd() const
{
  ros::Time time;

  for (const StepDataPair& p : step_data_map_)
  {
    if (time.isZero() || time < p.second->execution_end)
      time = p.second->execution_end;
  }
  return time;
}
}  // namespace l3
