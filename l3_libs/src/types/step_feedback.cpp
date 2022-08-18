#include <l3_libs/types/step_feedback.h>

namespace l3
{
void StepFeedback::reset()
{
  for (FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
    p.second->reset();
}

void StepFeedback::fromMsg(const l3_msgs::StepFeedback& msg)
{
  clear();

  for (const l3_msgs::StepFeedbackData& s : msg.step_feedback_data)
    feedback_data_.updateMovingLink(s.foot_step.origin.idx, makeShared<StepFeedbackData>(s));
  feedback_data_.setStepIndex(msg.idx);

  variantDataSetMsgToL3(msg.data, data);
}

void StepFeedback::toMsg(l3_msgs::StepFeedback& msg) const
{
  msg.step_feedback_data.clear();
  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
    msg.step_feedback_data.push_back(p.second->toMsg());

  msg.idx = feedback_data_.getStepIndex();

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
  feedback_data_.clear();
  changeable_ = false;
  executing_ = false;
  finished_ = false;
}

bool StepFeedback::isChangeable() const
{
  if (!feedback_data_.hasMovingLinks())
    return changeable_;

  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    if (!p.second->changeable)
      return false;
  }
  return true;
}

bool StepFeedback::isExecuting() const
{
  if (!feedback_data_.hasMovingLinks())
    return executing_;

  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    if (p.second->executing)
      return true;
  }
  return false;
}

bool StepFeedback::isFinished() const
{
  if (!feedback_data_.hasMovingLinks())
    return finished_;

  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    if (!p.second->finished)
      return false;
  }
  return true;
}

void StepFeedback::setChangeable()
{
  if (!feedback_data_.hasMovingLinks())
  {
    changeable_ = true;
    executing_ = false;
    finished_ = false;
  }

  for (FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    p.second->changeable = true;
    p.second->executing = false;
    p.second->finished = false;
  }
}

void StepFeedback::setExecuting()
{
  if (!feedback_data_.hasMovingLinks())
  {
    changeable_ = false;
    executing_ = true;
    finished_ = false;
  }

  for (FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    p.second->changeable = false;
    p.second->executing = true;
    p.second->finished = false;
  }
}

void StepFeedback::setFinished()
{
  if (!feedback_data_.hasMovingLinks())
  {
    changeable_ = false;
    executing_ = false;
    finished_ = true;
  }

  for (FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    p.second->changeable = false;
    p.second->executing = false;
    p.second->finished = true;
  }
}

ros::Time StepFeedback::firstExecutionStart() const
{
  ros::Time time;

  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    if (time.isZero() || time > p.second->execution_start)
      time = p.second->execution_start;
  }
  return time;
}

ros::Time StepFeedback::latestExecutionStart() const
{
  ros::Time time;

  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    if (time.isZero() || time < p.second->execution_start)
      time = p.second->execution_start;
  }
  return time;
}

ros::Time StepFeedback::firstExecutionEnd() const
{
  ros::Time time;

  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    if (time.isZero() || time > p.second->execution_end)
      time = p.second->execution_end;
  }
  return time;
}

ros::Time StepFeedback::latestExecutionEnd() const
{
  ros::Time time;

  for (const FeedbackStep::MovingDataPair& p : feedback_data_.getMovingLinks())
  {
    if (time.isZero() || time < p.second->execution_end)
      time = p.second->execution_end;
  }
  return time;
}
}  // namespace l3
