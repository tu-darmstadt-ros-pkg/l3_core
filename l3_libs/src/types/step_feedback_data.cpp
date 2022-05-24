#include <l3_libs/types/step_feedback_data.h>

namespace l3
{
StepFeedbackData::StepFeedbackData()
  : StepData()
  , changeable(false)
  , executing(false)
  , finished(false)
{}

StepFeedbackData::StepFeedbackData(const StepData& other)
  : StepData(other)
  , changeable(false)
  , executing(false)
  , finished(false)
{}

void StepFeedbackData::reset()
{
  changeable = false;
  executing = false;
  finished = false;
}

void StepFeedbackData::fromMsg(const l3_msgs::StepFeedbackData& msg)
{
  StepData::fromMsg(msg.step_data);
  changeable = msg.changeable;
  executing = msg.executing;
  finished = msg.finished;
  execution_start = msg.execution_start;
  execution_end = msg.execution_end;
}

void StepFeedbackData::toMsg(l3_msgs::StepFeedbackData& msg) const
{
  StepData::toMsg(msg.step_data);
  msg.changeable = changeable;
  msg.executing = executing;
  msg.finished = finished;
  msg.execution_start = execution_start;
  msg.execution_end = execution_end;
}

l3_msgs::StepFeedbackData StepFeedbackData::toMsg() const
{
  l3_msgs::StepFeedbackData msg;
  toMsg(msg);
  return msg;
}
}  // namespace l3
