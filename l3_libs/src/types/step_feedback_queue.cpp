#include <l3_libs/types/step_feedback_queue.h>

namespace l3
{
void StepFeedbackQueue::fromMsg(const l3_msgs::StepFeedbackQueue& msg)
{
  header = msg.header;
  clear();
  for (const l3_msgs::StepFeedback& s : msg.steps)
    enqueue(s.idx, StepFeedback::Ptr(new StepFeedback(s)));

  variantDataSetMsgToL3(msg.data, data);
}

void StepFeedbackQueue::toMsg(l3_msgs::StepFeedbackQueue& msg) const
{
  msg.header = header;
  msg.steps.clear();
  for (const Entry& e : *this)
    msg.steps.push_back(e.second->toMsg());

  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::StepFeedbackQueue StepFeedbackQueue::toMsg() const
{
  l3_msgs::StepFeedbackQueue msg;
  toMsg(msg);
  return msg;
}
}  // namespace l3
