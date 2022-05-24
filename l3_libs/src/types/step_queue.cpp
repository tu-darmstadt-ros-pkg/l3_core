#include <l3_libs/types/step_queue.h>

namespace l3
{
void StepQueue::fromMsg(const l3_msgs::StepQueue& msg)
{
  clear();
  header = msg.header;
  for (const l3_msgs::Step& s : msg.steps)
    enqueue(s.idx, Step::make(s));

  variantDataSetMsgToL3(msg.data, data);
}

void StepQueue::toMsg(l3_msgs::StepQueue& msg) const
{
  msg.header = header;
  msg.steps.clear();
  for (const Entry& e : *this)
    msg.steps.push_back(e.second->toMsg());

  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::StepQueue StepQueue::toMsg() const
{
  l3_msgs::StepQueue msg;
  toMsg(msg);
  return msg;
}
}  // namespace l3
