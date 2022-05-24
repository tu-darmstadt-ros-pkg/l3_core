#include <l3_libs/conversions/l3_msg_step_conversions.h>

namespace l3
{
void stepDataArrayMsgToL3(const l3_msgs::StepDataArray& msg, l3::StepDataArray& step_data)
{
  step_data.clear();
  for (const l3_msgs::StepData& s : msg)
    step_data.push_back(l3::StepData(s));
}

void stepDataArrayL3ToMsg(const l3::StepDataArray& step_data, l3_msgs::StepDataArray& msg)
{
  msg.clear();
  for (const l3::StepData& s : step_data)
    msg.push_back(s.toMsg());
}

void stepDataArrayL3ToMsg(l3::StepDataPtrArray step_data, l3_msgs::StepDataArray& msg)
{
  msg.clear();
  for (l3::StepData::ConstPtr s : step_data)
    msg.push_back(s->toMsg());
}

void stepDataArrayL3ToMsg(l3::StepDataConstPtrArray step_data, l3_msgs::StepDataArray& msg)
{
  msg.clear();
  for (l3::StepData::ConstPtr s : step_data)
    msg.push_back(s->toMsg());
}

void stepArrayMsgToL3(const l3_msgs::StepArray& msg, l3::StepArray& steps)
{
  steps.clear();
  for (const l3_msgs::Step& s : msg)
    steps.push_back(l3::Step(s));
}

void stepArrayL3ToMsg(const l3::StepArray& steps, l3_msgs::StepArray& msg)
{
  msg.clear();
  for (const l3::Step& s : steps)
    msg.push_back(s.toMsg());
}

void stepArrayL3ToMsg(l3::StepPtrArray steps, l3_msgs::StepArray& msg)
{
  msg.clear();
  for (l3::Step::ConstPtr s : steps)
    msg.push_back(s->toMsg());
}

void stepArrayL3ToMsg(l3::StepConstPtrArray steps, l3_msgs::StepArray& msg)
{
  msg.clear();
  for (l3::Step::ConstPtr s : steps)
    msg.push_back(s->toMsg());
}
}  // namespace l3
