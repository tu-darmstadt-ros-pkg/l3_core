#include <l3_libs/conversions/l3_msg_step_conversions.h>

namespace l3
{
void footStepDataArrayMsgToL3(const l3_msgs::FootStepDataArray& msg, l3::FootStepDataArray& foot_step_data)
{
  foot_step_data.clear();
  for (const l3_msgs::FootStepData& s : msg)
    foot_step_data.push_back(l3::FootStepData(s));
}

void footStepDataArrayL3ToMsg(const l3::FootStepDataArray& foot_step_data, l3_msgs::FootStepDataArray& msg)
{
  msg.clear();
  for (const l3::FootStepData& s : foot_step_data)
    msg.push_back(s.toMsg());
}

void footStepDataArrayL3ToMsg(l3::FootStepDataPtrArray foot_step_data, l3_msgs::FootStepDataArray& msg)
{
  msg.clear();
  for (l3::FootStepData::ConstPtr s : foot_step_data)
    msg.push_back(s->toMsg());
}

void footStepDataArrayL3ToMsg(l3::FootStepDataConstPtrArray foot_step_data, l3_msgs::FootStepDataArray& msg)
{
  msg.clear();
  for (l3::FootStepData::ConstPtr s : foot_step_data)
    msg.push_back(s->toMsg());
}

void baseStepDataArrayMsgToL3(const l3_msgs::BaseStepDataArray& msg, l3::BaseStepDataArray& base_step_data)
{
  base_step_data.clear();
  for (const l3_msgs::BaseStepData& s : msg)
    base_step_data.push_back(l3::BaseStepData(s));
}

void baseStepDataArrayMsgToL3(const l3::BaseStepDataArray& base_step_data, l3_msgs::BaseStepDataArray& msg)
{
  msg.clear();
  for (const l3::BaseStepData& s : base_step_data)
    msg.push_back(s.toMsg());
}

void baseStepDataArrayMsgToL3(l3::BaseStepDataPtrArray base_step_data, l3_msgs::BaseStepDataArray& msg)
{
  msg.clear();
  for (l3::BaseStepData::ConstPtr s : base_step_data)
    msg.push_back(s->toMsg());
}

void baseStepDataArrayMsgToL3(l3::BaseStepDataConstPtrArray base_step_data, l3_msgs::BaseStepDataArray& msg)
{
  msg.clear();
  for (l3::BaseStepData::ConstPtr s : base_step_data)
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
