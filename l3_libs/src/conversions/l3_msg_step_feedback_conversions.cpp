#include <l3_libs/conversions/l3_msg_step_feedback_conversions.h>

namespace l3
{
void stepFeedbackDataArrayMsgToL3(const l3_msgs::StepFeedbackDataArray& msg, l3::StepFeedbackDataArray& step_data)
{
  step_data.clear();
  for (const l3_msgs::StepFeedbackData& s : msg)
    step_data.push_back(l3::StepFeedbackData(s));
}

void stepFeedbackDataArrayL3ToMsg(const l3::StepFeedbackDataArray& step_data, l3_msgs::StepFeedbackDataArray& msg)
{
  msg.clear();
  for (const l3::StepFeedbackData& s : step_data)
    msg.push_back(s.toMsg());
}

void stepFeedbackDataArrayL3ToMsg(l3::StepFeedbackDataPtrArray step_data, l3_msgs::StepFeedbackDataArray& msg)
{
  msg.clear();
  for (l3::StepFeedbackData::ConstPtr s : step_data)
    msg.push_back(s->toMsg());
}

void stepFeedbackDataArrayL3ToMsg(l3::StepFeedbackDataConstPtrArray step_data, l3_msgs::StepFeedbackDataArray& msg)
{
  msg.clear();
  for (l3::StepFeedbackData::ConstPtr s : step_data)
    msg.push_back(s->toMsg());
}

void stepFeedbackArrayMsgToL3(const l3_msgs::StepFeedbackArray& msg, l3::StepFeedbackArray& steps)
{
  steps.clear();
  for (const l3_msgs::StepFeedback& s : msg)
    steps.push_back(l3::StepFeedback(s));
}

void stepFeedbackArrayL3ToMsg(const l3::StepFeedbackArray& steps, l3_msgs::StepFeedbackArray& msg)
{
  msg.clear();
  for (const l3::StepFeedback& s : steps)
    msg.push_back(s.toMsg());
}

void stepFeedbackArrayL3ToMsg(l3::StepFeedbackPtrArray steps, l3_msgs::StepFeedbackArray& msg)
{
  msg.clear();
  for (l3::StepFeedback::ConstPtr s : steps)
    msg.push_back(s->toMsg());
}

void stepFeedbackArrayL3ToMsg(l3::StepFeedbackConstPtrArray steps, l3_msgs::StepFeedbackArray& msg)
{
  msg.clear();
  for (l3::StepFeedback::ConstPtr s : steps)
    msg.push_back(s->toMsg());
}
}  // namespace l3
