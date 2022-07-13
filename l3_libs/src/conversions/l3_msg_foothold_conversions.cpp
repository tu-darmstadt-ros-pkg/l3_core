#include <l3_libs/conversions/l3_msg_foothold_conversions.h>

namespace l3
{
void footholdArrayMsgToL3(const l3_msgs::FootholdArray& msg, l3::FootholdArray& footholds)
{
  footholds.clear();
  for (const l3_msgs::Foothold& f : msg)
    footholds.push_back(l3::Foothold(f));
}

void footholdArrayL3ToMsg(const l3::FootholdArray& footholds, l3_msgs::FootholdArray& msg)
{
  msg.clear();
  for (const l3::Foothold& f : footholds)
    msg.push_back(f.toMsg());
}

void footholdArrayL3ToMsg(const l3::FootholdPtrArray& footholds, l3_msgs::FootholdArray& msg)
{
  msg.clear();
  for (l3::Foothold::ConstPtr f : footholds)
    msg.push_back(f->toMsg());
}

void footholdArrayL3ToMsg(const l3::FootholdConstPtrArray& footholds, l3_msgs::FootholdArray& msg)
{
  msg.clear();
  for (l3::Foothold::ConstPtr f : footholds)
    msg.push_back(f->toMsg());
}
}  // namespace l3
