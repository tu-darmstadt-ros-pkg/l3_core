#include <l3_libs/conversions/l3_msg_floating_base_conversions.h>

namespace l3
{
void floatingBaseArrayMsgToL3(const l3_msgs::FloatingBaseArray& msg, l3::FloatingBaseArray& floating_bases)
{
  floating_bases.clear();
  for (const l3_msgs::FloatingBase& f : msg)
    floating_bases.push_back(l3::FloatingBase(f));
}

void floatingBaseArrayL3ToMsg(const l3::FloatingBaseArray& floating_bases, l3_msgs::FloatingBaseArray& msg)
{
  msg.clear();
  for (const l3::FloatingBase& f : floating_bases)
    msg.push_back(f.toMsg());
}

void floatingBaseArrayL3ToMsg(const l3::FloatingBasePtrArray& floating_bases, l3_msgs::FloatingBaseArray& msg)
{
  msg.clear();
  for (l3::FloatingBase::ConstPtr f : floating_bases)
    msg.push_back(f->toMsg());
}

void floatingBaseArrayL3ToMsg(const l3::FloatingBaseConstPtrArray& floating_bases, l3_msgs::FloatingBaseArray& msg)
{
  msg.clear();
  for (l3::FloatingBase::ConstPtr f : floating_bases)
    msg.push_back(f->toMsg());
}
}  // namespace l3
