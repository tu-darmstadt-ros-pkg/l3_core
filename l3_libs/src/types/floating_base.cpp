#include <l3_libs/types/floating_base.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3
{
FloatingBase::FloatingBase()
  : BaseLink()
{}

FloatingBase::FloatingBase(const BaseIndex& idx, const Pose& pose, const std_msgs::Header& header, const VariantDataSet& data)
  : BaseLink(idx, pose, header, data)
{
}

FloatingBase::FloatingBase(const BaseIndex& idx, double x, double y, double z, double roll, double pitch, double yaw, const std_msgs::Header& header, const VariantDataSet& data)
  : BaseLink(idx, x, y, z, roll, pitch, yaw, header, data)
{
}

FloatingBase::FloatingBase(const l3_msgs::FloatingBase& msg) { fromMsg(msg); }

void FloatingBase::fromMsg(const l3_msgs::FloatingBase& msg)
{
  header = msg.header;
  idx = msg.idx;
  poseMsgToL3(msg.pose, pose_);
  pose_.getRPY(roll_, pitch_, yaw_);
  variantDataSetMsgToL3(msg.data, data);
}

void FloatingBase::toMsg(l3_msgs::FloatingBase& msg) const
{
  msg.header = header;
  msg.idx = idx;
  poseL3ToMsg(pose_, msg.pose);
  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::FloatingBase FloatingBase::toMsg() const
{
  l3_msgs::FloatingBase msg;
  toMsg(msg);
  return msg;
}
}  // namespace l3

L3_VARIANT_DATA_REGISTER_SERIALIZATION(l3::FloatingBase, l3::FloatingBase::serialize, l3::FloatingBase::deserialize)
