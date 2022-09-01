#include <l3_libs/types/foothold.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3
{
Foothold::Foothold()
  : BaseLink()
{}

Foothold::Foothold(const FootIndex& idx, const Pose& pose, const std_msgs::Header& header, const VariantDataSet& data)
  : BaseLink(idx, pose, header, data)
{
}

Foothold::Foothold(const FootIndex& idx, double x, double y, double z, double roll, double pitch, double yaw, const std_msgs::Header& header, const VariantDataSet& data)
  : BaseLink(idx, x, y, z, roll, pitch, yaw, header, data)
{
}

Foothold::Foothold(const l3_msgs::Foothold& msg) { fromMsg(msg); }

void Foothold::fromMsg(const l3_msgs::Foothold& msg)
{
  header = msg.header;
  idx = msg.idx;
  poseMsgToL3(msg.pose, pose_);
  pose_.getRPY(roll_, pitch_, yaw_);
  variantDataSetMsgToL3(msg.data, data);
}

void Foothold::toMsg(l3_msgs::Foothold& msg) const
{
  msg.header = header;
  msg.idx = idx;
  poseL3ToMsg(pose_, msg.pose);
  variantDataSetL3ToMsg(data, msg.data);
}

l3_msgs::Foothold Foothold::toMsg() const
{
  l3_msgs::Foothold msg;
  toMsg(msg);
  return msg;
}
}  // namespace l3

L3_VARIANT_DATA_REGISTER_SERIALIZATION(l3::Foothold, l3::Foothold::serialize, l3::Foothold::deserialize)
