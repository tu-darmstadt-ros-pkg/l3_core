#include <l3_libs/robot_description/leg_info.h>

#include <l3_libs/yaml_parser.h>
#include <l3_libs/conversions/l3_msg_foothold_conversions.h>

namespace l3
{
LegInfo::LegInfo()
  : idx(-1)
  , name("INVALID")
  , root_link("NO_LINK_SPECIFIED")
  , tip_link("NO_LINK_SPECIFIED")
  , joints(std::vector<std::string>())
{}

LegInfo::LegInfo(const XmlRpc::XmlRpcValue& params)
  : LegInfo()
{
  if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("LegInfo", "[LegInfo] Leg parameter has wrong format.");
    return;
  }

  XmlRpc::XmlRpcValue p = params;

  if (!getYamlValue(p, "idx", idx))
    return;

  if (!getYamlValue(p, "name", name))
    return;

  if (!getYamlValue(p, "root_link", root_link))
    return;

  if (!getYamlValue(p, "tip_link", tip_link))
    return;

  if (!getYamlValue(p, "joints", joints))
    return;
}

LegInfo::LegInfo(const l3_msgs::LegInfo& msg) { fromMsg(msg); }

void LegInfo::fromMsg(const l3_msgs::LegInfo& msg)
{
  idx = msg.idx;
  name = msg.name;
  root_link = msg.root_link;
  tip_link = msg.tip_link;
  joints = msg.joints;
}

void LegInfo::toMsg(l3_msgs::LegInfo& msg) const
{
  msg.idx = idx;
  msg.name = name;
  msg.root_link = root_link;
  msg.tip_link = tip_link;
  msg.joints = joints;
}
}  // namespace l3
