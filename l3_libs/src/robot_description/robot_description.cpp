#include <l3_libs/robot_description/robot_description.h>

#include <l3_libs/helper.h>

#include <l3_libs/conversions/l3_msg_foothold_conversions.h>

namespace l3
{
RobotDescription::RobotDescription(const XmlRpc::XmlRpcValue& params)
{
  params_ = params;

  if (params_.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("RobotDescription", "[RobotDescription] Parameters have wrong format.");
    return;
  }

  /// Parse feet section
  if (!params_.hasMember("feet"))
  {
    ROS_ERROR_NAMED("RobotDescription", "[RobotDescription] Missing 'feet' tag in parameters.");
    return;
  }
  parseFeet(params_["feet"]);

  /// Parse base section
  if (!params_.hasMember("bases"))
  {
    ROS_ERROR_NAMED("RobotDescription", "[RobotDescription] Missing 'bases' tag in parameters.");
    return;
  }
  parseBases(params_["bases"]);

  /// Parse leg section (optional)
  if (!params_.hasMember("legs"))
  {
    leg_info_map_.clear();

    leg_idx_.clear();
    leg_ids_.clear();
  }
  else
    parseLegs(params_["legs"]);

  generateIdxMaps();
  checkConsistency();
}

RobotDescription::RobotDescription(const l3_msgs::RobotDescription& msg) { fromMsg(msg); }

RobotDescription::RobotDescription(ros::NodeHandle& nh, const std::string& topic)
{
  l3_msgs::RobotDescription::ConstPtr desc_msg;

  while (!desc_msg)
  {
    desc_msg = ros::topic::waitForMessage<l3_msgs::RobotDescription>(topic, nh, ros::Duration(5.0));

    if (!desc_msg)
      ROS_WARN("[RobotDescription] No robot description received at topic '%s' yet!", ros::names::append(nh.getNamespace(), topic).c_str());
  }

  fromMsg(*desc_msg);
}

void RobotDescription::fromMsg(const l3_msgs::RobotDescription& msg)
{
  for (const l3_msgs::FootInfo& f : msg.foot_info)
    addFootInfo(FootInfo(f));

  for (const l3_msgs::BaseInfo& b : msg.base_info)
    addBaseInfo(BaseInfo(b));

  for (const l3_msgs::LegInfo& l : msg.leg_info)
    addLegInfo(LegInfo(l));

  /// @TODO: Handle params field

  generateIdxMaps();
  checkConsistency();
}

void RobotDescription::toMsg(l3_msgs::RobotDescription& msg) const
{
  for (const FootInfoPair& p : foot_info_map_)
    msg.foot_info.push_back(p.second.toMsg());

  for (const BaseInfoPair& p : base_info_map_)
    msg.base_info.push_back(p.second.toMsg());

  for (const LegInfoPair& p : leg_info_map_)
    msg.leg_info.push_back(p.second.toMsg());

  /// @TODO: Handle params field
}

bool RobotDescription::isValid() const
{
  if (foot_info_map_.empty())
    return false;

  if (foot_idx_.empty() || foot_ids_.empty())
    return false;

  if (foot_idx_.size() != foot_ids_.size() || leg_idx_.size() != leg_ids_.size())
    return false;

  return true;
}

bool RobotDescription::getFootInfo(const FootIndex& foot_idx, FootInfo& foot_info) const
{
  FootInfoMap::const_iterator itr = foot_info_map_.find(foot_idx);

  if (itr == foot_info_map_.end())
    return false;

  ROS_ASSERT(foot_idx == itr->first);

  foot_info = itr->second;
  return true;
}

FootInfo RobotDescription::getFootInfo(const FootIndex& foot_idx) const
{
  FootInfo foot_info;
  if (!getFootInfo(foot_idx, foot_info))
    ROS_WARN("[RobotDescription] Could not find FootInfo for foot id %u", foot_idx);
  return foot_info;
}

bool RobotDescription::getBaseInfo(const BaseIndex& base_idx, BaseInfo& base_info) const
{
  BaseInfoMap::const_iterator itr = base_info_map_.find(base_idx);

  if (itr == base_info_map_.end())
    return false;

  ROS_ASSERT(base_idx == itr->first);

  base_info = itr->second;
  return true;
}

BaseInfo RobotDescription::getBaseInfo(const BaseIndex& base_idx) const
{
  BaseInfo base_info;
  if (!getBaseInfo(base_idx, base_info))
    ROS_WARN("[RobotDescription] Could not find BaseInfo for base id %u", base_idx);
  return base_info;
}

bool RobotDescription::getLegInfo(const LegIndex& leg_idx, LegInfo& leg_info) const
{
  LegInfoMap::const_iterator itr = leg_info_map_.find(leg_idx);

  if (itr == leg_info_map_.end())
    return false;

  ROS_ASSERT(leg_idx == itr->first);

  leg_info = itr->second;
  return true;
}

LegInfo RobotDescription::getLegInfo(const LegIndex& leg_idx) const
{
  LegInfo leg_info;
  if (!getLegInfo(leg_idx, leg_info))
    ROS_WARN("[RobotDescription] Could not find LegInfo for leg id %u", leg_idx);
  return leg_info;
}

size_t RobotDescription::getFootIdx(const FootIndex& foot_idx) const
{
  auto itr = foot_idx_map_.find(foot_idx);

  if (itr == foot_idx_map_.end())
    return -1;
  else
    return itr->second;
}

size_t RobotDescription::getBaseIdx(const BaseIndex& base_idx) const
{
  auto itr = base_idx_map_.find(base_idx);

  if (itr == base_idx_map_.end())
    return -1;
  else
    return itr->second;
}

size_t RobotDescription::getLegIdx(const LegIndex& leg_idx) const
{
  auto itr = leg_idx_map_.find(leg_idx);

  if (itr == leg_idx_map_.end())
    return -1;
  else
    return itr->second;
}

FootholdArray RobotDescription::getNeutralStance(const Pose& center) const
{
  FootholdArray footholds;

  for (const Foothold& f : neutral_stance_)
    footholds.push_back(Foothold(f.idx, center * f.pose(), f.header));

  return footholds;
}

void RobotDescription::addFootInfo(const FootInfo& foot_info)
{
  foot_idx_.push_back(foot_info.idx);
  foot_ids_.push_back(foot_info.name);

  if (!foot_info.indirect)
    neutral_stance_.push_back(Foothold(foot_info.idx, foot_info.neutral_stance));

  foot_info_map_.insert(std::make_pair(foot_info.idx, foot_info));

  ROS_DEBUG("[RobotDescription] Added foot (%i, %s).", foot_info.idx, foot_info.name.c_str());
}

void RobotDescription::addBaseInfo(const BaseInfo& base_info)
{
  base_idx_.push_back(base_info.idx);
  base_ids_.push_back(base_info.name);

  base_info_map_.insert(std::make_pair(base_info.idx, base_info));

  ROS_DEBUG("[RobotDescription] Added base (%i, %s).", base_info.idx, base_info.name.c_str());
}

void RobotDescription::addLegInfo(const LegInfo& leg_info)
{
  leg_idx_.push_back(leg_info.idx);
  leg_ids_.push_back(leg_info.name);

  leg_info_map_.insert(std::make_pair(leg_info.idx, leg_info));

  ROS_DEBUG("[RobotDescription] Added leg (%i, %s).", leg_info.idx, leg_info.name.c_str());
}

void RobotDescription::parseFeet(XmlRpc::XmlRpcValue& feet)
{
  foot_info_map_.clear();

  foot_idx_.clear();
  foot_ids_.clear();
  neutral_stance_.clear();

  if (feet.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED("RobotDescription", "[RobotDescription] Feet configuration must be given as array.");
    return;
  }

  for (int i = 0; i < feet.size(); i++)
  {
    // obtain foot info
    FootInfo foot_info(feet[i]);
    addFootInfo(foot_info);
  }
}

void RobotDescription::parseBases(XmlRpc::XmlRpcValue& bases)
{
  base_info_map_.clear();

  base_idx_.clear();
  base_ids_.clear();

  if (bases.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED("RobotDescription", "[RobotDescription] Bases configuration must be given as array.");
    return;
  }

  for (int i = 0; i < bases.size(); i++)
  {
    // obtain foot info
    BaseInfo base_info(bases[i]);
    addBaseInfo(base_info);
  }
}

void RobotDescription::parseLegs(XmlRpc::XmlRpcValue& legs)
{
  leg_info_map_.clear();

  leg_idx_.clear();
  leg_ids_.clear();

  if (legs.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED("RobotDescription", "[RobotDescription] Legs configuration must be given as array.");
    return;
  }

  for (int i = 0; i < legs.size(); i++)
  {
    // obtain leg info
    LegInfo leg_info(legs[i]);
    addLegInfo(leg_info);
  }
}

void RobotDescription::generateIdxMaps()
{
  // foot idx to enumeration id in foot id vector
  foot_idx_map_.clear();
  size_t id = 0;
  for (const FootIndex& f : foot_idx_)
    foot_idx_map_[f] = id++;

  // foot idx to enumeration id in foot id vector
  leg_idx_map_.clear();
  id = 0;
  for (const LegIndex& l : leg_idx_)
    leg_idx_map_[l] = id++;

  // set of indirect foot idx
  indirect_foot_idx_.clear();
  for (const FootInfoPair& p : foot_info_map_)
  {
    if (p.second.indirect)
      indirect_foot_idx_.insert(p.first);
  }
}

bool RobotDescription::checkConsistency() const
{
  bool result = true;

  for (const LegInfoPair& p : leg_info_map_)
  {
    const LegInfo& leg_info = p.second;
    if (!hasFootInfo(leg_info.idx))
    {
      ROS_WARN("[RobotDescription] Leg %u ('%s') does not have a corresponding foot having the same idx! This violates the conventions and may lead to unexpected results.",
               leg_info.idx, leg_info.name.c_str());
      result = false;
    }
  }

  return result;
}
}  // namespace l3
