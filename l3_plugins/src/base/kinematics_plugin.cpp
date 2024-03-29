#include <l3_plugins/base/kinematics_plugin.h>

#include <l3_libs/helper.h>

namespace l3
{
KinematicsPlugin::KinematicsPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name)
{}

bool KinematicsPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  root_link_ = param("root_link", std::string(), true);
  leveled_base_ = param("leveled_base", true, true);

  return true;
}

void KinematicsPlugin::setRobotDescription(RobotDescription::ConstPtr robot_description) { robot_description_ = robot_description; }

Transform KinematicsPlugin::calcStaticFeetCenterToRoot() const { return calcStaticFeetCenterToBase() * calcBaseToRoot(); }

Transform KinematicsPlugin::calcStaticFeetCenterToRoot(const Pose& feet_center) const { return calcStaticFeetCenterToBase(feet_center) * calcBaseToRoot(); }

Transform KinematicsPlugin::calcFeetCenterToRoot(const Pose& feet_center, const FootholdArray& footholds) const
{
  return calcFeetCenterToBase(feet_center, footholds) * calcBaseToRoot();
}

Transform KinematicsPlugin::calcFeetCenterToRoot(const Pose& feet_center, const FootholdConstPtrArray& footholds) const
{
  return calcFeetCenterToBase(feet_center, footholds) * calcBaseToRoot();
}

Transform KinematicsPlugin::calcStaticFeetCenterToBase() const
{
  BaseInfo base_info = robot_description_->getBaseInfo(BaseInfo::MAIN_BODY_IDX);
  return base_info.link_to_feet_center_offset.inverse();
}

Transform KinematicsPlugin::calcStaticFeetCenterToBase(const Pose& feet_center) const
{
  if (leveled_base_)
    return Transform(0.0, 0.0, 0.0, feet_center.roll(), feet_center.pitch(), 0.0).inverse() * calcStaticFeetCenterToBase();
  else
    return calcStaticFeetCenterToBase();
}

Transform KinematicsPlugin::calcFeetCenterToBase(const Pose& feet_center, const FootholdArray& /*footholds*/) const
{
  return calcStaticFeetCenterToBase(feet_center);
}

Transform KinematicsPlugin::calcFeetCenterToBase(const Pose& feet_center, const FootholdConstPtrArray& /*footholds*/) const
{
  return calcStaticFeetCenterToBase(feet_center);
}

Transform KinematicsPlugin::calcBaseToRoot() const
{
  BaseInfo base_info = robot_description_->getBaseInfo(BaseInfo::MAIN_BODY_IDX);
  const std::string& base_link = base_info.link;

  Transform base_to_root;

  if (!calcStaticTransformForChain(base_link, root_link_, base_to_root))
    ROS_WARN("[%s] calcBaseToRoot: Could not determine transform from root ('%s') to base ('%s'). Using identity transform.", getName().c_str(), root_link_.c_str(),
             base_link.c_str());

  return base_to_root;
}

bool KinematicsPlugin::calcLegIK(const Pose& base_pose, const Foothold& foothold, const std::vector<double>& cur_q,
                                 std::vector<double>& q) const
{
  q.clear();

  // extract robot information
  BaseInfo base_info = robot_description_->getBaseInfo(BaseInfo::MAIN_BODY_IDX);
  const std::string& base_link = base_info.link;

  FootInfo foot_info;
  if (!robot_description_->getFootInfo(foothold.idx, foot_info))
    return false;

  LegInfo leg_info;
  if (!robot_description_->getLegInfo(foothold.idx, leg_info))
    return false;

  // here, input foothold are represented in planner frame (center of sole), just offset them into robot's foot frame
  // in order to match poses based on the foot frame as specified in FootInfo
  Foothold foothold_transformed = foothold;
  foothold_transformed.setPose(foothold.pose() * foot_info.link_to_sole_offset.inverse());

  // calculate (static) base to leg root transform (IK computes from leg root which must not be equal to base frame)
  /// @todo could be cached for efficieny reasons
  Transform base_to_leg_root;
  if (!calcStaticTransformForChain(base_link, leg_info.root_link, base_to_leg_root))
  {
    ROS_WARN_ONCE("[%s] calcLegIK: Could not determine transform from base ('%s') to leg root ('%s'). Using identity transform.", getName().c_str(), base_link.c_str(),
                  leg_info.root_link.c_str());
    base_to_leg_root = Transform();
  }

  // calculate (static) leg tip to foot transforms (IK computes to last joint frame which must not be equal to foot frame in which the planner expresses footholds)
  /// @todo could be cached for efficieny reasons
  Transform leg_tip_to_foot;
  if (!calcStaticTransformForChain(leg_info.tip_link, foot_info.link, leg_tip_to_foot))
  {
    ROS_WARN_ONCE("[%s] calcLegIK: Could not determine transform from leg tip ('%s') to foot ('%s'). Using identity transform.", getName().c_str(), leg_info.tip_link.c_str(),
                  foot_info.link.c_str());
    leg_tip_to_foot = Transform();
  }

  // finally transform target pose relative to base
  Pose target = base_to_leg_root * base_pose.inverse() * (foothold.pose() * foot_info.link_to_sole_offset.inverse() * leg_tip_to_foot.inverse());

  // get ik solution
  if (cur_q.empty())
  {
    if (!calcInverseKinematicsForChain(leg_info.root_link, leg_info.tip_link, target, q))
    {
      // ROS_WARN("[%s] calcLegIK: No IK solution found for leg '%s' (%u)!", getName().c_str(), leg_info.name.c_str(), leg_info.idx);
      return false;
    }
  }
  else
  {
    if (!calcInverseKinematicsForChain(leg_info.root_link, leg_info.tip_link, target, cur_q, q))
    {
      // ROS_WARN("[%s] calcLegIK: No IK solution found for leg '%s' (%u)!", getName().c_str(), leg_info.name.c_str(), leg_info.idx);
      return false;
    }
  }

  if (q.size() != leg_info.joints.size())
  {
    ROS_ERROR_ONCE("[%s] calcLegIK: IK returned wrong number of joints for leg '%s' (%u)! Expected %lu but got %lu.", getName().c_str(), leg_info.name.c_str(), leg_info.idx,
                   leg_info.joints.size(), q.size());
    return false;
  }

  return true;
}

bool KinematicsPlugin::calcNeutralStanceIK(const Pose& base_pose, std::map<LegIndex, std::vector<double>>& leg_joint_states) const
{
  leg_joint_states.clear();

  for (const FootInfoPair& p : robot_description_->getFootInfoMap())
  {
    const FootInfo& foot_info = p.second;

    // ignore foot without leg info
    if (!robot_description_->hasLegInfo(foot_info.idx))
      continue;

    std::vector<double> q;
    if (calcLegIK(base_pose, Foothold(foot_info.idx, foot_info.neutral_stance), q))
    {
      leg_joint_states[foot_info.idx] = q;
    }
    else
    {
      ROS_ERROR("[%s] calcNeutralStanceIK: Failed to calculate neutral stance IK for leg %u (%s)!", getName().c_str(), foot_info.idx,
                robot_description_->getLegInfo(foot_info.idx).name.c_str());
      return false;
    }
  }

  return true;
}
}  // namespace l3
