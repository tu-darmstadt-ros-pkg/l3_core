#include <l3_plugins/std/cyclic_gait_generator.h>

#include <l3_libs/yaml_parser.h>

#include <l3_plugins/robot_model.h>

namespace l3
{
CyclicGaitGenerator::CyclicGaitGenerator()
  : GaitGeneratorPlugin("cyclic_gait_generator")
{}

void CyclicGaitGenerator::setRobotDescription(RobotDescription::ConstPtr robot_description)
{
  GaitGeneratorPlugin::setRobotDescription(robot_description);

  cycle_.clear();
  succ_.clear();
  pred_.clear();
  start_.clear();

  if (hasParam("cycle"))
  {
    if (!getExpandStatesIdxArray("cycle", cycle_))
      return;
  }
  else
  {
    ROS_INFO_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] No cycle was given, generating simple foot cycling.");
    for (const FootInfoPair& p : RobotModel::description()->getFootInfoMap())
    {
      if (!p.second.indirect)
        cycle_.push_back(ExpandStatesIdx{ FootIndexArray{ p.first }, BaseIndexArray{} });
    }

    if (cycle_.empty())
    {
      ROS_ERROR_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] RobotDescription does not contain any feet!");
      return;
    }
  }

  // output cycle
  std::string out;
  for (const ExpandStatesIdx& arr : cycle_)
  {
    out += "[ ";
    for (const FootIndex& idx : arr.foot_idx)
      out += toString(idx) + " ";
    out += "] -> ";
  }
  ROS_INFO_STREAM("Cycle: " + out);

  // generate lookup tables
  for (size_t i = 0; i < cycle_.size(); i++)
  {
    if (i == 0)
    {
      pred_[cycle_.front().foot_idx] = cycle_.back().foot_idx;
      succ_[cycle_.back().foot_idx] = cycle_.front().foot_idx;
    }
    else
    {
      pred_[cycle_[i].foot_idx] = cycle_[i - 1].foot_idx;
      succ_[cycle_[i - 1].foot_idx] = cycle_[i].foot_idx;
    }
  }

  // check start patterns
  if (hasParam("start"))
  {
    if (!getExpandStatesIdxArray("start", start_))
      return;

    // sanity check of input
    for (const ExpandStatesIdx& arr : start_)
    {
      if (pred_.find(arr.foot_idx) == pred_.end())
      {
        ROS_ERROR_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] At least one input start option does not match with any element of the given cycle! Fix it immediatly!");
        return;
      }
    }
  }
  else
  {
    ROS_INFO_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] No start cycle was given, using each cycle element for possible start.");
    for (const ExpandStatesIdx& arr : cycle_)
      start_.push_back(arr);
  }
}

ExpandStatesIdxArray CyclicGaitGenerator::predMovingPatterns(Step::ConstPtr /*step*/, const ExpandStatesIdxArray& next_seq) const
{
  // allow for starting with any leg
  if (next_seq.empty() || next_seq.front().foot_idx.empty())
    return start_;

  const FootIndexArray& next = next_seq.front().foot_idx;
  ROS_ASSERT(pred_.find(next) != pred_.end());
  return ExpandStatesIdxArray{ ExpandStatesIdx{ FootIndexArray{ pred_.find(next)->second }, BaseIndexArray{} } };
}

ExpandStatesIdxArray CyclicGaitGenerator::succMovingPatterns(Step::ConstPtr /*step*/, const ExpandStatesIdxArray& last_seq) const
{
  // allow for starting with any leg
  if (last_seq.empty() || last_seq.back().foot_idx.empty())
    return start_;

  const FootIndexArray& last = last_seq.back().foot_idx;
  ROS_ASSERT(succ_.find(last) != succ_.end());

  return ExpandStatesIdxArray{ ExpandStatesIdx{ FootIndexArray{ succ_.find(last)->second }, BaseIndexArray{} } };
}

bool CyclicGaitGenerator::getExpandStatesIdxArray(const std::string& key, ExpandStatesIdxArray& foot_base)
{
  ROS_ASSERT(robot_description_);

  foot_base.clear();
  MultiFootIndexArray array;

  XmlRpc::XmlRpcValue val;
  if (!getParam(key, val, XmlRpc::XmlRpcValue()))
    return false;

  if (val.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;

  // check if MultiFootIndexArray is given
  if (val[0].getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (!getYamlValue(val, array))
      return false;
  }
  // check if simple FootIndexArray is given
  else
  {
    FootIndexArray idx_arr;
    if (!getYamlValue(val, idx_arr))
      return false;

    for (const FootIndex& idx : idx_arr)
      array.push_back(FootIndexArray{ idx });
  }

  for (const FootIndexArray& idx_arr : array)
  {
    foot_base.push_back(ExpandStatesIdx{ idx_arr, BaseIndexArray{} });
  }

  // consistency checks
  for (const ExpandStatesIdx& idx_arr : foot_base)
  {
    for (const FootIndex& idx : idx_arr.foot_idx)
    {
      if (!robot_description_->hasFootInfo(idx))
      {
        ROS_ERROR_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] Parameter '%s' refers to non-existent foot idx %i!", key.c_str(), idx);
        return false;
      }
    }
  }

  if (array.empty())
  {
    ROS_ERROR_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] Parameter '%s' does not contain any feet!", key.c_str());
    return false;
  }

  return true;
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::CyclicGaitGenerator, l3::GaitGeneratorPlugin)
