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
    if (!getCycleFromYaml("cycle", cycle_))
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

  // generate lookup table from given cycle as array
  for (size_t i = 0; i < cycle_.size(); i++)
  {
    // close loop
    if (i == 0)
    {
      pred_[cycle_.front()] = cycle_.back();
      succ_[cycle_.back()] = cycle_.front();
    }
    // connect successive elements
    else
    {
      pred_[cycle_[i]] = cycle_[i - 1];
      succ_[cycle_[i - 1]] = cycle_[i];
    }
  }

  // check start patterns
  if (hasParam("start"))
  {
    if (!getCycleFromYaml("start", start_))
      return;

    // sanity check of input
    for (const ExpandStatesIdx& arr : start_)
    {
      if (pred_.find(arr) == pred_.end())
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
  if (next_seq.empty())
    return start_;

  const ExpandStatesIdx& next = next_seq.front();
  if (next.foot_idx.empty() && next.floating_base_idx.empty())
    return start_;

  ROS_ASSERT(pred_.find(next) != pred_.end());
  return ExpandStatesIdxArray{ pred_.find(next)->second };
}

ExpandStatesIdxArray CyclicGaitGenerator::succMovingPatterns(Step::ConstPtr /*step*/, const ExpandStatesIdxArray& last_seq) const
{
  // allow for starting with any leg
  if (last_seq.empty())
    return start_;

  const ExpandStatesIdx& last = last_seq.back();
  if (last.foot_idx.empty() && last.floating_base_idx.empty())
    return start_;

  ROS_ASSERT(succ_.find(last) != succ_.end());
  return ExpandStatesIdxArray{ succ_.find(last)->second };
}

bool CyclicGaitGenerator::getCycleFromYaml(const std::string& key, ExpandStatesIdxArray& cycle)
{
  ROS_ASSERT(robot_description_);

  cycle.clear();
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
    cycle.push_back(ExpandStatesIdx{ idx_arr, BaseIndexArray{} });
  }

  // consistency checks
  for (const ExpandStatesIdx& idx_arr : cycle)
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
