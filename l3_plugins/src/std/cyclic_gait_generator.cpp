#include <l3_plugins/std/cyclic_gait_generator.h>

#include <l3_libs/yaml_parser.h>

#include <l3_plugins/robot_model.h>

namespace l3
{
CyclicGaitGenerator::CyclicGaitGenerator()
  : GaitGeneratorPlugin("cyclic_gait_generator")
{}

bool CyclicGaitGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GaitGeneratorPlugin::loadParams(params))
    return false;

  getParam("ignore_floating_base", ignore_floating_base_, false, true);

  loadCycle();

  return true;
}

void CyclicGaitGenerator::setRobotDescription(RobotDescription::ConstPtr robot_description)
{
  GaitGeneratorPlugin::setRobotDescription(robot_description);

  loadCycle();
}

ExpandStatesIdxArray CyclicGaitGenerator::predMovingPatterns(Step::ConstPtr /*step*/, const ExpandStatesIdxArray& next_seq) const
{
  // allow for starting with any leg
  if (next_seq.empty())
    return start_;

  ExpandStatesIdx next = next_seq.front();
  if (next.foot_idx.empty() && next.floating_base_idx.empty())
    return start_;

  if (ignore_floating_base_)
    next.floating_base_idx.clear();

  // search next moving pattern
  std::map<ExpandStatesIdx, ExpandStatesIdx>::const_iterator itr = pred_.find(next);
  if (itr != succ_.end())
    return ExpandStatesIdxArray{ itr->second };
  else
  {
    ROS_WARN("[%s] No predecessor pattern '%s' defined! Fix it immediatly!", getName().c_str(), toString(next).c_str());
    return ExpandStatesIdxArray();
  }
}

ExpandStatesIdxArray CyclicGaitGenerator::succMovingPatterns(Step::ConstPtr /*step*/, const ExpandStatesIdxArray& last_seq) const
{
  // allow for starting with any leg
  if (last_seq.empty())
    return start_;

  ExpandStatesIdx last = last_seq.back();
  if (last.foot_idx.empty() && last.floating_base_idx.empty())
    return start_;

  if (ignore_floating_base_)
    last.floating_base_idx.clear();

  // search next moving pattern
  std::map<ExpandStatesIdx, ExpandStatesIdx>::const_iterator itr = succ_.find(last);
  if (itr != succ_.end())
    return ExpandStatesIdxArray{ itr->second };
  else
  {
    ROS_WARN("[%s] No successor for pattern '%s' defined! Fix it immediatly!", getName().c_str(), toString(last).c_str());
    return ExpandStatesIdxArray();
  }
}

bool CyclicGaitGenerator::loadCycle()
{
  if (!robot_description_)
    return false;

  cycle_.clear();
  succ_.clear();
  pred_.clear();
  start_.clear();

  if (hasParam("cycle"))
  {
    if (!getCycleFromYaml("cycle", cycle_))
      return false;
  }
  else
  {
    ROS_INFO_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] No cycle was given, generating simple foot cycling.");
    for (const FootInfoPair& p : robot_description_->getFootInfoMap())
    {
      if (!p.second.indirect)
        cycle_.push_back(ExpandStatesIdx{ FootIndexArray{ p.first }, BaseIndexArray{} });
    }

    if (cycle_.empty())
    {
      ROS_ERROR_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] RobotDescription does not contain any feet!");
      return false;
    }
  }

  // output cycle
  ROS_INFO_STREAM("Cycle: " << toString(cycle_));

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
      return false;

    // sanity check of input
    for (const ExpandStatesIdx& arr : start_)
    {
      if (pred_.find(arr) == pred_.end())
      {
        ROS_ERROR_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] At least one input start option does not match with any element of the given cycle! Fix it immediatly!");
        return false;
      }
    }
  }
  else
  {
    ROS_INFO_NAMED("CyclicGaitGenerator", "[CyclicGaitGenerator] No start cycle was given, using each cycle element for possible start.");
    for (const ExpandStatesIdx& arr : cycle_)
      start_.push_back(arr);
  }

  return true;
}

bool CyclicGaitGenerator::getCycleFromYaml(const std::string& key, ExpandStatesIdxArray& cycle)
{
  ROS_ASSERT(robot_description_);

  cycle.clear();

  XmlRpc::XmlRpcValue cycle_params;
  if (!getParam(key, cycle_params, XmlRpc::XmlRpcValue()))
    return false;

  /// Foot cycle only
  /// case 1: [0, 1, 2, 3]        # (list) Single-leg cycle
  /// case 2: [[0, 2], [1, 3]]    # (list of list) Multi-leg cycle

  /// With floating base cycle
  /// case 3: [{fh: [0], fb: [0]}, {fh: [1, 2], fb: [0]}, ...]    # (list of dicts (of lists))
  /// or
  /// - {fh: 0}
  /// - {fb: 0}
  /// - {fh: 0, fb: 0}
  /// - {fh: [0], fb: [0]}
  /// - {fh: [1, 2], fb: [0]}

  if (cycle_params.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;

  /// detect structure of config and generate pattern map

  // check if simple FootIndexArray is given
  if (cycle_params[0].getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    FootIndexArray idx_arr;
    if (!getYamlValue(cycle_params, idx_arr))
      return false;

    for (const FootIndex& idx : idx_arr)
      cycle.push_back(ExpandStatesIdx{ FootIndexArray{ idx }, BaseIndexArray{} });
  }
  // check if MultiFootIndexArray is given
  else if (cycle_params[0].getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < cycle_params.size(); i++)
    {
      XmlRpc::XmlRpcValue p = cycle_params[i];

      // check type
      if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_NAMED(getName(), "[%s] Mulit-foot cycle requires each element to be a list. Element at index %lu is from type '%s'!", getName().c_str(), i,
                        l3::toString(p.getType()).c_str());
        return false;
      }

      // extract foot index array
      ExpandStatesIdx expand_idx;
      if (!getYamlValue(p, expand_idx.foot_idx))
        return false;

      cycle.push_back(expand_idx);
    }
  }
  // check if floating base is included
  else if (cycle_params[0].getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (size_t i = 0; i < cycle_params.size(); i++)
    {
      XmlRpc::XmlRpcValue p = cycle_params[i];

      // check type
      if (p.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_NAMED(getName(), "[%s] Floating base cycle requires each element to be a dict. Element at index %lu is from type '%s'!", getName().c_str(), i,
                        l3::toString(p.getType()).c_str());
        return false;
      }

      ExpandStatesIdx expand_idx;

      // extract foot index array
      if (p.hasMember("fh"))
      {
        XmlRpc::XmlRpcValue fh = p["fh"];

        if (fh.getType() == XmlRpc::XmlRpcValue::TypeInt)
          expand_idx.foot_idx.push_back(static_cast<int>(fh));
        else if (fh.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          if (!getYamlValue(fh, expand_idx.foot_idx))
            return false;
        }
      }

      // extract floating base index array
      if (p.hasMember("fb"))
      {
        XmlRpc::XmlRpcValue fb = p["fb"];

        if (fb.getType() == XmlRpc::XmlRpcValue::TypeInt)
          expand_idx.floating_base_idx.push_back(static_cast<int>(fb));
        else if (fb.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          if (!getYamlValue(fb, expand_idx.floating_base_idx))
            return false;
        }
      }

      cycle.push_back(expand_idx);
    }
  }
  // invalid format
  else
  {
    ROS_ERROR_NAMED(getName(), "[%s] Config has invalid format!", getName().c_str());
    return false;
  }

  /// consistency checks
  for (const ExpandStatesIdx& exp_idx : cycle)
  {
    for (const FootIndex& idx : exp_idx.foot_idx)
    {
      if (!robot_description_->hasFootInfo(idx))
      {
        ROS_ERROR_NAMED(getName(), "[%s] Config refers to non-existent foot idx %i!", getName().c_str(), idx);
        return false;
      }
    }

    for (const BaseIndex& idx : exp_idx.floating_base_idx)
    {
      if (!robot_description_->hasBaseInfo(idx))
      {
        ROS_ERROR_NAMED(getName(), "[%s] Config refers to non-existent base idx %i!", getName().c_str(), idx);
        return false;
      }
    }
  }

  if (cycle.empty())
  {
    ROS_ERROR_NAMED(getName(), "[%s] Config does not contain any element!", getName().c_str());
    return false;
  }

  return true;
}

std::string CyclicGaitGenerator::toString(const ExpandStatesIdxArray& cycle)
{
  std::stringstream ss;

  for (const ExpandStatesIdx& step : cycle)
    ss << toString(step) << " -> ";

  ss << "...";

  return ss.str();
}

std::string CyclicGaitGenerator::toString(const ExpandStatesIdx& step)
{
  std::stringstream ss;

  ss << "[ ";

  if (step.foot_idx.empty())
    ss << "- ";
  for (const FootIndex& idx : step.foot_idx)
    ss << idx << " ";

  ss << "| ";

  if (step.floating_base_idx.empty())
    ss << "- ";
  for (const BaseIndex& idx : step.floating_base_idx)
    ss << idx << " ";

  ss << "]";

  return ss.str();
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::CyclicGaitGenerator, l3::GaitGeneratorPlugin)
