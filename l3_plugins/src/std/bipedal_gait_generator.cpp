#include <l3_plugins/std/bipedal_gait_generator.h>

namespace l3
{
BipedalGaitGenerator::BipedalGaitGenerator()
  : GaitGeneratorPlugin("bipedal_gait_generator")
{}

void BipedalGaitGenerator::setRobotDescription(RobotDescription::ConstPtr robot_description)
{
  GaitGeneratorPlugin::setRobotDescription(robot_description);

  if (robot_description_->footIdxList().size() != 2)
  {
    ROS_WARN_NAMED("BipedalGaitGenerator",
                   "[BipedalGaitGenerator] Current robot description is not bipedal! Expected exactly 2 feet, %lu feet are given. The gait generator may not work as expected.",
                   robot_description_->footIdxList().size());
  }
}

ExpandStatesIdxArray BipedalGaitGenerator::predMovingPatterns(Step::ConstPtr step, const ExpandStatesIdxArray& next_seq) const
{
  // basically the same as forward case
  if (next_seq.empty())
    return succMovingPatterns(step, ExpandStatesIdxArray());
  else
    return succMovingPatterns(step, ExpandStatesIdxArray{ ExpandStatesIdx(FootIndexArray{ next_seq.front().foot_idx }, BaseIndexArray{}) });
}

ExpandStatesIdxArray BipedalGaitGenerator::succMovingPatterns(Step::ConstPtr /*step*/, const ExpandStatesIdxArray& last_seq) const
{
  ROS_ASSERT(robot_description_);
  ROS_ASSERT(robot_description_->footIdxList().size() == 2);  // check for consistent model (two legs)

  ExpandStatesIdxArray patterns;

  if (last_seq.empty() || last_seq.back().foot_idx.empty())
  {
    patterns.push_back(ExpandStatesIdx{ FootIndexArray{ robot_description_->footIdxList()[0] }, BaseIndexArray{} });
    patterns.push_back(ExpandStatesIdx{ FootIndexArray{ robot_description_->footIdxList()[1] }, BaseIndexArray{} });
    return patterns;
  }

  // alternating feet
  if (robot_description_->footIdxList()[0] == last_seq.back().foot_idx[0])
    patterns.push_back(ExpandStatesIdx{ FootIndexArray{ robot_description_->footIdxList()[1] }, BaseIndexArray{} });
  else
    patterns.push_back(ExpandStatesIdx{ FootIndexArray{ robot_description_->footIdxList()[0] }, BaseIndexArray{} });

  return patterns;
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::BipedalGaitGenerator, l3::GaitGeneratorPlugin)
