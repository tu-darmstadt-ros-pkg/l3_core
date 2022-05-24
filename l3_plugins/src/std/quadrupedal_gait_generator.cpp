#include <l3_plugins/std/quadrupedal_gait_generator.h>

namespace l3
{
QuadrupedalGaitGenerator::QuadrupedalGaitGenerator()
  : GaitGeneratorPlugin("quadruped_gait_generator")
{}

void QuadrupedalGaitGenerator::setRobotDescription(RobotDescription::ConstPtr robot_description)
{
  GaitGeneratorPlugin::setRobotDescription(robot_description);

  if (robot_description_->footIdxList().size() != 4)
  {
    ROS_WARN_NAMED("QuadrupedalGaitGenerator",
                   "[QuadrupedalGaitGenerator] Current robot description is not quadrupedal! Expected exactly 4 feet, %lu feet are given. The gait generator may not work as "
                   "expected.",
                   robot_description_->footIdxList().size());
  }
}

ExpandStatesIdxArray QuadrupedalGaitGenerator::predMovingPatterns(Step::ConstPtr step, const ExpandStatesIdxArray& next_seq) const
{
  // basically the same as forward case
  if (next_seq.empty())
    return succMovingPatterns(step, ExpandStatesIdxArray());
  else
    return succMovingPatterns(step, next_seq);
}

ExpandStatesIdxArray QuadrupedalGaitGenerator::succMovingPatterns(Step::ConstPtr /*step*/, const ExpandStatesIdxArray& last_seq) const
{
  ROS_ASSERT(robot_description_);
  ROS_ASSERT(robot_description_->footIdxList().size() == 4);  // check for consistent model (four legs)

  ExpandStatesIdxArray patterns;

  // trot gait
  if (last_seq.empty() || last_seq.back().foot_idx.empty())
  {
    patterns.push_back(ExpandStatesIdx{ FootIndexArray {robot_description_->footIdxList()[0], robot_description_->footIdxList()[3]}, BaseIndexArray{} });
    patterns.push_back(ExpandStatesIdx{ FootIndexArray {robot_description_->footIdxList()[1], robot_description_->footIdxList()[2]}, BaseIndexArray{} });
    return patterns;
  }

  // alternating feet
  if (robot_description_->footIdxList()[0] == last_seq.back().foot_idx.back())
    patterns.push_back(ExpandStatesIdx{ FootIndexArray {robot_description_->footIdxList()[1], robot_description_->footIdxList()[2]}, BaseIndexArray{} });
  else
    patterns.push_back(ExpandStatesIdx{ FootIndexArray {robot_description_->footIdxList()[0], robot_description_->footIdxList()[3]}, BaseIndexArray{} });

  return patterns;
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::QuadrupedalGaitGenerator, l3::GaitGeneratorPlugin)
