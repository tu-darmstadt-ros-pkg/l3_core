#include <l3_libs/types/joint_states.h>

#include <ros/ros.h>

namespace l3
{
void JointStates::clear()
{
  header_ = std_msgs::Header();
  names_.clear();
  positions_.clear();
  velocities_.clear();
  efforts_.clear();
}

void JointStates::updateJoint(const std::string& name, double position, double velocity, double effort)
{
  names_.insert(name);
  positions_[name] = position;
  velocities_[name] = velocity;
  efforts_[name] = effort;
}

void JointStates::updateJoint(const std_msgs::Header& header, const std::string& name, double position, double velocity, double effort)
{
  header_ = header;
  updateJoint(name, position, velocity, effort);
}

void JointStates::updateJoints(const std::vector<std::string>& names, const std::vector<double>& positions, const std::vector<double>& velocities,
                               const std::vector<double>& efforts)
{
  bool use_positions = names.size() == positions.size();
  bool use_velocities = names.size() == velocities.size();
  bool use_efforts = names.size() == efforts.size();

  if (!use_positions && !use_velocities && !use_efforts)
    return;

  for (size_t i = 0; i < names.size(); i++)
  {
    names_.insert(names[i]);

    if (use_positions)
      positions_[names[i]] = positions[i];
    if (use_velocities)
      velocities_[names[i]] = velocities[i];
    if (use_efforts)
      efforts_[names[i]] = efforts[i];
  }
}

void JointStates::updateJoints(const std_msgs::Header& header, const std::vector<std::string>& names, const std::vector<double>& positions, const std::vector<double>& velocities,
                               const std::vector<double>& efforts)
{
  header_ = header;
  updateJoints(names, positions, velocities, efforts);
}
}  // namespace l3
