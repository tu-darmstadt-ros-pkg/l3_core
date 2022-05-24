#include <l3_plugins/aggregator/joint_cmd_interface_aggregator.h>

namespace l3
{
JointCmdInterfaceAggregator::JointCmdInterfaceAggregator(const std::string& name)
  : vigir_pluginlib::PluginAggregator<JointCmdInterfacePlugin>(name)
{}

void JointCmdInterfaceAggregator::updateJoint(const std::string& name, double position, double velocity, double effort)
{
  for (JointCmdInterfacePlugin::Ptr plugin : plugins_)
    plugin->updateJoint(name, position, velocity, effort);
}
void JointCmdInterfaceAggregator::updateJoint(const std_msgs::Header& header, const std::string& name, double position, double velocity, double effort)
{
  for (JointCmdInterfacePlugin::Ptr plugin : plugins_)
    plugin->updateJoint(header, name, position, velocity, effort);
}

void JointCmdInterfaceAggregator::updateJoints(const std::vector<std::string>& names, const std::vector<double>& positions, const std::vector<double>& velocities,
                                               const std::vector<double>& efforts)
{
  for (JointCmdInterfacePlugin::Ptr plugin : plugins_)
    plugin->updateJoints(names, positions, velocities, efforts);
}
void JointCmdInterfaceAggregator::updateJoints(const std_msgs::Header& header, const std::vector<std::string>& names, const std::vector<double>& positions,
                                               const std::vector<double>& velocities, const std::vector<double>& efforts)
{
  for (JointCmdInterfacePlugin::Ptr plugin : plugins_)
    plugin->updateJoints(header, names, positions, velocities, efforts);
}

void JointCmdInterfaceAggregator::setJointStates(const JointStates& joint_states)
{
  for (JointCmdInterfacePlugin::Ptr plugin : plugins_)
    plugin->setJointStates(joint_states);
}

std::vector<std::string> JointCmdInterfaceAggregator::getJointNames() const
{
  std::vector<std::string> names;

  for (JointCmdInterfacePlugin::Ptr plugin : plugins_)
  {
    std::vector<std::string> n = plugin->getJointNames();
    names.insert(names.end(), n.begin(), n.end());
  }

  return names;
}

void JointCmdInterfaceAggregator::sendJointCmd() const
{
  for (JointCmdInterfacePlugin::Ptr plugin : plugins_)
    plugin->sendJointCmd();
}
}  // namespace l3
