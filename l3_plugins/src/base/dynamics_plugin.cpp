#include <l3_plugins/base/dynamics_plugin.h>

namespace l3
{
DynamicsPlugin::DynamicsPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name)
{}

void DynamicsPlugin::setRobotDescription(RobotDescription::ConstPtr robot_description) { robot_description_ = robot_description; }
}  // namespace l3
