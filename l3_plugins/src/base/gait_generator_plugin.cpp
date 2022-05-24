#include <l3_plugins/base/gait_generator_plugin.h>

namespace l3
{
GaitGeneratorPlugin::GaitGeneratorPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name)
{}

void GaitGeneratorPlugin::setRobotDescription(RobotDescription::ConstPtr robot_description) { robot_description_ = robot_description; }
}  // namespace l3
