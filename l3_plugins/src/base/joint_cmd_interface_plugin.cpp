#include <l3_plugins/base/joint_cmd_interface_plugin.h>

namespace l3
{
JointCmdInterfacePlugin::JointCmdInterfacePlugin(const std::string& name)
  : InterfacePlugin(name)
{}

bool JointCmdInterfacePlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!InterfacePlugin::loadParams(params))
    return false;

  mode_ = static_cast<Mode>(param("mode", static_cast<int>(Mode::POSITION), true));
  return true;
}
}  // namespace l3
