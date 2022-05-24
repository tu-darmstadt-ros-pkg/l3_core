#include <l3_plugins/base/ft_sensor_plugin.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

namespace l3
{
FTSensorPlugin::FTSensorPlugin(const std::string& name)
  : InterfacePlugin(name)
{}

bool FTSensorPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!InterfacePlugin::initialize(params))
    return false;

  return true;
}
}  // namespace l3
