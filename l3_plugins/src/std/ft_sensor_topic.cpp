#include <l3_plugins/std/ft_sensor_topic.h>

namespace l3
{
FTSensorTopic::FTSensorTopic(const std::string& name)
  : FTSensorPlugin(name)
{}

bool FTSensorTopic::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!FTSensorPlugin::initialize(params))
    return false;

  ft_sub_ = nh_.subscribe(param("topic", std::string("ft")), 1, &FTSensorTopic::wrenchCB, this);

  return true;
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::FTSensorTopic, l3::InterfacePlugin)
