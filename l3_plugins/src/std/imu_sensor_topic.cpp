#include <l3_plugins/std/imu_sensor_topic.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

namespace l3
{
IMUSensorTopic::IMUSensorTopic(const std::string& name)
  : IMUSensorPlugin(name)
{}

bool IMUSensorTopic::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!IMUSensorPlugin::initialize(params))
    return false;

  imu_sub_ = nh_.subscribe(param("topic", std::string("imu")), 1, &IMUSensorTopic::imuCb, this);
  return true;
}

void IMUSensorTopic::imuCb(const sensor_msgs::ImuConstPtr msg)
{
  UniqueLock lock(plugin_mutex_);
  imuMsgToL3(*msg, imu_data_);
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::IMUSensorTopic, l3::InterfacePlugin)
