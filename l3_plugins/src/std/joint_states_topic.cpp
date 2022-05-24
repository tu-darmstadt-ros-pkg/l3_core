#include <l3_plugins/std/joint_states_topic.h>

#include <l3_libs/conversions/l3_msg_conversions.h>

namespace l3
{
JointStatesTopic::JointStatesTopic(const std::string& name)
  : JointStatesPlugin(name)
{}

bool JointStatesTopic::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!JointStatesPlugin::initialize(params))
    return false;

  joint_states_sub_ = nh_.subscribe(param("topic", std::string("joint_states")), 1, &JointStatesTopic::jointStateCb, this);
  return true;
}

void JointStatesTopic::jointStateCb(const sensor_msgs::JointStateConstPtr msg)
{
  UniqueLock lock(plugin_mutex_);
  jointStateMsgToL3(*msg, joint_states_);
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::JointStatesTopic, l3::InterfacePlugin)
