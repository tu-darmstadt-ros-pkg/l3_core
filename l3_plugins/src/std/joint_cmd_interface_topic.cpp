#include <l3_plugins/std/joint_cmd_interface_topic.h>

#include <std_msgs/Float64.h>

namespace l3
{
JointCmdInterfaceTopic::JointCmdInterfaceTopic(const std::string& name)
  : JointCmdInterfacePlugin(name)
{}

bool JointCmdInterfaceTopic::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!JointCmdInterfacePlugin::initialize(params))
    return false;

  joint_names_.clear();
  XmlRpc::XmlRpcValue joint_names = param("joints", XmlRpc::XmlRpcValue());
  if (joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < joint_names.size(); i++)
    {
      const std::string& name = static_cast<std::string>(joint_names[i]);
      joint_names_.push_back(name);
    }
  }
  else
  {
    ROS_ERROR("[JointCmdInterface] Joints must be given as an array of strings.");
    return false;
  }

  joint_cmd_pub_.clear();

  std::string ns = param("ns", std::string(), true);

  std::string suffix;
  if (!getParam("suffix", suffix, std::string(), true))
  {
    switch (mode_)
    {
      case Mode::POSITION:
        suffix = "_position/command";
        break;
      case Mode::VELOCITY:
        suffix = "_velocity/command";
        break;
      case Mode::EFFORT:
        suffix = "_effort/command";
        break;
    }
  }

  int queue_size = param("queue_size", 1, true);

  for (size_t i = 0; i < joint_names_.size(); i++)
    joint_cmd_pub_.push_back(nh_.advertise<std_msgs::Float64>(ros::names::clean(ns + "/" + joint_names_[i] + suffix), queue_size));

  return true;
}

void JointCmdInterfaceTopic::sendJointCmd() const
{
  std_msgs::Float64 cmd;

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    const std::string& name = joint_names_[i];

    if (!joint_states_.hasJoint(name))
    {
      ROS_WARN_THROTTLE(1.0, "[JointCmdTopic] No command for joint '%s' is set.", name.c_str());
      continue;
    }

    switch (mode_)
    {
      case Mode::POSITION:
        cmd.data = joint_states_.position(name);
        break;
      case Mode::VELOCITY:
        cmd.data = joint_states_.velocity(name);
        break;
      case Mode::EFFORT:
        cmd.data = joint_states_.effort(name);
        break;
    }

    joint_cmd_pub_[i].publish(cmd);
  }
}
}  // namespace l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3::JointCmdInterfaceTopic, l3::InterfacePlugin)
