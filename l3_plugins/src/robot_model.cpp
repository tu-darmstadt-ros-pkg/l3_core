#include <l3_plugins/robot_model.h>

#include <l3_libs/helper.h>

namespace l3
{
RobotModel::RobotModel() {}

bool RobotModel::initialize(const XmlRpc::XmlRpcValue& params)
{
  // register plugins
  vigir_pluginlib::PluginManager::addPluginClassLoader<KinematicsPlugin>("l3_plugins", "l3::KinematicsPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<DynamicsPlugin>("l3_plugins", "l3::DynamicsPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<GaitGeneratorPlugin>("l3_plugins", "l3::GaitGeneratorPlugin");

  // parse params
  XmlRpc::XmlRpcValue p = params;
  if (!p.valid() || p.size() == 0)
  {
    ROS_ERROR_NAMED("RobotModel", "[RobotModel] No valid robot model config was given.");
    return false;
  }

  // load parameters
  if (!mutableInstance().parseConfig(p))
  {
    ROS_ERROR_NAMED("RobotModel", "[RobotModel] Error while parsing parameters!");
    return false;
  }

  return true;
}

bool RobotModel::initialize(ros::NodeHandle& nh, const std::string& topic)
{
  // register plugins
  vigir_pluginlib::PluginManager::addPluginClassLoader<KinematicsPlugin>("l3_plugins", "l3::KinematicsPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<DynamicsPlugin>("l3_plugins", "l3::DynamicsPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<GaitGeneratorPlugin>("l3_plugins", "l3::GaitGeneratorPlugin");

  l3_msgs::RobotModel::ConstPtr model_msg;

  while (!model_msg)
  {
    model_msg = ros::topic::waitForMessage<l3_msgs::RobotModel>(topic, nh, ros::Duration(5.0));

    if (!model_msg)
      ROS_WARN("[RobotModel] No robot model received at topic '%s' yet!", ros::names::append(nh.getNamespace(), topic).c_str());
  }

  mutableInstance().fromMsg(*model_msg);
  return true;
}

void RobotModel::fromMsg(const l3_msgs::RobotModel& msg)
{
  UniqueLock lock(mutex_);

  robot_description_ = RobotDescription::Ptr(new RobotDescription(msg.robot_description));

  // load plugins
  if (!msg.kinematics_plugin.name.empty())
    vigir_pluginlib::PluginManager::addPlugin(msg.kinematics_plugin);
  if (!msg.dynamics_plugin.name.empty())
    vigir_pluginlib::PluginManager::addPlugin(msg.dynamics_plugin);
  if (!msg.gait_generator_plugin.name.empty())
    vigir_pluginlib::PluginManager::addPlugin(msg.gait_generator_plugin);

  lock.unlock();

  loadPlugins();
}

void RobotModel::toMsg(l3_msgs::RobotModel& msg) const
{
  SharedLock lock(mutex_);

  if (robot_description_)
    robot_description_->toMsg(msg.robot_description);

  if (kinematics_)
    msg.kinematics_plugin = kinematics_->getDescription();
  if (dynamics_)
    msg.dynamics_plugin = dynamics_->getDescription();
  if (gait_generator_)
    msg.gait_generator_plugin = gait_generator_->getDescription();
}

bool RobotModel::loadPlugins()
{
  bool result = true;

  result &= loadKinematicsPlugin();
  result &= loadDynamicsPlugin();
  result &= loadGaitGeneratorPlugin();

  printPluginSummary();

  return result;
}

bool RobotModel::loadKinematicsPlugin()
{
  UniqueLock lock(instance().mutex_);

  // get kinematics plugins
  if (vigir_pluginlib::PluginManager::hasPluginsByBaseClass("l3::KinematicsPlugin"))
    vigir_pluginlib::PluginManager::getPlugin(mutableInstance().kinematics_);

  return true;
}

bool RobotModel::loadDynamicsPlugin()
{
  UniqueLock lock(instance().mutex_);

  // get dynamics plugins
  if (vigir_pluginlib::PluginManager::hasPluginsByBaseClass("l3::DynamicsPlugin"))
    vigir_pluginlib::PluginManager::getPlugin(mutableInstance().dynamics_);

  return true;
}

bool RobotModel::loadGaitGeneratorPlugin()
{
  UniqueLock lock(instance().mutex_);

  // get gait generator
  if (vigir_pluginlib::PluginManager::hasPluginsByBaseClass("l3::GaitGeneratorPlugin"))
  {
    vigir_pluginlib::PluginManager::getPlugin(mutableInstance().gait_generator_);
    instance().gait_generator_->setRobotDescription(instance().robot_description_);
  }

  return true;
}

void RobotModel::printPluginSummary()
{
  SharedLock lock(instance().mutex_);

  ROS_INFO("[RobotModel] Plugins loaded:");

  // check kinematics plugins
  if (instance().kinematics_)
    ROS_INFO("    %s", instance().kinematics_->getName().c_str());
  else
    ROS_WARN("    No KinematicsPlugin loaded!");

  // check dynamics plugins
  if (instance().dynamics_)
    ROS_INFO("    %s", instance().dynamics_->getName().c_str());
  else
    ROS_INFO("    No DynamicsPlugin loaded!");

  // get gait generator
  if (instance().gait_generator_)
    ROS_INFO("    %s", instance().gait_generator_->getName().c_str());
  else
    ROS_WARN("    No GaitGeneratorPlugin loaded!");
}

bool RobotModel::parseConfig(XmlRpc::XmlRpcValue& params)
{
  if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("RobotModel", "[RobotModel] Parameters have wrong format.");
    return false;
  }

  /// Parse description section
  if (!params.hasMember("description"))
  {
    ROS_ERROR_NAMED("RobotModel", "[RobotModel] Missing 'description' tag in parameters.");
    return false;
  }

  UniqueLock lock(instance().mutex_);
  mutableInstance().robot_description_ = RobotDescription::Ptr(new RobotDescription(params["description"]));
  lock.unlock();

  /// Parse plugins section (optional)
  if (!params.hasMember("plugins"))
    return loadPlugins();

  XmlRpc::XmlRpcValue& plugins = params["plugins"];
  vigir_pluginlib_msgs::PluginDescription desc;

  // obtain kinematics plugin
  if (plugins.hasMember("kinematics"))
  {
    if (!vigir_pluginlib::PluginManager::parsePluginDescription(desc, plugins["kinematics"]))
    {
      ROS_ERROR_NAMED("RobotModel", "[RobotModel] 'kinematics' plugin description malformed.");
      return false;
    }
    vigir_pluginlib::PluginManager::addPlugin(desc);
  }

  // obtain dynamics plugin
  if (plugins.hasMember("dynamics"))
  {
    if (!vigir_pluginlib::PluginManager::parsePluginDescription(desc, plugins["dynamics"]))
    {
      ROS_ERROR_NAMED("RobotModel", "[RobotModel] 'dynamics' plugin description malformed.");
      return false;
    }
    vigir_pluginlib::PluginManager::addPlugin(desc);
  }

  // obtain gait generator plugin
  if (plugins.hasMember("gait_generator"))
  {
    if (!vigir_pluginlib::PluginManager::parsePluginDescription(desc, plugins["gait_generator"]))
    {
      ROS_ERROR_NAMED("RobotModel", "[RobotModel] 'gait_generator' plugin description malformed.");
      return false;
    }
    vigir_pluginlib::PluginManager::addPlugin(desc);
  }

  return loadPlugins();
}

Pose RobotModel::calcFeetCenter(const FootholdArray& footholds)
{
  SharedLock lock(instance().mutex_);

  if (instance().kinematics_)
    return instance().kinematics_->calcFeetCenter(footholds);
  else
    return l3::calcFeetCenter(footholds);
}

Pose RobotModel::calcFeetCenter(const FootholdConstPtrArray& footholds)
{
  SharedLock lock(instance().mutex_);

  if (instance().kinematics_)
    return instance().kinematics_->calcFeetCenter(footholds);
  else
    return l3::calcFeetCenter(footholds);
}
}  // namespace l3
