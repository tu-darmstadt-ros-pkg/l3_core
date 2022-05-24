#include <l3_plugins/robot_model_server.h>

namespace l3
{
RobotModelServer::RobotModelServer()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  vigir_pluginlib::PluginManager::initialize(nh);

  // get local params
  std::string robot_model_path = pnh.param("robot_model_path", std::string("robot_model"));

  XmlRpc::XmlRpcValue params;
  if (!nh.getParam(robot_model_path, params))
  {
    ROS_ERROR_NAMED("RobotModelServer", "[RobotModelServer] No robot model config found at rosparam server. Lookup path '%s'",
                    ros::names::append(nh.getNamespace(), robot_model_path).c_str());
    return;
  }

  if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("RobotModelServer", "[RobotModelServer] Parameters have wrong format.");
    return;
  }

  if (!RobotModel::initialize(params))
  {
    ROS_ERROR("[RobotModelServer] Initialization of RobotModel failed!");
    return;
  }

  robot_description_pub_ = nh.advertise<l3_msgs::RobotDescription>("robot_description", 1, true);
  robot_description_pub_.publish(RobotModel::instance().description()->toMsg());

  robot_model_pub_ = nh.advertise<l3_msgs::RobotModel>("robot_model", 1, true);
  robot_model_pub_.publish(RobotModel::instance().toMsg());
}
}  // namespace l3

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_server");
  l3::RobotModelServer node;
  ros::spin();

  return 0;
}
