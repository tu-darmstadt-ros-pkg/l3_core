#include <l3_libs/robot_description/robot_description_server.h>

namespace l3
{
RobotDescriptionServer::RobotDescriptionServer()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get local params
  std::string robot_model_path = pnh.param("robot_model_path", std::string("robot_model"));

  XmlRpc::XmlRpcValue params;
  if (!nh.getParam(robot_model_path, params))
  {
    ROS_ERROR_NAMED("RobotDescriptionServer", "[RobotDescriptionServer] No robot model config found at rosparam server. Lookup path '%s'",
                    ros::names::append(nh.getNamespace(), robot_model_path).c_str());
    return;
  }

  if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("RobotDescriptionServer", "[RobotDescriptionServer] Parameters have wrong format.");
    return;
  }

  if (!params.hasMember("description"))
  {
    ROS_ERROR_NAMED("RobotDescriptionServer", "[RobotDescriptionServer] Missing robot description in parameters.");
    return;
  }

  // init robot configuration
  robot_description_ = RobotDescription::Ptr(new RobotDescription(params["description"]));
  if (!robot_description_->isValid())
  {
    ROS_ERROR_NAMED("RobotDescriptionServer", "[RobotDescriptionServer] No valid robot description available!");
    return;
  }

  robot_description_pub_ = nh.advertise<l3_msgs::RobotDescription>("robot_description", 1, true);
  robot_description_pub_.publish(robot_description_->toMsg());
}
}  // namespace l3

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_description_server");
  l3::RobotDescriptionServer node;
  ros::spin();

  return 0;
}
