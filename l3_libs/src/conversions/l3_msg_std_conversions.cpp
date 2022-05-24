#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3
{
void pointArrayMsgToL3(const std::vector<geometry_msgs::Point>& msg, l3::PointArray& p)
{
  p.clear();
  l3::Point l3_p;
  for (const geometry_msgs::Point& msg_p : msg)
  {
    pointMsgToL3(msg_p, l3_p);
    p.push_back(l3_p);
  }
}

void pointArrayL3ToMsg(const l3::PointArray& p, std::vector<geometry_msgs::Point>& msg)
{
  msg.clear();
  geometry_msgs::Point msg_p;
  for (const l3::Point& l3_p : p)
  {
    pointL3ToMsg(l3_p, msg_p);
    msg.push_back(msg_p);
  }
}

void poseArrayMsgToL3(const std::vector<geometry_msgs::Pose>& msg, l3::PoseArray& p)
{
  p.clear();
  l3::Pose l3_p;
  for (const geometry_msgs::Pose& msg_p : msg)
  {
    poseMsgToL3(msg_p, l3_p);
    p.push_back(l3_p);
  }
}

void poseArrayL3ToMsg(const l3::PoseArray& p, std::vector<geometry_msgs::Pose>& msg)
{
  msg.clear();
  geometry_msgs::Pose msg_p;
  for (const l3::Pose& l3_p : p)
  {
    poseL3ToMsg(l3_p, msg_p);
    msg.push_back(msg_p);
  }
}

void vectorArrayMsgToL3(const std::vector<geometry_msgs::Vector3>& msg, l3::Vector3Array& v)
{
  v.clear();
  l3::Vector3 l3_v;
  for (const geometry_msgs::Vector3& msg_p : msg)
  {
    vectorMsgToL3(msg_p, l3_v);
    v.push_back(l3_v);
  }
}

void vectorArrayL3ToMsg(const l3::Vector3Array& v, std::vector<geometry_msgs::Vector3>& msg)
{
  msg.clear();
  geometry_msgs::Vector3 msg_p;
  for (const l3::Vector3& l3_v : v)
  {
    vectorL3ToMsg(l3_v, msg_p);
    msg.push_back(msg_p);
  }
}

void wrenchMsgToL3(const geometry_msgs::Wrench& msg, l3::WrenchData& wrench_data)
{
  vectorMsgToL3(msg.force, wrench_data.force);
  vectorMsgToL3(msg.torque, wrench_data.torque);
}

void wrenchMsgToL3(const geometry_msgs::WrenchStamped& msg, l3::WrenchData& wrench_data)
{
  wrenchMsgToL3(msg.wrench, wrench_data);
  wrench_data.header = msg.header;
}

void wrenchL3ToMsg(const l3::WrenchData& wrench_data, geometry_msgs::Wrench& msg)
{
  vectorL3ToMsg(wrench_data.force, msg.force);
  vectorL3ToMsg(wrench_data.torque, msg.torque);
}

void wrenchL3ToMsg(const l3::WrenchData& wrench_data, geometry_msgs::WrenchStamped& msg)
{
  wrenchL3ToMsg(wrench_data, msg.wrench);
  msg.header = wrench_data.header;
}

void twistMsgToL3(const geometry_msgs::Twist& msg, l3::TwistData& twist_data)
{
  vectorMsgToL3(msg.linear, twist_data.linear);
  vectorMsgToL3(msg.angular, twist_data.angular);
}

void twistMsgToL3(const geometry_msgs::TwistStamped& msg, l3::TwistData& twist_data)
{
  twistMsgToL3(msg.twist, twist_data);
  twist_data.header = msg.header;
}

void twistL3ToMsg(const l3::TwistData& twist_data, geometry_msgs::Twist& msg)
{
  vectorL3ToMsg(twist_data.linear, msg.linear);
  vectorL3ToMsg(twist_data.angular, msg.angular);
}

void twistL3ToMsg(const l3::TwistData& twist_data, geometry_msgs::TwistStamped& msg)
{
  twistL3ToMsg(twist_data, msg.twist);
  msg.header = twist_data.header;
}

void matrixMsgToL3(const boost::array<double, 9>& msg, l3::Matrix3& m) { m << msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7], msg[8]; }

void matrixL3ToMsg(const l3::Matrix3& m, boost::array<double, 9>& msg)
{
  for (int i = 0; i < 9; i++)
    msg[i] = m.coeff(i);
}

void imuMsgToL3(const sensor_msgs::Imu& msg, l3::ImuData& imu_data)
{
  imu_data.header = msg.header;

  quaternionMsgToL3(msg.orientation, imu_data.orientation);
  matrixMsgToL3(msg.orientation_covariance, imu_data.orientation_covariance);

  vectorMsgToL3(msg.angular_velocity, imu_data.angular_velocity);
  matrixMsgToL3(msg.angular_velocity_covariance, imu_data.angular_velocity_covariance);

  vectorMsgToL3(msg.linear_acceleration, imu_data.linear_acceleration);
  matrixMsgToL3(msg.linear_acceleration_covariance, imu_data.linear_acceleration_covariance);
}

void imuL3ToMsg(const l3::ImuData& imu_data, sensor_msgs::Imu& msg)
{
  msg.header = imu_data.header;

  quaternionL3ToMsg(imu_data.orientation, msg.orientation);
  matrixL3ToMsg(imu_data.orientation_covariance, msg.orientation_covariance);

  vectorL3ToMsg(imu_data.angular_velocity, msg.angular_velocity);
  matrixL3ToMsg(imu_data.angular_velocity_covariance, msg.angular_velocity_covariance);

  vectorL3ToMsg(imu_data.linear_acceleration, msg.linear_acceleration);
  matrixL3ToMsg(imu_data.linear_acceleration_covariance, msg.linear_acceleration_covariance);
}

void jointStateMsgToL3(const sensor_msgs::JointState& msg, l3::JointStates& joint_states)
{
  joint_states.clear();
  joint_states.updateJoints(msg.header, msg.name, msg.position, msg.velocity, msg.effort);
}

void jointStateL3ToMsg(const l3::JointStates& joint_states, sensor_msgs::JointState& msg)
{
  msg.header = joint_states.header();
  msg.name = joint_states.names();

  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();
  for (const std::string& name : msg.name)
  {
    msg.position.push_back(joint_states.position(name));
    msg.velocity.push_back(joint_states.velocity(name));
    msg.effort.push_back(joint_states.effort(name));
  }
}
}  // namespace l3
