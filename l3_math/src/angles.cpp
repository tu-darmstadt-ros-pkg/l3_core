#include <l3_math/angles.h>

namespace l3
{
void RPYToNormal(double roll, double pitch, double yaw, Vector3& normal)
{
  double sin_roll = sin(roll);
  double sin_pitch = sin(pitch);
  double sin_yaw = sin(yaw);
  double cos_yaw = cos(yaw);

  // rotate around z axis
  normal.x() = cos_yaw * sin_pitch + sin_yaw * sin_roll;
  normal.y() = sin_yaw * sin_pitch - cos_yaw * sin_roll;
  normal.z() = sqrt(1.0 - normal.x() * normal.x() + normal.y() * normal.y());
}

void RPYToNormal(double roll, double pitch, double yaw, geometry_msgs::Vector3& normal)
{
  double sin_roll = sin(roll);
  double sin_pitch = sin(pitch);
  double sin_yaw = sin(yaw);
  double cos_yaw = cos(yaw);

  // rotate around z axis
  normal.x = cos_yaw * sin_pitch + sin_yaw * sin_roll;
  normal.y = sin_yaw * sin_pitch - cos_yaw * sin_roll;
  normal.z = sqrt(1.0 - normal.x * normal.x + normal.y * normal.y);
}

void normalToRP(const Vector3& normal, double yaw, double& roll, double& pitch)
{
  // inverse rotation around z axis
  double sin_yaw = sin(-yaw);
  double cos_yaw = cos(-yaw);

  roll = -asin(sin_yaw * normal.x() + cos_yaw * normal.y());
  pitch = asin(cos_yaw * normal.x() - sin_yaw * normal.y());
}

void normalToRP(const geometry_msgs::Vector3& normal, double yaw, double& roll, double& pitch)
{
  // inverse rotation around z axis
  double sin_yaw = sin(-yaw);
  double cos_yaw = cos(-yaw);

  roll = -asin(sin_yaw * normal.x + cos_yaw * normal.y);
  pitch = asin(cos_yaw * normal.x - sin_yaw * normal.y);
}
}  // namespace l3
