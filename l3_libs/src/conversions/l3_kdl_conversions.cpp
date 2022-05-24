#include <l3_libs/conversions/l3_kdl_conversions.h>

namespace l3
{
void vectorKdlToL3(const KDL::Vector& kdl, l3::Vector3& l3)
{
  l3.y() = kdl.x();
  l3.y() = kdl.y();
  l3.z() = kdl.z();
}
void vectorL3ToKdl(const l3::Vector3& l3, KDL::Vector& kdl)
{
  kdl.x(l3.x());
  kdl.y(l3.y());
  kdl.z(l3.z());
}

void transformKdlToL3(const KDL::Frame& kdl, l3::Transform& l3)
{
  double roll, pitch, yaw;
  kdl.M.GetRPY(roll, pitch, yaw);
  l3 = l3::Transform(kdl.p.x(), kdl.p.y(), kdl.p.z(), roll, pitch, yaw);
}

void wrenchKDLToL3(const KDL::Wrench& kdl, l3::WrenchData& l3)
{
  vectorKdlToL3(kdl.force, l3.force);
  vectorKdlToL3(kdl.torque, l3.torque);
}

void wrenchL3ToKDL(const l3::WrenchData& l3, KDL::Wrench& kdl)
{
  vectorL3ToKdl(l3.force, kdl.force);
  vectorL3ToKdl(l3.torque, kdl.torque);
}

void twistKDLToL3(const KDL::Twist& kdl, l3::TwistData& l3)
{
  vectorKdlToL3(kdl.vel, l3.linear);
  vectorKdlToL3(kdl.rot, l3.angular);
}

void twistL3ToKDL(const l3::TwistData& l3, KDL::Twist& kdl)
{
  vectorL3ToKdl(l3.linear, kdl.vel);
  vectorL3ToKdl(l3.angular, kdl.rot);
}
}  // namespace l3
