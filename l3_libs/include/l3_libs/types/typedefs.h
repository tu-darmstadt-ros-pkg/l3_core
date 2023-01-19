//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Felix Sternkopf TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_LIBS_TYPEDEFS_H__
#define L3_LIBS_TYPEDEFS_H__

#include <l3_libs/types/eigen_types.h>

namespace l3
{
template <class T>
struct Stamped
{
  std_msgs::Header header;
  T data;
};

typedef Stamped<Vector2> StampedVector2;
typedef Stamped<Vector3> StampedVector3;
typedef Stamped<Vector4> StampedVector4;
typedef Stamped<Point> StampedPoint;
typedef Stamped<Pose> StampedPose;
typedef Stamped<Pose2D> StampedPose2D;
typedef Stamped<Position> StampedPosition;
typedef Stamped<Position2D> StampedPosition2D;
typedef Stamped<Transform> StampedTransform;

L3_STATIC_ASSERT_MOVEABLE(StampedVector2)
L3_STATIC_ASSERT_MOVEABLE(StampedVector3)
L3_STATIC_ASSERT_MOVEABLE(StampedVector4)
L3_STATIC_ASSERT_MOVEABLE(StampedPoint)
L3_STATIC_ASSERT_MOVEABLE(StampedPose)
L3_STATIC_ASSERT_MOVEABLE(StampedPose2D)
L3_STATIC_ASSERT_MOVEABLE(StampedPosition)
L3_STATIC_ASSERT_MOVEABLE(StampedPosition2D)
L3_STATIC_ASSERT_MOVEABLE(StampedTransform)

struct ImuData
{
  std_msgs::Header header;

  Quaternion orientation;
  Matrix3 orientation_covariance;

  Vector3 angular_velocity;
  Eigen::Matrix3d angular_velocity_covariance;

  Vector3 linear_acceleration;
  Matrix3 linear_acceleration_covariance;
};

struct WrenchData
{
  inline std::vector<double> toList()
  {
    std::vector<double> result;
    result.push_back(force.x());
    result.push_back(force.y());
    result.push_back(force.z());
    result.push_back(torque.x());
    result.push_back(torque.y());
    result.push_back(torque.z());
    return result;
  }

  std_msgs::Header header;

  Vector3 force;
  Vector3 torque;
};

struct TwistData
{
  std_msgs::Header header;

  Vector3 linear;
  Vector3 angular;
};

// assertion to ensure that alle classes are implemented to support movable operation
L3_STATIC_ASSERT_MOVEABLE(ImuData)
L3_STATIC_ASSERT_MOVEABLE(WrenchData)
L3_STATIC_ASSERT_MOVEABLE(TwistData)
}  // namespace l3

#endif
