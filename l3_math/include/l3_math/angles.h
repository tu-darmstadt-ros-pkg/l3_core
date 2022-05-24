//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_MATH_ANGLES_H__
#define L3_MATH_ANGLES_H__

#include <cmath>

#include <angles/angles.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <l3_libs/types/types.h>

namespace l3
{
constexpr double toRadians(double degrees) { return degrees * M_PI / 180.0; }

constexpr double toDegrees(double radians) { return radians * 180.0 / M_PI; }

inline double normalizeAnglePositive(double angle) { return angles::normalize_angle_positive(angle); }

inline double normalizeAngle(double angle) { return angles::normalize_angle(angle); }

inline double shortestAngularDistance(double from, double to) { return angles::shortest_angular_distance(from, to); }

void RPYToNormal(double roll, double pitch, double yaw, Vector3& normal);
void RPYToNormal(double roll, double pitch, double yaw, geometry_msgs::Vector3& normal);

void normalToRP(const Vector3& normal, double yaw, double& roll, double& pitch);
void normalToRP(const geometry_msgs::Vector3& normal, double yaw, double& roll, double& pitch);

template <typename T>
void quaternionToNormalYaw(const geometry_msgs::Quaternion& q, T& normal, double& yaw)
{
  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(q, q_tf);

  double r, p, y;
  tf::Matrix3x3(q_tf).getRPY(r, p, y);

  RPYToNormal(r, p, y, normal);
  yaw = y;
}

template <typename T>
void quaternionToNormal(const geometry_msgs::Quaternion& q, T& normal)
{
  double yaw;
  quaternionToNormalYaw(q, normal, yaw);
}

template <typename T>
void normalToQuaternion(const T& normal, double yaw, geometry_msgs::Quaternion& q)
{
  double r, p;
  normalToRP(normal, yaw, r, p);
  q = tf::createQuaternionMsgFromRollPitchYaw(r, p, yaw);
}

inline double calcOrientation(const Vector3& vec) { return atan2(vec.y(), vec.x()); }
}  // namespace l3

#endif
