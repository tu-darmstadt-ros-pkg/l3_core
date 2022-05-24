//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_LIBS_L3_TF_CONVERSIONS_H__
#define L3_LIBS_L3_TF_CONVERSIONS_H__

#include <tf_conversions/tf_eigen.h>

#include <l3_libs/types/typedefs.h>

namespace l3
{
/// Converts a tf Point into a l3 Point
inline void pointTFToL3(const tf::Point& tf, l3::Point& p) { tf::vectorTFToEigen(tf, p); }

/// Converts a l3 Point into a tf Point
inline void pointL3ToTF(const l3::Point& p, tf::Point& tf) { tf::vectorEigenToTF(p, tf); }

/// Converts an array of tf Point into an array of l3 Point
void pointArrayTFToL3(const std::vector<tf::Point>& tf, l3::PointArray& p);

/// Converts an array of l3 Point into an array of tf Point
void pointArrayL3ToTF(const l3::PointArray& p, std::vector<tf::Point>& tf);

/// Converts a tf Pose into a l3 Pose
inline void poseTFToL3(const tf::Pose& tf, l3::Pose& p) { tf::poseTFToEigen(tf, p); }

/// Converts a l3 Pose into a tf Pose
inline void poseL3ToTF(const l3::Pose& p, tf::Pose& tf) { tf::poseEigenToTF(p, tf); }

/// Converts an array of tf Pose into an array of l3 Pose
void poseArrayTFToL3(const std::vector<tf::Pose>& tf, l3::PoseArray& p);

/// Converts an array of l3 Pose into an array of tf Pose
void poseArrayL3ToTF(const l3::PoseArray& p, std::vector<tf::Pose>& tf);

/// Converts a tf Quaternion into a l3 Quaternion
inline void quaternionTFToL3(const tf::Quaternion& tf, l3::Quaternion& q) { tf::quaternionTFToEigen(tf, q); }

/// Converts a l3 Quaternion into a tf Quaternion
inline void quaternionL3ToTF(const l3::Quaternion& q, tf::Quaternion& tf) { tf::quaternionEigenToTF(q, tf); }

/// Converts a tf Transform into a l3 Pose
inline void transformTFToL3(const tf::Transform& tf, l3::Pose& p) { tf::transformTFToEigen(tf, p); }

/// Converts a l3 Pose into a tf Transform
inline void transformL3ToTF(const l3::Pose& p, tf::Transform& tf) { tf::transformEigenToTF(p, tf); }

/// Converts a tf Vector3 into a l3 Vector3
inline void vectorTFToL3(const tf::Vector3& tf, l3::Vector3& v) { tf::vectorTFToEigen(tf, v); }

/// Converts a l3 Vector3 into a tf Vector3
inline void vectorL3ToTF(const l3::Vector3& v, tf::Vector3& tf) { tf::vectorEigenToTF(v, tf); }

/// Converts an array of tf Vector3 into an array of l3 Vector3
void vectorArrayTFToL3(const std::vector<tf::Vector3>& tf, l3::Vector3Array& v);

/// Converts an array of l3 Vector3 into an array of tf Vector3
void vectorArrayL3ToTF(const l3::Vector3Array& v, std::vector<tf::Vector3>& tf);

/// Converts a tf Matrix3x3 into a l3 Matrix3
inline void matrixTFToL3(const tf::Matrix3x3& tf, l3::Matrix3& m) { tf::matrixTFToEigen(tf, m); }

/// Converts a l3 Matrix3 into a tf Matrix3x3
inline void matrixL3ToTF(const l3::Matrix3& m, tf::Matrix3x3& tf) { tf::matrixEigenToTF(m, tf); }
}  // namespace l3

#endif
