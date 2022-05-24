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

#ifndef L3_LIBS_L3_KDL_CONVERSIONS_H__
#define L3_LIBS_L3_KDL_CONVERSIONS_H__

#include <eigen_conversions/eigen_kdl.h>

#include <l3_libs/types/typedefs.h>

namespace l3
{
/// Converts a KDL Vector into a l3 Vector
void vectorKdlToL3(const KDL::Vector& kdl, l3::Vector3& l3);

/// Converts a l3 Vector into a KDL Vector
void vectorL3ToKdl(const l3::Vector3& l3, KDL::Vector& kdl);

/// Converts a KDL Pose into a l3 Vector
inline void poseKdlToL3(const KDL::Frame& kdl, l3::Pose& l3) { tf::transformKDLToEigen(kdl, l3); }

/// Converts a l3 Pose into a KDL Vector
inline void poseL3ToKdl(const l3::Pose& l3, KDL::Frame& kdl) { tf::transformEigenToKDL(l3, kdl); }

/// Converts a KDL Transform into a l3 Vector
void transformKdlToL3(const KDL::Frame& kdl, l3::Transform& l3);

/// Converts a l3 Transform into a KDL Vector
inline void transformL3ToKdl(const l3::Transform& l3, KDL::Frame& kdl) { kdl = KDL::Frame(KDL::Rotation::RPY(l3.roll(), l3.pitch(), l3.yaw()), KDL::Vector(l3.x(), l3.y(), l3.z())); }

/// Converts a KDL rotation into an l3 quaternion
inline void quaternionKDLToL3(const KDL::Rotation& kdl, l3::Quaternion& l3) { tf::quaternionKDLToEigen(kdl, l3); }

/// Converts an l3 quaternion into a KDL rotation
inline void quaternionL3ToKDL(const l3::Quaternion& l3, KDL::Rotation& kdl) { tf::quaternionEigenToKDL(l3, kdl); }

/// Converts a KDL wrench into an l3 WrenchData
void wrenchKDLToL3(const KDL::Wrench& kdl, l3::WrenchData& l3);

/// Converts a l3 WrenchData into a KDL wrench
void wrenchL3ToKDL(const l3::WrenchData& l3, KDL::Wrench& kdl);

/// Converts a KDL twist into an l3 TwistData
void twistKDLToL3(const KDL::Twist& kdl, l3::TwistData& l3);

/// Converts a l3 TwistData into a KDL Twist
void twistL3ToKDL(const l3::TwistData& l3, KDL::Twist& kdl);
}  // namespace l3

#endif
