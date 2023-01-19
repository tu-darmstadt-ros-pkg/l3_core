//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_LIBS_L3_MSG_STD_CONVERSIONS_H__
#define L3_LIBS_L3_MSG_STD_CONVERSIONS_H__

#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <eigen_conversions/eigen_msg.h>

#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/joint_states.h>

namespace l3
{
/// Converts a Point message into a l3 Point
inline void pointMsgToL3(const geometry_msgs::Point& msg, l3::Point& p) { tf::pointMsgToEigen(msg, p); }

/// Converts a l3 Point into a Point message
inline void pointL3ToMsg(const l3::Point& p, geometry_msgs::Point& msg) { tf::pointEigenToMsg(p, msg); }

/// Converts an array of Point messages into an array of l3 Point
void pointArrayMsgToL3(const std::vector<geometry_msgs::Point>& msg, l3::PointArray& p);

/// Converts an array of l3 Point into an array of Point message
void pointArrayL3ToMsg(const l3::PointArray& p, std::vector<geometry_msgs::Point>& msg);

/// Converts a Pose message into a l3 Pose
inline void poseMsgToL3(const geometry_msgs::Pose& msg, l3::Pose& p) { tf::poseMsgToEigen(msg, p); }

/// Converts a l3 Pose into a Pose message
inline void poseL3ToMsg(const l3::Pose& p, geometry_msgs::Pose& msg) { tf::poseEigenToMsg(p, msg); }

/// Converts an array of Pose messages into an array of l3 Pose
void poseArrayMsgToL3(const std::vector<geometry_msgs::Pose>& msg, l3::PoseArray& p);

/// Converts an array of l3 Pose into an array of Pose message
void poseArrayL3ToMsg(const l3::PoseArray& p, std::vector<geometry_msgs::Pose>& msg);

/// Converts a Quaternion message into a l3 Quaternion
inline void quaternionMsgToL3(const geometry_msgs::Quaternion& msg, l3::Quaternion& q) { tf::quaternionMsgToEigen(msg, q); }

/// Converts a l3 Quaternion into a Quaternion message
inline void quaternionL3ToMsg(const l3::Quaternion& q, geometry_msgs::Quaternion& msg) { tf::quaternionEigenToMsg(q, msg); }

/// Converts a Transform message into a l3 Pose
inline void transformMsgToL3(const geometry_msgs::Transform& msg, l3::Pose& p) { tf::transformMsgToEigen(msg, p); }

/// Converts a l3 Pose into a Transform message
inline void transformL3ToMsg(const l3::Pose& p, geometry_msgs::Transform& msg) { tf::transformEigenToMsg(p, msg); }

/// Converts a Vector3 message into a l3 Vector3
inline void vectorMsgToL3(const geometry_msgs::Vector3& msg, l3::Vector3& v) { tf::vectorMsgToEigen(msg, v); }

/// Converts a l3 Vector3 into a Vector3 message
inline void vectorL3ToMsg(const l3::Vector3& v, geometry_msgs::Vector3& msg) { tf::vectorEigenToMsg(v, msg); }

/// Converts an array of Vector3 messages into an array of l3 Vector3
void vectorArrayMsgToL3(const std::vector<geometry_msgs::Vector3>& msg, l3::Vector3Array& v);

/// Converts an array of l3 Vector3 into an array of Vector3 message
void vectorArrayL3ToMsg(const l3::Vector3Array& v, std::vector<geometry_msgs::Vector3>& msg);

/// Converts a Wrench message into a L3 WrenchData
void wrenchMsgToL3(const geometry_msgs::Wrench& msg, l3::WrenchData& wrench_data);
void wrenchMsgToL3(const geometry_msgs::WrenchStamped& msg, l3::WrenchData& wrench_data);

/// Converts a L3 WrenchData into a Wrench message
void wrenchL3ToMsg(const l3::WrenchData& wrench_data, geometry_msgs::Wrench& msg);
void wrenchL3ToMsg(const l3::WrenchData& wrench_data, geometry_msgs::WrenchStamped& msg);

/// Converts a Twist message into a L3 TwistData
void twistMsgToL3(const geometry_msgs::Twist& msg, l3::TwistData& twist_data);
void twistMsgToL3(const geometry_msgs::TwistStamped& msg, l3::TwistData& twist_data);

/// Converts a L3 TwistData into a Twist message
void twistL3ToMsg(const l3::TwistData& twist_data, geometry_msgs::Twist& msg);
void twistL3ToMsg(const l3::TwistData& twist_data, geometry_msgs::TwistStamped& msg);

/// Converts a float64[9] message into a l3 Matrix3
void matrixMsgToL3(const boost::array<double, 9>& msg, l3::Matrix3& m);

/// Converts a l3 Matrix3 into a float64[9] message
void matrixL3ToMsg(const l3::Matrix3& m, boost::array<double, 9>& msg);

/// Converts an Imu message into an l3 ImuData
void imuMsgToL3(const sensor_msgs::Imu& msg, l3::ImuData& imu_data);

/// Converts a l3 ImuData into an Imu message
void imuL3ToMsg(const l3::ImuData& imu_data, sensor_msgs::Imu& msg);

/// Converts a JointState message into a l3 JointStates
void jointStateMsgToL3(const sensor_msgs::JointState& msg, l3::JointStates& joint_states);

/// Converts a l3 JointStates into a JointState message
void jointStateL3ToMsg(const l3::JointStates& joint_states, sensor_msgs::JointState& msg);
}  // namespace l3

#endif
