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

#ifndef L3_JOINT_CMD_INTERFACE_PLUGIN_H__
#define L3_JOINT_CMD_INTERFACE_PLUGIN_H__

#include <ros/ros.h>

#include <l3_libs/types/joint_states.h>

#include <l3_plugins/base/interface_plugin.h>

namespace l3
{
class JointCmdInterfacePlugin : public InterfacePlugin
{
public:
  enum Mode
  {
    POSITION = 0,
    VELOCITY = 1,
    EFFORT = 3
  };

  // typedefs
  typedef l3::SharedPtr<JointCmdInterfacePlugin> Ptr;
  typedef l3::SharedPtr<const JointCmdInterfacePlugin> ConstPtr;

  JointCmdInterfacePlugin(const std::string& name);

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  /**
   * @brief Updates specific joint state with given values. Exisiting (non-updated) joints are kept.
   * Call clear() for clean update.
   * @param header Timestamp of joint states
   * @param name Name of joint
   * @param position Position [rad] of joint
   * @param velocity (optional) Velocity of joint [rad/s]
   * @param effort (optional) Effort/Torque of joint [Nm]
   */
  inline void updateJoint(const std::string& name, double position, double velocity = 0.0, double effort = 0.0) { joint_states_.updateJoint(name, position, velocity, effort); }
  inline void updateJoint(const std_msgs::Header& header, const std::string& name, double position, double velocity = 0.0, double effort = 0.0)
  {
    joint_states_.updateJoint(header, name, position, velocity, effort);
  }

  /**
   * @brief Updates multiple joint states with given values. Exisiting (non-updated) joints are kept.
   * Call clear() for clean update.
   * @param header Timestamp of joint states
   * @param name List of name of joint (order must be the same througout all list)
   * @param position Positions [rad] of joints
   * @param velocity (optional) Velocities of joints [rad/s]
   * @param effort (optional) Efforts/Torques of joints [Nm]
   */
  inline void updateJoints(const std::vector<std::string>& names, const std::vector<double>& positions, const std::vector<double>& velocities = std::vector<double>(),
                           const std::vector<double>& efforts = std::vector<double>())
  {
    joint_states_.updateJoints(names, positions, velocities, efforts);
  }
  inline void updateJoints(const std_msgs::Header& header, const std::vector<std::string>& names, const std::vector<double>& positions,
                           const std::vector<double>& velocities = std::vector<double>(), const std::vector<double>& efforts = std::vector<double>())
  {
    joint_states_.updateJoints(header, names, positions, velocities, efforts);
  }

  inline void setJointStates(const JointStates& joint_states) { joint_states_ = joint_states; }

  /**
   * @brief Returns list of names of used joints.
   * @return list of joint names
   */
  inline std::vector<std::string> getJointNames() const { return joint_names_; }

  /**
   * @brief Triggers execution of current joint configuration
   */
  virtual void sendJointCmd() const = 0;

protected:
  Mode mode_;
  JointStates joint_states_;
  std::vector<std::string> joint_names_;
};
}  // namespace l3

#endif
