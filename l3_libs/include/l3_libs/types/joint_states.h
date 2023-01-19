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

#ifndef L3_JOINT_STATES_H__
#define L3_JOINT_STATES_H__

#include <vector>
#include <map>
#include <set>
#include <string>

#include <std_msgs/Header.h>

#include <l3_libs/macros.h>

namespace l3
{
class JointStates
{
public:
  void clear();

  /**
   * @brief Updates specific joint state with given values. Exisiting (non-updated) joints are kept.
   * Call clear() for clean update.
   * @param header Timestamp of joint states
   * @param name Name of joint
   * @param position Position [rad] of joint
   * @param velocity (optional) Velocity of joint [rad/s]
   * @param effort (optional) Effort/Torque of joint [Nm]
   */
  void updateJoint(const std::string& name, double position, double velocity = 0.0, double effort = 0.0);
  void updateJoint(const std_msgs::Header& header, const std::string& name, double position, double velocity = 0.0, double effort = 0.0);

  /**
   * @brief Updates multiple joint states with given values. Exisiting (non-updated) joints are kept.
   * Call clear() for clean update.
   * @param header Timestamp of joint states
   * @param name List of name of joint (order must be the same througout all list)
   * @param position Positions [rad] of joints
   * @param velocity (optional) Velocities of joints [rad/s]
   * @param effort (optional) Efforts/Torques of joints [Nm]
   */
  void updateJoints(const std::vector<std::string>& names, const std::vector<double>& positions, const std::vector<double>& velocities = std::vector<double>(),
                    const std::vector<double>& efforts = std::vector<double>());
  void updateJoints(const std_msgs::Header& header, const std::vector<std::string>& names, const std::vector<double>& positions,
                    const std::vector<double>& velocities = std::vector<double>(), const std::vector<double>& efforts = std::vector<double>());

  const std_msgs::Header& header() const { return header_; }

  inline std::vector<std::string> names() const { return std::vector<std::string>(names_.begin(), names_.end()); }

  inline double position(const std::string& name) const
  {
    auto itr = positions_.find(name);
    return itr != positions_.end() ? itr->second : 0.0;
  }
  inline double& position(const std::string& name) { return positions_[name]; }
  inline double velocity(const std::string& name) const
  {
    auto itr = velocities_.find(name);
    return itr != velocities_.end() ? itr->second : 0.0;
  }
  inline double& velocity(const std::string& name) { return velocities_[name]; }
  inline double effort(const std::string& name) const
  {
    auto itr = efforts_.find(name);
    return itr != efforts_.end() ? itr->second : 0.0;
  }
  inline double& effort(const std::string& name) { return efforts_[name]; }

  inline bool hasJoint(const std::string& name) const { return positions_.find(name) != positions_.end(); }

protected:
  std_msgs::Header header_;

  std::set<std::string> names_;
  std::map<std::string, double> positions_;
  std::map<std::string, double> velocities_;
  std::map<std::string, double> efforts_;
};

L3_STATIC_ASSERT_MOVEABLE(JointStates)
}  // namespace l3

#endif
