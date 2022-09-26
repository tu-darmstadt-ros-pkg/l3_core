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

#ifndef L3_GAIT_GENERATOR_PLUGIN_H__
#define L3_GAIT_GENERATOR_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_pluginlib/plugin.h>

#include <l3_libs/types/types.h>
#include <l3_libs/robot_description/robot_description.h>

namespace l3
{
class GaitGeneratorPlugin : public vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef l3::SharedPtr<GaitGeneratorPlugin> Ptr;
  typedef l3::SharedPtr<const GaitGeneratorPlugin> ConstPtr;

  GaitGeneratorPlugin(const std::string& name);

  bool isUnique() const final { return true; }

  virtual void setRobotDescription(RobotDescription::ConstPtr robot_description);

  /**
   * @brief Determine the preceding legs which may/must be moved (backwards planning).
   * @param step Current step describing the movement of the robot. Can be null!
   * @param next_seq Future history of foot and base movement given as arrays of foot and base indeces.
   * @return List of all possible combinations of moveable feet for the immediate next step. In general,
   * the sublists would contain only a single foot index, when the robot is not able to move multiple legs at once.
   * The returned list should reflect any preferences by sorting the entries with the most preferred pattern at front.
   */
  virtual ExpandStatesIdxArray predMovingPatterns(Step::ConstPtr step, const ExpandStatesIdxArray& next_seq) const = 0;

  /**
   * @brief Determine the succeeding legs which may/must be moved (forward planning).
   * @param step Current step describing the movement of the robot. Can be null!
   * @param last_seq Past history of foot movement given as array of foot indeces with the preceding step at the BACK of the list.
   * @return List of all possible combinations of moveable feet for the immediate next step. In general,
   * the sublists would contain only a single foot index, when the robot is not able to move multiple legs at once.
   * The returned list should reflect any preferences by sorting the entries with the most preferred pattern at front.
   */
  virtual ExpandStatesIdxArray succMovingPatterns(Step::ConstPtr step, const ExpandStatesIdxArray& last_seq) const = 0;

protected:
  RobotDescription::ConstPtr robot_description_;
};
}  // namespace l3

#endif
