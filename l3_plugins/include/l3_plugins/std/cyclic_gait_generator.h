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

#ifndef L3_CYCLIC_GAIT_GENERATOR_H__
#define L3_CYCLIC_GAIT_GENERATOR_H__

#include <ros/ros.h>

#include <l3_plugins/base/gait_generator_plugin.h>

namespace l3
{
/**
 * @brief The CyclicGaitGenerator class generates cyclic gait as configured.
 * Hereby, any combination of foot as well as floating base gaits can be used.
 *
 * Config:
 * cycle:
 * Foot cycle only
 * case 1: [0, 1, 2, 3]        # (list) Single-leg cycle
 * case 2: [[0, 2], [1, 3]]    # (list of list) Multi-leg cycle
 *
 * With floating base cycle
 * case 3: [{fh: [0], fb: [0]}, {fh: [1, 2], fb: [0]}, ...]    # (list of dicts (of lists))
 * or
 * - {fh: 0}
 * - {fb: 0}
 * - {fh: 0, fb: 0}
 * - {fh: [0], fb: [0]}
 * - {fh: [1, 2], fb: [0]}
 */
class CyclicGaitGenerator : public GaitGeneratorPlugin
{
  typedef std::map<ExpandStatesIdx, ExpandStatesIdx> LookupTable;

public:
  // typedefs
  typedef l3::SharedPtr<CyclicGaitGenerator> Ptr;
  typedef l3::SharedPtr<const CyclicGaitGenerator> ConstPtr;

  CyclicGaitGenerator();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  void setRobotDescription(RobotDescription::ConstPtr robot_description) override;

  /**
   * @brief Determine the preceding legs which may/must be moved (backwards planning).
   * @param step Current step describing the movement of the robot. Can be null!
   * @param next_seq Future history of foot movement given as array of foot indeces with the succeeding step at the FRONT of the list.
   * @return List of all possible combinations of moveable feet for the immediate next step. In general,
   * the sublists would contain only a single foot index, when the robot is not able to move multiple legs at once.
   * The returned list should reflect any preferences by sorting the entries with the most preferred pattern at front.
   */
  ExpandStatesIdxArray predMovingPatterns(Step::ConstPtr step, const ExpandStatesIdxArray& next_seq) const override;

  /**
   * @brief Determine the succeeding legs which may/must be moved (forward planning).
   * @param step Current step describing the movement of the robot. Can be null!
   * @param last_seq Past history of foot movement given as array of foot indeces with the preceding step at the BACK of the list.
   * @return List of all possible combinations of moveable feet for the immediate next step. In general,
   * the sublists would contain only a single foot index, when the robot is not able to move multiple legs at once.
   * The returned list should reflect any preferences by sorting the entries with the most preferred pattern at front.
   */
  ExpandStatesIdxArray succMovingPatterns(Step::ConstPtr step, const ExpandStatesIdxArray& last_seq) const override;

protected:
  bool loadCycle();
  bool getCycleFromYaml(const std::string& key, ExpandStatesIdxArray& cycle);

  static std::string toString(const ExpandStatesIdxArray& cycle);
  static std::string toString(const ExpandStatesIdx& step);

  ExpandStatesIdxArray cycle_;
  LookupTable succ_;
  LookupTable pred_;
  ExpandStatesIdxArray start_;

  bool ignore_floating_base_;
};
}  // namespace l3

#endif
