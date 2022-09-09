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

#ifndef L3_STEP_DATA_H__
#define L3_STEP_DATA_H__

#include <l3_msgs/FootStepData.h>

#include <l3_libs/types/foothold.h>
#include <l3_libs/types/variant_data.h>

namespace l3
{
/**
 * @brief Basic step data structure for handling high level step data.
 * This structure describes the transition between two footholds (=step).
 */
struct FootStepData
{
  // typedefs
  typedef SharedPtr<FootStepData> Ptr;
  typedef SharedPtr<const FootStepData> ConstPtr;

  FootStepData();

  FootStepData(Foothold::ConstPtr origin, Foothold::ConstPtr target, const Transform delta, double sway_duration, double step_duration, double swing_height);

  /**
   * @brief Generates step based on difference of footholds such that target = origin + step.
   */
  inline FootStepData(Foothold::ConstPtr origin, Foothold::ConstPtr target, double sway_duration = 0.0, double step_duration = 0.0, double swing_height = 0.0)
    : FootStepData(origin, target, Foothold::getDelta(*origin, *target), sway_duration, step_duration, swing_height)
  {}

  inline FootStepData(const l3_msgs::FootStepData& msg) { fromMsg(msg); }

  inline static FootStepData::Ptr make() { return makeShared<FootStepData>(); }
  inline static FootStepData::Ptr make(const FootStepData& other) { return makeShared<FootStepData>(other); }
  inline static FootStepData::Ptr make(Foothold::ConstPtr origin, Foothold::ConstPtr target, const Transform delta, double sway_duration, double step_duration, double swing_height)
  {
    return makeShared<FootStepData>(origin, target, delta, sway_duration, step_duration, swing_height);
  }
  inline static FootStepData::Ptr make(Foothold::ConstPtr origin, Foothold::ConstPtr target, double sway_duration = 0.0, double step_duration = 0.0, double swing_height = 0.0)
  {
    return makeShared<FootStepData>(origin, target, sway_duration, step_duration, swing_height);
  }
  inline static FootStepData::Ptr make(const l3_msgs::FootStepData& msg) { return makeShared<FootStepData>(msg); }

  void fromMsg(const l3_msgs::FootStepData& msg);

  void toMsg(l3_msgs::FootStepData& msg) const;
  l3_msgs::FootStepData toMsg() const;

  std::string toString() const;

  Foothold::ConstPtr origin;
  Foothold::ConstPtr target;

  // step delta
  double dx, dy, dz;
  double droll, dpitch, dyaw;

  // generic params (optional use)
  double sway_duration;
  double step_duration;
  double swing_height;

  VariantDataSet data;  // may contain user specific data
};

inline std::ostream& operator<<(std::ostream& stream, const FootStepData& step_data) { return stream << step_data.toString(); }

typedef std::vector<FootStepData> FootStepDataArray;
typedef std::vector<FootStepData::Ptr> FootStepDataPtrArray;
typedef std::vector<FootStepData::ConstPtr> FootStepDataConstPtrArray;
typedef std::pair<const FootIndex, FootStepData> FootIdxFootStepDataPair;
typedef std::map<FootIndex, FootStepData> FootIdxFootStepDataMap;

L3_STATIC_ASSERT_MOVEABLE(FootStepData)
}  // namespace l3

// msgs Definitions
namespace l3_msgs
{
typedef std::vector<l3_msgs::FootStepData> FootStepDataArray;
}

#endif
