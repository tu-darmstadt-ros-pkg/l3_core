//=================================================================================================
// Copyright (c) 2023, Alexander stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_BASE_STEP_DATA_H__
#define L3_BASE_STEP_DATA_H__

#include <l3_msgs/BaseStepData.h>

#include <l3_libs/types/floating_base.h>
#include <l3_libs/types/variant_data.h>

namespace l3
{
/**
 * @brief Basic base step data structure for handling high level base step data.
 * This structure describes the transition between two floating bases (=step).
 */
struct BaseStepData
{
  // typedefs
  typedef SharedPtr<BaseStepData> Ptr;
  typedef SharedPtr<const BaseStepData> ConstPtr;

  BaseStepData();

  BaseStepData(FloatingBase::ConstPtr origin, FloatingBase::ConstPtr target, const Transform delta, double step_duration);

  /**
   * @brief Generates base step based on difference of footholds such that target = origin + step.
   */
  inline BaseStepData(FloatingBase::ConstPtr origin, FloatingBase::ConstPtr target, double step_duration = 0.0)
    : BaseStepData(origin, target, FloatingBase::getDelta(*origin, *target), step_duration)
  {}

  inline BaseStepData(const l3_msgs::BaseStepData& msg) { fromMsg(msg); }

  inline static BaseStepData::Ptr make() { return makeShared<BaseStepData>(); }
  inline static BaseStepData::Ptr make(const BaseStepData& other) { return makeShared<BaseStepData>(other); }
  inline static BaseStepData::Ptr make(FloatingBase::ConstPtr origin, FloatingBase::ConstPtr target, const Transform delta, double step_duration)
  {
    return makeShared<BaseStepData>(origin, target, delta, step_duration);
  }
  inline static BaseStepData::Ptr make(FloatingBase::ConstPtr origin, FloatingBase::ConstPtr target, double step_duration = 0.0)
  {
    return makeShared<BaseStepData>(origin, target, step_duration);
  }
  inline static BaseStepData::Ptr make(const l3_msgs::BaseStepData& msg) { return makeShared<BaseStepData>(msg); }

  void fromMsg(const l3_msgs::BaseStepData& msg);

  void toMsg(l3_msgs::BaseStepData& msg) const;
  l3_msgs::BaseStepData toMsg() const;

  std::string toString() const;

  FloatingBase::ConstPtr origin;
  FloatingBase::ConstPtr target;

  // step delta
  double dx, dy, dz;
  double droll, dpitch, dyaw;

  // generic params (optional use)
  double step_duration;

  VariantDataSet data;  // may contain user specific data
};

inline std::ostream& operator<<(std::ostream& stream, const BaseStepData& base_step_data) { return stream << base_step_data.toString(); }

typedef std::vector<BaseStepData> BaseStepDataArray;
typedef std::vector<BaseStepData::Ptr> BaseStepDataPtrArray;
typedef std::vector<BaseStepData::ConstPtr> BaseStepDataConstPtrArray;
typedef std::pair<const BaseIndex, BaseStepData> BaseIdxBaseStepDataPair;
typedef std::map<BaseIndex, BaseStepData> BaseIdxBaseStepDataMap;

L3_STATIC_ASSERT_MOVEABLE(BaseStepData)
}  // namespace l3

// msgs Definitions
namespace l3_msgs
{
typedef std::vector<l3_msgs::BaseStepData> BaseStepDataArray;
}

#endif
