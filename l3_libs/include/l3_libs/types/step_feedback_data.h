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

#ifndef L3_STEP_FEEDBACK_DATA_H__
#define L3_STEP_FEEDBACK_DATA_H__

#include <l3_msgs/StepFeedbackData.h>

#include <l3_libs/types/base_step_data.h>
#include <l3_libs/types/foot_step_data.h>

namespace l3
{
/**
 * @brief More detailed step data information including low-level walking data.
 * This structure describes the execution result of the transition between two footholds (=step).
 */
struct StepFeedbackData
{
  // typedefs
  typedef SharedPtr<StepFeedbackData> Ptr;
  typedef SharedPtr<const StepFeedbackData> ConstPtr;

  StepFeedbackData();
  StepFeedbackData(const FootStepData& other);

  void reset();

  inline StepFeedbackData(const l3_msgs::StepFeedbackData& msg) { fromMsg(msg); }

  void fromMsg(const l3_msgs::StepFeedbackData& msg);

  void toMsg(l3_msgs::StepFeedbackData& msg) const;
  l3_msgs::StepFeedbackData toMsg() const;

  FootStepData foot_step;
  BaseStepData base_step;

  bool changeable;  // indicates if step can still be changed
  bool executing;   // indicates if step is currently executed
  bool finished;    // indicates if step has been executed

  ros::Time execution_start;  // timestamp when execution of step will start
  ros::Time execution_end;    // timestamp when execution of step will be completed
};

typedef std::pair<const FootIndex, StepFeedbackData> FootIdxStepFeedbackDataPair;
typedef std::map<FootIndex, StepFeedbackData> FootIdxStepFeedbackDataMap;
typedef std::vector<StepFeedbackData> StepFeedbackDataArray;
typedef std::vector<StepFeedbackData::Ptr> StepFeedbackDataPtrArray;
typedef std::vector<StepFeedbackData::ConstPtr> StepFeedbackDataConstPtrArray;

L3_STATIC_ASSERT_MOVEABLE(StepFeedbackData)
}  // namespace l3

// msgs Definitions
namespace l3_msgs
{
typedef std::vector<l3_msgs::StepFeedbackData> StepFeedbackDataArray;
}

#endif
