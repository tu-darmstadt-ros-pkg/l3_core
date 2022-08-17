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

#ifndef L3_STEP_FEEDBACK_H__
#define L3_STEP_FEEDBACK_H__

#include <l3_msgs/StepFeedback.h>

#include <l3_libs/types/base_step.h>
#include <l3_libs/types/step_feedback_data.h>

namespace l3
{
/**
 * @brief Basic step data structure for handling execution feedback steps.
 * A step is the collection of all moving and supporting feet. For all
 * moving feet the corresponding transition is stored as StepData.
 */
class StepFeedback : public BaseStep<StepFeedbackData::Ptr>
{
public:
  // typedefs
  typedef SharedPtr<StepFeedback> Ptr;
  typedef SharedPtr<const StepFeedback> ConstPtr;

  StepFeedback()
    : BaseStep()
    , changeable_(false)
    , executing_(false)
    , finished_(false)
  {}

  StepFeedback(const StepIndex& idx)
    : BaseStep(idx)
    , changeable_(false)
    , executing_(false)
    , finished_(false)
  {}

  void reset();

  inline StepFeedback(const l3_msgs::StepFeedback& msg) { fromMsg(msg); }

  void fromMsg(const l3_msgs::StepFeedback& msg);

  void toMsg(l3_msgs::StepFeedback& msg) const;
  l3_msgs::StepFeedback toMsg() const;

  /**
   * @brief Clears all step data
   */
  void clear() override;

  bool isChangeable() const;
  bool isExecuting() const;
  bool isFinished() const;

  void setChangeable();
  void setExecuting();
  void setFinished();

  ros::Time firstExecutionStart() const;
  ros::Time latestExecutionStart() const;
  ros::Time firstExecutionEnd() const;
  ros::Time latestExecutionEnd() const;

  inline double stepDuration() const { return (latestExecutionEnd() - firstExecutionStart()).toSec(); }

private:
  bool changeable_;
  bool executing_;
  bool finished_;
};

typedef std::vector<StepFeedback> StepFeedbackArray;
typedef std::vector<StepFeedback::Ptr> StepFeedbackPtrArray;
typedef std::vector<StepFeedback::ConstPtr> StepFeedbackConstPtrArray;

L3_STATIC_ASSERT_MOVEABLE(StepFeedback)
}  // namespace l3

// msgs Definitions
namespace l3_msgs
{
typedef std::vector<l3_msgs::StepFeedback> StepFeedbackArray;
}

#endif
