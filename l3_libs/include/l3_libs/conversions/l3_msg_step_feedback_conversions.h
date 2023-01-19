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

#ifndef L3_LIBS_L3_MSG_STEP_CONVERSIONS_H__
#define L3_LIBS_L3_MSG_STEP_CONVERSIONS_H__

#include <ros/ros.h>

#include <l3_msgs/Step.h>
#include <l3_msgs/StepFeedbackData.h>
#include <l3_msgs/StepFeedbackQueue.h>

#include <l3_libs/types/step_feedback.h>
#include <l3_libs/types/step_feedback_data.h>
#include <l3_libs/types/step_feedback_queue.h>

namespace l3
{
/// Converts a StepFeedbackData message into a l3 StepFeedbackData
inline void stepFeedbackDataMsgToL3(const l3_msgs::StepFeedbackData& msg, l3::StepFeedbackData& step_data) { step_data.fromMsg(msg); }

/// Converts a l3 StepFeedbackData into a StepFeedbackData message
inline void stepFeedbackDataL3ToMsg(const l3::StepFeedbackData& step_data, l3_msgs::StepFeedbackData& msg) { step_data.toMsg(msg); }

/// Converts a StepFeedback message into a l3 StepFeedback
inline void stepFeedbackMsgToL3(const l3_msgs::StepFeedback& msg, l3::StepFeedback& step) { step.fromMsg(msg); }

/// Converts a l3 StepFeedback into a StepFeedback message
inline void stepFeedbackL3ToMsg(const l3::StepFeedback& step, l3_msgs::StepFeedback& msg) { step.toMsg(msg); }

/// Converts a StepFeedbackQueue message into a l3 StepFeedbackQueue
inline void stepFeedbackQueueMsgToL3(const l3_msgs::StepFeedbackQueue& msg, l3::StepFeedbackQueue& step_queue) { step_queue.fromMsg(msg); }

/// Converts a l3 StepFeedbackQueue into a StepFeedbackQueue message
inline void stepFeedbackQueueL3ToMsg(const l3::StepFeedbackQueue& step_queue, l3_msgs::StepFeedbackQueue& msg) { step_queue.toMsg(msg); }

/// Converts a StepFeedbackDataArray message into a StepFeedbackDataArray
void stepFeedbackDataArrayMsgToL3(const l3_msgs::StepFeedbackDataArray& msg, l3::StepFeedbackDataArray& step_data);

/// Converts a StepFeedbackDataArray into a StepFeedbackDataArray message
void stepFeedbackDataArrayL3ToMsg(const l3::StepFeedbackDataArray& step_data, l3_msgs::StepFeedbackDataArray& msg);

/// Converts a StepFeedbackDataPtrArray into a StepFeedbackDataArray message
void stepFeedbackDataArrayL3ToMsg(l3::StepFeedbackDataPtrArray step_data, l3_msgs::StepFeedbackDataArray& msg);

/// Converts a StepFeedbackDataConstPtrArray into a StepFeedbackDataArray message
void stepFeedbackDataArrayL3ToMsg(l3::StepFeedbackDataConstPtrArray step_data, l3_msgs::StepFeedbackDataArray& msg);

/// Converts a StepFeedbackArray message into a StepFeedbackArray
void stepFeedbackArrayMsgToL3(const l3_msgs::StepFeedbackArray& msg, l3::StepFeedbackArray& steps);

/// Converts a StepFeedbackArray into a StepFeedbackArray message
void stepFeedbackArrayL3ToMsg(const l3::StepFeedbackArray& steps, l3_msgs::StepFeedbackArray& msg);

/// Converts a StepFeedbackPtrArray into a StepFeedbackArray message
void stepFeedbackArrayL3ToMsg(l3::StepFeedbackPtrArray steps, l3_msgs::StepFeedbackArray& msg);

/// Converts a StepFeedbackConstPtrArray into a StepFeedbackArray message
void stepFeedbackArrayL3ToMsg(l3::StepFeedbackConstPtrArray steps, l3_msgs::StepFeedbackArray& msg);
}  // namespace l3

#endif
