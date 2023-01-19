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

#include <l3_msgs/BaseStepData.h>
#include <l3_msgs/FootStepData.h>
#include <l3_msgs/Step.h>
#include <l3_msgs/StepQueue.h>

#include <l3_libs/types/base_step_data.h>
#include <l3_libs/types/foot_step_data.h>
#include <l3_libs/types/step.h>
#include <l3_libs/types/step_queue.h>

namespace l3
{
/// Converts a Step message into a l3 StepData
inline void footStepDataMsgToL3(const l3_msgs::FootStepData& msg, l3::FootStepData& step_data) { step_data.fromMsg(msg); }

/// Converts a l3 StepData into a Step Data message
inline void footStepDataL3ToMsg(const l3::FootStepData& step_data, l3_msgs::FootStepData& msg) { step_data.toMsg(msg); }

/// Converts a StepDataArray message into a StepDataArray
void footStepDataArrayMsgToL3(const l3_msgs::FootStepDataArray& msg, l3::FootStepDataArray& step_data);

/// Converts a StepDataArray into a StepDataArray message
void footStepDataArrayL3ToMsg(const l3::FootStepDataArray& step_data, l3_msgs::FootStepDataArray& msg);

/// Converts a StepDataPtrArray into a StepDataArray message
void footStepDataArrayL3ToMsg(l3::FootStepDataPtrArray step_data, l3_msgs::FootStepDataArray& msg);

/// Converts a StepDataConstPtrArray into a StepDataArray message
void footStepDataArrayL3ToMsg(l3::FootStepDataConstPtrArray step_data, l3_msgs::FootStepDataArray& msg);

/// Converts a Base Step Data message into a l3 BaseStepData
inline void baseStepDataMsgToL3(const l3_msgs::BaseStepData& msg, l3::BaseStepData& base_step_data) { base_step_data.fromMsg(msg); }

/// Converts a l3 BaseStepData into a Base Step Data message
inline void baseStepDataL3ToMsg(const l3::BaseStepData& base_step_data, l3_msgs::BaseStepData& msg) { base_step_data.toMsg(msg); }

/// Converts a StepDataArray message into a StepDataArray
void baseStepDataArrayMsgToL3(const l3_msgs::BaseStepDataArray& msg, l3::BaseStepDataArray& base_step_data);

/// Converts a StepDataArray into a StepDataArray message
void baseStepDataArrayL3ToMsg(const l3::BaseStepDataArray& base_step_data, l3_msgs::BaseStepDataArray& msg);

/// Converts a StepDataPtrArray into a StepDataArray message
void baseStepDataArrayL3ToMsg(l3::BaseStepDataPtrArray base_step_data, l3_msgs::BaseStepDataArray& msg);

/// Converts a StepDataConstPtrArray into a StepDataArray message
void baseStepDataArrayL3ToMsg(l3::BaseStepDataConstPtrArray base_step_data, l3_msgs::BaseStepDataArray& msg);

/// Converts a Step message into a l3 Step
inline void stepMsgToL3(const l3_msgs::Step& msg, l3::Step& step) { step.fromMsg(msg); }

/// Converts a l3 Step into a Step message
inline void stepL3ToMsg(const l3::Step& step, l3_msgs::Step& msg) { step.toMsg(msg); }

/// Converts a StepQueue message into a l3 StepQueue
inline void stepQueueMsgToL3(const l3_msgs::StepQueue& msg, l3::StepQueue& step_queue) { step_queue.fromMsg(msg); }

/// Converts a l3 StepQueue into a StepQueue message
inline void stepQueueL3ToMsg(const l3::StepQueue& step_queue, l3_msgs::StepQueue& msg) { step_queue.toMsg(msg); }

/// Converts a StepArray message into a StepArray
void stepArrayMsgToL3(const l3_msgs::StepArray& msg, l3::StepArray& steps);

/// Converts a StepArray into a StepArray message
void stepArrayL3ToMsg(const l3::StepArray& steps, l3_msgs::StepArray& msg);

/// Converts a StepPtrArray into a StepArray message
void stepArrayL3ToMsg(l3::StepPtrArray steps, l3_msgs::StepArray& msg);

/// Converts a StepConstPtrArray into a StepArray message
void stepArrayL3ToMsg(l3::StepConstPtrArray steps, l3_msgs::StepArray& msg);
}  // namespace l3

#endif
