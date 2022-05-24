//=================================================================================================
// Copyright (c) 2022, Alexander stumpf, Felix Sternkopf, TU Darmstadt
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

#ifndef L3_LIBS_L3_MSG_FLOATING_BASE_CONVERSIONS_H__
#define L3_LIBS_L3_MSG_FLOATING_BASE_CONVERSIONS_H__

#include <ros/ros.h>

#include <l3_msgs/FloatingBase.h>
#include <l3_libs/types/floating_base.h>

namespace l3
{
/// Converts a FloatingBase message into a l3 FloatingBase
inline void floatingBaseMsgToL3(const l3_msgs::FloatingBase& msg, l3::FloatingBase& floating_base) { floating_base.fromMsg(msg); }

/// Converts a l3 FloatingBase into a FloatingBase message
inline void floatingBaseL3ToMsg(const l3::FloatingBase& floating_base, l3_msgs::FloatingBase& msg) { floating_base.toMsg(msg); }

/// Converts a FloatingBaseArray message into a FloatingBaseArray
void floatingBaseArrayMsgToL3(const l3_msgs::FloatingBaseArray& msg, FloatingBaseArray& floating_bases);

/// Converts a FloatingBaseArray into a FloatingBaseArray message
void floatingBaseArrayL3ToMsg(const l3::FloatingBaseArray& floating_bases, l3_msgs::FloatingBaseArray& msg);
void floatingBaseArrayL3ToMsg(const l3::FloatingBaseConstPtrArray& floating_bases, l3_msgs::FloatingBaseArray& msg);
}  // namespace l3

#endif
