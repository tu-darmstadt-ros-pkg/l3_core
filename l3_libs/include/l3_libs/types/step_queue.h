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

#ifndef L3_STEP_QUEUE_H__
#define L3_STEP_QUEUE_H__

#include <l3_msgs/StepQueue.h>

#include <l3_libs/types/step.h>
#include <l3_libs/types/base_queue.h>

namespace l3
{
class StepQueue : public BaseQueue<Step::Ptr>
{
public:
  StepQueue()
    : BaseQueue<Step::Ptr>()
  {}

  StepQueue(const StepQueue& other)
    : BaseQueue<Step::Ptr>(other)
  {}

  StepQueue(StepQueue&& other)
    : BaseQueue<Step::Ptr>(std::move(other))
  {}

  inline StepQueue(const l3_msgs::StepQueue& msg) { fromMsg(msg); }

  inline StepQueue& swap(StepQueue& other)
  {
    BaseQueue<Step::Ptr>::swap(other);
    return *this;
  }

  inline StepQueue& operator=(const StepQueue& other)
  {
    BaseQueue<Step::Ptr>::operator=(other);
    return *this;
  }

  inline StepQueue& operator=(StepQueue&& other)
  {
    BaseQueue<Step::Ptr>::operator=(std::move(other));
    return *this;
  }

  void fromMsg(const l3_msgs::StepQueue& msg);

  void toMsg(l3_msgs::StepQueue& msg) const;
  l3_msgs::StepQueue toMsg() const;
};

L3_STATIC_ASSERT_MOVEABLE(StepQueue)
}  // namespace l3

#endif
