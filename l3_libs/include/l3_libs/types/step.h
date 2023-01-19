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

#ifndef L3_STEP_H__
#define L3_STEP_H__

#include <l3_msgs/Step.h>

#include <l3_libs/types/abstract_step.h>
#include <l3_libs/types/base_step_data.h>
#include <l3_libs/types/foot_step_data.h>

namespace l3
{
struct ExpandStatesIdx
{
  ExpandStatesIdx()
    : foot_idx()
    , floating_base_idx()
  {}

  ExpandStatesIdx(const FootIndexArray& foot_idx, const BaseIndexArray& floating_base_idx)
    : foot_idx(foot_idx)
    , floating_base_idx(floating_base_idx)
  {}

  ExpandStatesIdx(FootIndexArray&& foot_idx, BaseIndexArray&& floating_base_idx)
    : foot_idx(std::move(foot_idx))
    , floating_base_idx(std::move(floating_base_idx))
  {}

  bool operator==(const ExpandStatesIdx& other) const { return foot_idx == other.foot_idx && floating_base_idx == floating_base_idx; }

  bool operator<(const ExpandStatesIdx& other) const
  {
    // lexicographically check arrays

    // #1: Try to decide via floating base indeces
    if (floating_base_idx != other.floating_base_idx)  // (assume to be a quicker check than using foot idx)
      return floating_base_idx < other.floating_base_idx;
    // #2: Decide via foot indeces
    else
      return foot_idx < other.foot_idx;
  }

  FootIndexArray foot_idx;
  BaseIndexArray floating_base_idx;
};

L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(l3::ExpandStatesIdx)

typedef std::vector<ExpandStatesIdx> ExpandStatesIdxArray;

/**
 * @brief Basic step data structure for handling high level steps.
 * A step is the collection of all moving and supporting feet. For all
 * moving feet the corresponding transition is stored as StepData.
 */
class Step
{
public:
  // typedefs
  typedef AbstractStep<FootStepData::Ptr, Foothold::ConstPtr> FootStep;
  typedef AbstractStep<BaseStepData::Ptr, FloatingBase::ConstPtr> BaseStep;

  typedef SharedPtr<Step> Ptr;
  typedef SharedPtr<const Step> ConstPtr;

  Step() {}

  Step(const StepIndex& idx)
  {
    foot_step_.setStepIndex(idx);
    base_step_.setStepIndex(idx);
  }

  inline Step(const l3_msgs::Step& msg) { fromMsg(msg); }

  inline static Step::Ptr make() { return makeShared<Step>(); }
  inline static Step::Ptr make(const Step& other) { return makeShared<Step>(other); }
  inline static Step::Ptr make(const StepIndex& idx) { return makeShared<Step>(idx); }
  inline static Step::Ptr make(const l3_msgs::Step& msg) { return makeShared<Step>(msg); }

  void fromMsg(const l3_msgs::Step& msg);

  void toMsg(l3_msgs::Step& msg) const;
  l3_msgs::Step toMsg() const;

  /**
   * @brief Returns assigned step index
   * @param idx step index
   * @return Assigned step index
   */
  inline void setStepIndex(const StepIndex& idx)
  {
    foot_step_.setStepIndex(idx);
    base_step_.setStepIndex(idx);
  }
  inline const StepIndex& getStepIndex() const { return foot_step_.getStepIndex(); }

  /**
   * @brief Clears all step data
   */
  void clear();

  /**
   * @brief Checks if any step data, support footholds or floating base are available.
   * @return False, if neither step data nor support footholds nor floating base are available.
   */
  bool empty() const { return foot_step_.empty() && base_step_.empty(); }

  /**
   * @brief Determines total max step duration
   * @return Total (max) step duration
   */
  double getStepDuration() const { return std::max(foot_step_.getStepDuration(), base_step_.getStepDuration()); }

  const FootStep& footStep() const { return foot_step_; }
  FootStep& footStep() { return foot_step_; }

  const BaseStep& baseStep() const { return base_step_; }
  BaseStep& baseStep() { return base_step_; }

  inline Step& transform(const tf::Transform transform, const std_msgs::Header& header = std_msgs::Header()) { return this->transform(Transform(transform), header); }
  Step& transform(const Transform& transform, const std_msgs::Header& header = std_msgs::Header());

  /**
   * @brief Returns support and step target footholds in a single list
   * @return List of all support and step target footholds
   */
  FootholdConstPtrArray getAllFootholds() const;

  /**
   * @brief Returns foothold, which can be either a moving or supporting foot, represented in this step.
   * @param foot_idx foot index to look up
   * @return Found foothold, which can be either a moving or supporting foot
   */
  Foothold::ConstPtr getFoothold(const FootIndex& foot_idx) const;

  /**
   * @brief Returns support and step target floating bases in a single list
   * @return List of all support and step target floating bases
   */
  FloatingBaseConstPtrArray getAllFloatingBases() const;

  /**
   * @brief Returns floating base, which can be either a moving or resting base, represented in this step.
   * @param base_idx base index to look up
   * @return Found floating base, which can be either a moving or resting base
   */
  FloatingBase::ConstPtr getFloatingBase(const BaseIndex& base_idx) const;

  VariantDataSet data;  // may contain user specific data

private:
  FootStep foot_step_;
  BaseStep base_step_;
};

typedef std::vector<Step> StepArray;
typedef std::vector<const Step> StepConstArray;
typedef std::vector<Step::Ptr> StepPtrArray;
typedef std::vector<Step::ConstPtr> StepConstPtrArray;

L3_STATIC_ASSERT_MOVEABLE(Step)
}  // namespace l3

// msgs Definitions
namespace l3_msgs
{
typedef std::vector<l3_msgs::Step> StepArray;
}

#endif
