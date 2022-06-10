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

#ifndef L3_STEP_H__
#define L3_STEP_H__

#include <l3_msgs/Step.h>

#include <l3_libs/types/base_step.h>
#include <l3_libs/types/step_data.h>
#include <l3_libs/types/base_step_data.h>

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
class Step : public BaseStep<StepData::Ptr>
{
public:
  // typedefs
  typedef std::pair<const BaseIndex, BaseStepData::Ptr> BaseStepDataPair;
  typedef std::map<BaseIndex, BaseStepData::Ptr> BaseStepDataMap;

  typedef SharedPtr<Step> Ptr;
  typedef SharedPtr<const Step> ConstPtr;

  Step()
    : BaseStep()
  {}

  Step(const StepIndex& idx)
    : BaseStep(idx)
  {}

  inline Step(const l3_msgs::Step& msg) { fromMsg(msg); }

  inline static Step::Ptr make() { return makeShared<Step>(); }
  inline static Step::Ptr make(const Step& other) { return makeShared<Step>(other); }
  inline static Step::Ptr make(const StepIndex& idx) { return makeShared<Step>(idx); }
  inline static Step::Ptr make(const l3_msgs::Step& msg) { return makeShared<Step>(msg); }

  void fromMsg(const l3_msgs::Step& msg);

  void toMsg(l3_msgs::Step& msg) const;
  l3_msgs::Step toMsg() const;

  /**
   * @brief Clears all step data
   */
  void clear() override;

  /**
   * @brief Checks if any step data, support footholds or floating base are available.
   * @return False, if neither step data nor support footholds nor floating base are available.
   */
  bool empty() const override { return BaseStep<StepData::Ptr>::empty() && support_.empty() && moving_bases_map_.empty() && resting_bases_map_.empty(); }

  inline Step& transform(const tf::Transform transform, const std_msgs::Header& header = std_msgs::Header()) { return this->transform(Transform(transform), header); }
  Step& transform(const Transform& transform, const std_msgs::Header& header = std_msgs::Header());

  /**
   * @brief Returns support and step target footholds in a single list
   * @return List of all support and step target footholds
   */
  FootholdConstPtrArray getFootholds() const;

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
  FloatingBaseConstPtrArray getFloatingBases() const;

  /**
   * @brief Returns floating base, which can be either a moving or resting base, represented in this step.
   * @param base_idx base index to look up
   * @return Found floating base, which can be either a moving or resting base
   */
  FloatingBase::ConstPtr getFloatingBase(const BaseIndex& base_idx) const;

  /**
   * @brief Clears list of support footholds
   */
  inline void clearSupport() { support_.clear(); }

  /**
   * @brief Returns number of supporting footholds
   * @return Number of supporting footholds
   */
  inline size_t supportSize() const { return support_.size(); }

  /**
   * @brief Checks if support footholds are available
   * @return True, if support footholds are available
   */
  inline bool hasSupport() const { return !support_.empty(); }
  inline bool hasSupport(const FootIndex& foot_idx) const { return support_.find(foot_idx) != support_.end(); }

  /**
   * @brief Updates support footholds by given input
   * @param foothold support foothold to be updated
   */
  void updateSupport(Foothold::ConstPtr foothold) { support_[foothold->idx] = foothold; }

  /**
   * @brief Tries to return support foothold for given foot index.
   * @param foot_idx foot index to look up
   * @return Supporting foothold. If no entry was found for given input foothold index,
   * a null pointer is returned.
   */
  Foothold::ConstPtr getSupport(const FootIndex& foot_idx) const
  {
    Foothold::ConstPtr foothold;
    FootholdConstPtrMap::const_iterator itr = support_.find(foot_idx);
    if (itr != support_.end())
      foothold = itr->second;

    return foothold;
  }

  /**
   * @brief Exposes the internal support foothold map
   * @return Internal support foothold map
   */
  inline const FootholdConstPtrMap& getSupportMap() const { return support_; }
  inline FootholdConstPtrMap& getSupportMap() { return support_; }

  /**
   * @brief Returns list of all foot indeces represented by all support footholds
   * @return List of foot indeces
   */
  inline FootIndexArray getSupportFootIndeces() const { return keysAsArray<FootIndexArray>(support_); }

  /**
   * @brief Clears internally all base step data
   */
  inline void clearMovingFloatingBases() { moving_bases_map_.clear(); }

  /**
   * @brief Returns number of stored base step data information
   * @return Number of stored base step data information
   */
  inline size_t movingFloatingBasesSize() const { return moving_bases_map_.size(); }

  /**
   * @brief Checks if moving floating base is available
   * @return True, if moving floating base is available
   */
  inline bool hasMovingFloatingBase() const { return !moving_bases_map_.empty(); }
  inline bool hasMovingFloatingBase(const BaseIndex& idx) const { return moving_bases_map_.find(idx) != moving_bases_map_.end(); }

  /**
   * @brief Updates internal moving floating base map using given input
   * @param base_step_data moving base to be updated
   */
  void updateMovingFloatingBase(const BaseStepData::Ptr& base_step_data) { moving_bases_map_[base_step_data->origin->idx] = base_step_data; }

  /**
   * @brief Tries to return moving floating base for given base index.
   * @param base_idx base index to look up
   * @return moving floating base for given base index is available. If no moving floating base is
   * available, the returned data is a null pointer.
   */
  BaseStepData::ConstPtr getMovingFloatingBase(const BaseIndex& base_idx) const
  {
    BaseStepData::ConstPtr base_step_data;
    typename BaseStepDataMap::const_iterator itr = moving_bases_map_.find(base_idx);
    if (itr != moving_bases_map_.end())
      base_step_data = itr->second;

    return base_step_data;
  }

  /**
   * @brief Exposes the internal moving floating base map
   * @return Internal moving floating base map
   */
  inline const BaseStepDataMap& getMovingFloatingBaseMap() const { return moving_bases_map_; }
  inline BaseStepDataMap& getMovingFloatingBaseMap() { return moving_bases_map_; }

  /**
   * @brief Returns list of all moving floating bases indeces represented by all step datas
   * @return List of moving floating base indeces
   */
  inline BaseIndexArray getMovingFloatingBaseIndeces() const { return keysAsArray<BaseIndexArray>(moving_bases_map_); }

  /**
   * @brief Clears list of resting floating bases
   */
  inline void clearRestingFloatingBases() { resting_bases_map_.clear(); }

  /**
   * @brief Returns number of resting floating bases
   * @return Number of resting floating bases
   */
  inline size_t restingFloatingBasesSize() const { return resting_bases_map_.size(); }

  /**
   * @brief Checks if resting floating bases are available
   * @return True, if resting floating bases are available
   */
  inline bool hasRestingFloatingBase() const { return !resting_bases_map_.empty(); }
  inline bool hasRestingFloatingBase(const BaseIndex& base_idx) const { return resting_bases_map_.find(base_idx) != resting_bases_map_.end(); }

  /**
   * @brief Updates resting floating bases by given input
   * @param floating_base resting floating_base to be updated
   */
  void updateRestingFloatingBase(FloatingBase::ConstPtr floating_base) { resting_bases_map_[floating_base->idx] = floating_base; }

  /**
   * @brief Tries to return resting floating base for given base index.
   * @param base_idx base index to look up
   * @return Resting floating base. If no entry was found for given input base index,
   * a null pointer is returned.
   */
  FloatingBase::ConstPtr getRestingFloatingBase(const BaseIndex& base_idx) const
  {
    FloatingBase::ConstPtr floating_base;
    FloatingBaseConstPtrMap::const_iterator itr = resting_bases_map_.find(base_idx);
    if (itr != resting_bases_map_.end())
      floating_base = itr->second;

    return floating_base;
  }

  /**
   * @brief Exposes the internal resting floating base map
   * @return Internal resting floating base map
   */
  inline const FloatingBaseConstPtrMap& getRestingFloatingBaseMap() const { return resting_bases_map_; }
  inline FloatingBaseConstPtrMap& getRestingFloatingBaseMap() { return resting_bases_map_; }

  /**
   * @brief Returns list of all resting base indeces
   * @return List of resting floating base indeces
   */
  inline BaseIndexArray getRestingFloatingBaseIndeces() const { return keysAsArray<BaseIndexArray>(resting_bases_map_); }

private:
  FootholdConstPtrMap support_;  // map of stored foothold data

  BaseStepDataMap moving_bases_map_;           // floating base step data
  FloatingBaseConstPtrMap resting_bases_map_;  // floating base support position
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
