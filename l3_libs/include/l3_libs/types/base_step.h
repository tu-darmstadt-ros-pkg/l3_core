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

#ifndef L3_BASE_STEP_H__
#define L3_BASE_STEP_H__

#include <l3_msgs/Step.h>

#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/foothold.h>
#include <l3_libs/types/floating_base.h>
#include <l3_libs/types/variant_data.h>
#include <l3_libs/helper.h>

namespace l3
{
typedef int StepIndex;
typedef std::vector<StepIndex> StepIndexArray;

/**
 * @brief Basic step data template structure for handling high level step data.
 * For example, step data might contain target foothold positions for each step.
 */
template <typename StepDataType>
class BaseStep
{
public:
  // typedefs
  typedef std::pair<const FootIndex, StepDataType> StepDataPair;
  typedef std::map<FootIndex, StepDataType> StepDataMap;

  BaseStep()
    : idx_(-1)
  {}

  BaseStep(const StepIndex& idx)
    : idx_(idx)
  {}

  /**
   * @brief Returns assigned step index
   * @param idx step index
   * @return Assigned step index
   */
  inline const StepIndex& setStepIndex(const StepIndex& idx) { idx_ = idx; return idx_; }
  inline const StepIndex& getStepIndex() const { return idx_; }

  inline typename StepDataMap::iterator begin() { return step_data_map_.begin(); }
  inline typename StepDataMap::const_iterator begin() const { return step_data_map_.begin(); }

  inline typename StepDataMap::iterator end() { return step_data_map_.end(); }
  inline typename StepDataMap::const_iterator end() const { return step_data_map_.end(); }

  inline typename StepDataMap::reverse_iterator rbegin() { return step_data_map_.rbegin(); }
  inline typename StepDataMap::const_reverse_iterator rbegin() const { return step_data_map_.rbegin(); }

  inline typename StepDataMap::reverse_iterator rend() { return step_data_map_.rend(); }
  inline typename StepDataMap::const_reverse_iterator rend() const { return step_data_map_.rend(); }

  /**
   * @brief Clears all stored data
   */
  virtual void clear()
  {
    idx_ = -1;
    step_data_map_.clear();
  }

  /**
   * @brief Checks if any data is available.
   * @return False, if no data is available.
   */
  virtual bool empty() const { return step_data_map_.empty(); }

  /**
   * @brief Clears internally all step data
   */
  inline void clearStepData() { step_data_map_.clear(); }

  /**
   * @brief Returns number of stored step data information
   * @return Number of stored step data information
   */
  inline size_t stepDataSize() const { return step_data_map_.size(); }

  /**
   * @brief Checks if step data is available
   * @return True, if step data is available
   */
  inline bool hasStepData() const { return !step_data_map_.empty(); }
  inline bool hasStepData(const FootIndex& idx) const { return step_data_map_.find(idx) != step_data_map_.end(); }

  /**
   * @brief Updates internal step data map using given input
   * @param step_data step data to be updated
   */
  void updateStepData(const StepDataType& step_data) { step_data_map_[step_data->origin->idx] = step_data; }

  /**
   * @brief Tries to return step data for given foot index.
   * @param foot_idx foot index to look up
   * @param step_data [out] return variable in which found step data is referenced
   * @return True, if step data was for given foot index is available
   */
  bool getStepData(const FootIndex& foot_idx, StepDataType& step_data) const
  {
    typename StepDataMap::const_iterator itr = step_data_map_.find(foot_idx);
    if (itr != step_data_map_.end())
    {
      step_data = itr->second;
      return true;
    }
    else
      return false;
  }

  /**
   * @brief Tries to return step data for given foot index.
   * @param foot_idx foot index to look up
   * @return Step data for given foot index is available. If no step data is
   * available, the returned data is empty.
   */
  StepDataType getStepData(const FootIndex& foot_idx) const
  {
    StepDataType step_data;
    getStepData(foot_idx, step_data);
    return step_data;
  }

  /**
   * @brief Exposes the internal step data map
   * @return Internal step data map
   */
  inline const StepDataMap& getStepDataMap() const { return step_data_map_; }
  inline StepDataMap& getStepDataMap() { return step_data_map_; }

  /**
   * @brief Returns list of all foot indeces represented by all step datas
   * @return List of foot indeces
   */
  inline FootIndexArray getStepFootIndeces() const { return keysAsArray<FootIndexArray>(step_data_map_); }

  /**
   * @brief Determines total step duration
   * @return Total (max) step duration
   */
  double getStepDuration() const
  {
    double duration = 0.0;
    for (const StepDataPair& p : step_data_map_)
      duration = std::max(duration, p.second->step_duration);

    return duration;
  }

  VariantDataSet data;          // may contain user specific data

protected:
  StepIndex idx_;               // index number of step; all succeeding steps must have an unique increasing index

  StepDataMap step_data_map_;   // map of stored step data
};

L3_STATIC_ASSERT_MOVEABLE(StepIndex)
}  // namespace l3

#endif
