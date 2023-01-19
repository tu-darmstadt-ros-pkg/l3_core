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

#ifndef L3_ABSTRACT_STEP_H__
#define L3_ABSTRACT_STEP_H__

#include <l3_msgs/Step.h>

#include <l3_libs/types/typedefs.h>
#include <l3_libs/types/base_link.h>
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
template <class MovingDataType, class NonMovingDataType>
class AbstractStep
{
public:
  // typedefs
  typedef std::pair<const LinkIndex, MovingDataType> MovingDataPair;
  typedef std::map<LinkIndex, MovingDataType> MovingDataMap;

  typedef std::pair<const LinkIndex, NonMovingDataType> NonMovingDataPair;
  typedef std::map<LinkIndex, NonMovingDataType> NonMovingDataMap;

  AbstractStep()
    : idx_(-1)
  {}

  AbstractStep(const StepIndex& idx)
    : idx_(idx)
  {}

  /**
   * @brief Returns assigned step index
   * @param idx step index
   */
  inline void setStepIndex(const StepIndex& idx) { idx_ = idx; }
  inline const StepIndex& getStepIndex() const { return idx_; }

  /**
   * @brief Clears all stored data
   */
  void clear()
  {
    idx_ = -1;
    moving_links_map_.clear();
    non_moving_links_map_.clear();
  }

  /**
   * @brief Checks if any data is available.
   * @return False, if no data is available.
   */
  bool empty() const { return moving_links_map_.empty() && non_moving_links_map_.empty(); }

  /**
   * @brief Returns the total number of all moving and non-moving links stored in this step.
   * @return Total number of moving and non-moving links.
   */
  size_t size() const { return moving_links_map_.size() + non_moving_links_map_.size(); }

  /**
   * @brief Check if link is defined in this step
   * @param idx link index to look up
   * @return true if link is defined in this step
   */
  bool hasLink(const LinkIndex& idx) const { return hasMovingLink(idx) || hasNonMovingLink(idx); }

  /**
   * @brief Determines total max step duration
   * @return Total (max) step duration
   */
  double getStepDuration() const
  {
    double duration = 0.0;
    for (const MovingDataPair& p : moving_links_map_)
      duration = std::max(duration, p.second->step_duration);

    return duration;
  }

  /**********************************/
  /*** MOVING LINKS *****************/
  /**********************************/

  /**
   * @brief Returns number of stored moving link information
   * @return Number of stored moving links information
   */
  inline size_t movingLinksSize() const { return moving_links_map_.size(); }

  /**
   * @brief Checks if moving links are available
   * @param idx link index to look up
   * @return True, if moving links are available
   */
  inline bool hasMovingLinks() const { return !moving_links_map_.empty(); }
  inline bool hasMovingLink(const LinkIndex& idx) const { return moving_links_map_.find(idx) != moving_links_map_.end(); }

  /**
   * @brief Updates internal moving link map using given input
   * @param idx link index to look up
   * @param data moving link to be updated
   */
  void updateMovingLink(const LinkIndex& idx, const MovingDataType& data)
  {
    moving_links_map_[idx] = data;
    removeNonMovingLink(idx);
  }

  /**
   * @brief Removes moving link
   * @param idx link index to delete
   */
  void removeMovingLink(const LinkIndex& idx) { moving_links_map_.erase(idx); }

  /**
   * @brief Tries to return moving link for given link index.
   * @param idx link index to look up
   * @param data [out] return variable in which found moving link is referenced
   * @return True, if moving link was for given link index is available
   */
  bool getMovingLink(const LinkIndex& idx, MovingDataType& data) const
  {
    typename MovingDataMap::const_iterator itr = moving_links_map_.find(idx);
    if (itr != moving_links_map_.end())
    {
      data = itr->second;
      return true;
    }
    else
      return false;
  }

  /**
   * @brief Tries to return moving link for given link index.
   * @param idx link index to look up
   * @return moving link for given link index is available. If no moving link is
   * available, the returned data is empty.
   */
  MovingDataType getMovingLink(const LinkIndex& idx) const
  {
    MovingDataType data;
    getMovingLink(idx, data);
    return data;
  }

  /**
   * @brief Exposes the internal moving link map
   * @return Internal moving link map
   */
  inline const MovingDataMap& getMovingLinks() const { return moving_links_map_; }
  inline MovingDataMap& getMovingLinks() { return moving_links_map_; }

  /**
   * @brief Returns list of all link indeces represented by all moving links
   * @return List of link indeces
   */
  inline LinkIndexArray getMovingLinkIndeces() const { return keysAsArray<LinkIndexArray>(moving_links_map_); }

  /**********************************/
  /*** NON-MOVING LINKS *************/
  /**********************************/

  /**
   * @brief Returns number of non-moving links
   * @return Number of non-moving links
   */
  inline size_t nonMovingLinksSize() const { return non_moving_links_map_.size(); }

  /**
   * @brief Checks if non-moving links are available
   * @param idx link index to look up
   * @return True, if non-moving links are available
   */
  inline bool hasNonMovingLinks() const { return !non_moving_links_map_.empty(); }
  inline bool hasNonMovingLink(const LinkIndex& idx) const { return non_moving_links_map_.find(idx) != non_moving_links_map_.end(); }

  /**
   * @brief Updates non-moving links by given input
   * @param idx link index to look up
   * @param data non-moving link to be updated
   */
  void updateNonMovingLink(const LinkIndex& idx, const NonMovingDataType& data)
  {
    non_moving_links_map_[idx] = data;
    removeMovingLink(idx);
  }

  /**
   * @brief Removes non-moving link
   * @param idx link index to delete
   */
  void removeNonMovingLink(const LinkIndex& idx) { non_moving_links_map_.erase(idx); }

  /**
   * @brief Tries to return non-moving link given link index.
   * @param idx link index to look up
   * @param step_data [out] return variable in which found non-moving link is referenced
   * @return True, if non-moving link was for given link index is available
   */
  bool getNonMovingLink(const LinkIndex& idx, NonMovingDataType& data) const
  {
    typename NonMovingDataMap::const_iterator itr = non_moving_links_map_.find(idx);
    if (itr != non_moving_links_map_.end())
    {
      data = itr->second;
      return true;
    }
    else
      return false;
  }

  /**
   * @brief Tries to return non-moving link for given link index.
   * @param idx link index to look up
   * @return Non-moving link. If no entry was found for given input link index,
   * a null pointer is returned.
   */
  NonMovingDataType getNonMovingLink(const LinkIndex& idx) const
  {
    NonMovingDataType link;
    getNonMovingLink(idx, link);
    return link;
  }

  /**
   * @brief Exposes the internal non-moving link map
   * @return Internal non-moving link map
   */
  inline const NonMovingDataMap& getNonMovingLinks() const { return non_moving_links_map_; }
  inline NonMovingDataMap& getNonMovingLinks() { return non_moving_links_map_; }

  /**
   * @brief Returns list of all link indeces represented by all non-moving links
   * @return List of link indeces
   */
  inline LinkIndexArray getNonMovingLinkIndeces() const { return keysAsArray<LinkIndexArray>(non_moving_links_map_); }

protected:
  StepIndex idx_;  // index number of step; all succeeding steps must have an unique increasing index

  MovingDataMap moving_links_map_;
  NonMovingDataMap non_moving_links_map_;
};

L3_STATIC_ASSERT_MOVEABLE(StepIndex)
}  // namespace l3

#endif
