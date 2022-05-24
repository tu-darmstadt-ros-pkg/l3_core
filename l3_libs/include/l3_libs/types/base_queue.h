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

#ifndef L3_BASE_QUEUE_H__
#define L3_BASE_QUEUE_H__

#include <mutex>

#include <l3_libs/types/memory.h>
#include <l3_libs/types/step.h>
#include <l3_libs/types/variant_data.h>
#include <l3_libs/helper.h>

namespace l3
{
/**
 * @brief Generic template data structure for handling step queues (managed list of steps).
 * This data structure is thread safe.
 */
template <typename StepType>
class BaseQueue : protected std::map<StepIndex, StepType>
{
  typedef std::map<StepIndex, StepType> Map;

public:
  // typedefs
  typedef std::pair<const StepIndex, StepType> Entry;
  typedef SharedPtr<BaseQueue<StepType>> Ptr;
  typedef SharedPtr<const BaseQueue<StepType>> ConstPtr;

  // constructor
  BaseQueue()
    : Map()
  {}

  BaseQueue(const BaseQueue& other)
    : BaseQueue(other, SharedLock(other.mutex_))
  {}

  BaseQueue(BaseQueue&& other)
    : BaseQueue(std::move(other), UniqueLock(other.mutex_))
  {}

protected:
  BaseQueue(const BaseQueue& other, SharedLock /*other_lock*/)
    : Map(other)
    , header(other.header)
    , data(other.data)
  {}

  BaseQueue(BaseQueue&& other, UniqueLock /*other_lock*/)
    : Map(std::move(other))
    , header(std::move(other.header))
    , data(std::move(other.data))
  {}

public:
  BaseQueue& swap(BaseQueue& other)
  {
    if (*this != other)
    {
      UniqueLock this_lock(mutex_, boost::defer_lock);
      UniqueLock other_lock(other.mutex_, boost::defer_lock);
      std::lock(this_lock, other_lock);

      Map::swap(other);
      std::swap(header, other.header);
      data.swap(other.data);
    }
    return *this;
  }

  BaseQueue& operator=(const BaseQueue& other)
  {
    if (*this != other)
    {
      UniqueLock this_lock(mutex_, boost::defer_lock);
      SharedLock other_lock(other.mutex_, boost::defer_lock);
      std::lock(this_lock, other_lock);

      Map::operator=(other);
      header = other.header;
      data = other.data;
    }
    return *this;
  }

  BaseQueue& operator=(BaseQueue&& other)
  {
    if (*this != other)
    {
      UniqueLock this_lock(mutex_, boost::defer_lock);
      UniqueLock other_lock(other.mutex_, boost::defer_lock);
      std::lock(this_lock, other_lock);

      Map::operator=(std::move(other));
      header = std::move(other.header);
      data = std::move(other.data);
    }
    return *this;
  }

  // forward base interfaces
  void clear()
  {
    Map::clear();
    header = std_msgs::Header();
    data.clear();
  }

  inline bool empty() const { return Map::empty(); }
  inline size_t size() const { return Map::size(); }

  typedef typename Map::iterator iterator;
  typedef typename Map::const_iterator const_iterator;
  typedef typename Map::reverse_iterator reverse_iterator;
  typedef typename Map::const_reverse_iterator const_reverse_iterator;

  inline typename Map::iterator begin() { return Map::begin(); }
  inline typename Map::const_iterator begin() const { return Map::begin(); }

  inline typename Map::iterator end() { return Map::end(); }
  inline typename Map::const_iterator end() const { return Map::end(); }

  inline typename Map::reverse_iterator rbegin() { return Map::rbegin(); }
  inline typename Map::const_reverse_iterator rbegin() const { return Map::rbegin(); }

  inline typename Map::reverse_iterator rend() { return Map::rend(); }
  inline typename Map::const_reverse_iterator rend() const { return Map::rend(); }

  inline typename Map::iterator find(const StepIndex& step_idx) { return Map::find(step_idx); }
  inline typename Map::const_iterator find(const StepIndex& step_idx) const { return Map::find(step_idx); }

  // new methods
  template <typename Container = std::vector<StepType>>
  void toArray(Container& v) const
  {
    SharedLock lock(mutex_);
    v = valuesAsArray<Container>(*this);
  }

  template <typename Container = std::vector<StepType>>
  inline Container asArray() const
  {
    SharedLock lock(mutex_);
    return valuesAsArray<Container>(*this);
  }

  bool hasStep(const StepIndex& step_idx) const
  {
    SharedLock lock(mutex_);
    return Map::find(step_idx) != Map::end();
  }

  void enqueue(const StepIndex& step_idx, const StepType& step)
  {
    UniqueLock lock(mutex_);
    this->operator[](step_idx) = step;
  }

  StepType pop()
  {
    UniqueLock lock(mutex_);

    StepType step;
    if (!this->empty())
    {
      step = this->begin()->second;
      this->erase(this->begin());
    }
    return step;
  }

  StepType getStep(const StepIndex& step_idx) const
  {
    SharedLock lock(mutex_);

    StepType step;
    getStep(step_idx, step);
    return step;
  }

  bool getStep(const StepIndex& step_idx, StepType& step) const
  {
    SharedLock lock(mutex_);

    typename Map::const_iterator itr = this->find(step_idx);
    if (itr != this->end())
    {
      step = itr->second;
      return true;
    }
    else
      return false;
  }

  /** Returns preceding step in queue */
  StepType getPrevStep(const StepIndex& step_idx) const
  {
    SharedLock lock(mutex_);

    StepType step;
    getPrevStep(step_idx, step);
    return step;
  }

  bool getPrevStep(const StepIndex& step_idx, StepType& step) const
  {
    typename Map::const_iterator itr = std::prev(this->find(step_idx));
    if (itr != this->end())
    {
      step = itr->second;
      return true;
    }
    else
      return false;
  }

  /** Returns succeeding step in queue */
  StepType getNextStep(const StepIndex& step_idx) const
  {
    SharedLock lock(mutex_);

    StepType step;
    getNextStep(step_idx, step);
    return step;
  }

  bool getNextStep(const StepIndex& step_idx, StepType& step) const
  {
    typename Map::const_iterator itr = std::next(this->find(step_idx));
    if (itr != this->end())
    {
      step = itr->second;
      return true;
    }
    else
      return false;
  }

  void removeStep(const StepIndex& index)
  {
    UniqueLock lock(mutex_);
    this->erase(index);
  }

  /**
   * @brief Removes steps in specified range [from_step_idx, to_step_idx]
   * @param from_step_idx first step index
   * @param to_step_idx last step index; if set -1 (default) all steps >=from_step_idx will be deleted
   */
  void removeSteps(const StepIndex& from_step_idx, const StepIndex& to_step_idx = -1)
  {
    if (empty())
      return;

    StepIndex from = std::max(from_step_idx, firstStepIndex());
    if (to_step_idx >= 0 && from > to_step_idx)
      return;

    UniqueLock lock(mutex_);

    typename Map::iterator start_itr = Map::find(from);
    typename Map::iterator end_itr = to_step_idx >= 0 ? Map::find(to_step_idx + 1) : end();
    Map::erase(start_itr, end_itr);
  }

  StepType firstStep() const
  {
    SharedLock lock(mutex_);

    if (this->empty())
      return StepType();
    else
      return this->begin()->second;
  }

  StepIndex firstStepIndex() const
  {
    SharedLock lock(mutex_);

    if (this->empty())
      return -1;
    else
      return firstStep()->getStepIndex();
  }

  StepType lastStep() const
  {
    SharedLock lock(mutex_);

    if (this->empty())
      return StepType();
    else
      return this->rbegin()->second;
  }

  StepIndex lastStepIndex() const
  {
    SharedLock lock(mutex_);

    if (this->empty())
      return -1;
    else
      return lastStep()->getStepIndex();
  }

  std_msgs::Header header;
  VariantDataSet data;  // may contain user specific data

protected:
  mutable Mutex mutex_;
};
}  // namespace l3

#endif
