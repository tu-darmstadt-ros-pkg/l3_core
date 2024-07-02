//=================================================================================================
// Copyright (c) 2024, Alexander Stumpf, TU Darmstadt
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

#pragma once

#include <chrono>
#include <numeric>

#include <l3_libs/types/types.h>

namespace l3
{
class Profiler
{
public:
  // typedefs
  typedef l3::SharedPtr<Profiler> Ptr;
  typedef l3::SharedPtr<const Profiler> ConstPtr;

  /**
   * @brief Constructor
   * @param max_num_samples Number of samples to store
   */
  Profiler(size_t max_num_samples = 100);

  /**
   * @brief Start the profiling of a code section
   */
  void start();

  /**
   * @brief Stop the profiling of a code section
   */
  void stop();

  /**
   * @brief Reset the profiling
   */
  void reset();

  /**
   * @brief Get the statistics of the profiling
   * @param mean [out] Returns mean duration in milliseconds, nan if no samples
   * @param min [out] Returns minimum duration in milliseconds, nan if no samples
   * @param max [out] Returns maximum duration in milliseconds, nan if no samples
   * @return Last duration in milliseconds, nan if no samples
   */
  double getStatistics(double& mean, double& min, double& max) const;

  /**
   * @brief Get the duration of the last profiling
   * @return Duration in milliseconds, nan if no samples
   */
  double getLastDuration() const;

  /**
   * @brief Get the mean duration of all profiling
   * @return Duration in milliseconds, nan if no samples
   */
  double getMeanDuration() const;

  /**
   * @brief Get the minimum duration of all profiling
   * @return Duration in milliseconds, nan if no samples
   */
  double getMinDuration() const;

  /**
   * @brief Get the maximum duration of all profiling
   * @return Duration in milliseconds, nan if no samples
   */
  double getMaxDuration() const;

private:
  std::chrono::high_resolution_clock::time_point process_start_;

  size_t max_num_samples_;
  size_t current_sample_;
  std::vector<double> samples_;
};
}  // namespace l3
