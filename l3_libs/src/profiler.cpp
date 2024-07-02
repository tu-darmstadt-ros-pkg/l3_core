#include <l3_libs/profiler.h>

namespace l3
{
  Profiler::Profiler(size_t max_num_samples)
    : max_num_samples_(max_num_samples)
    , current_sample_(0)
  {
    samples_.reserve(max_num_samples_);
  }

  void Profiler::start()
  {
    process_start_ = std::chrono::high_resolution_clock::now();
  }

  void Profiler::stop()
  {
    double sample = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - process_start_).count();

    if (samples_.size() < max_num_samples_)
    {
      samples_.push_back(sample);
      current_sample_ = samples_.size() - 1;
    }
    else
    {
      current_sample_ = (current_sample_ + 1) % max_num_samples_;
      samples_[current_sample_] = sample;
    }
  }

  void Profiler::reset()
  {
    current_sample_ = 0;
    samples_.clear();
  }

  double Profiler::getStatistics(double& mean, double& min, double& max) const
  {
    // Return nan if no samples
    if (samples_.empty())
    {
      mean = std::numeric_limits<double>::quiet_NaN();
      min = std::numeric_limits<double>::quiet_NaN();
      max = std::numeric_limits<double>::quiet_NaN();
      return std::numeric_limits<double>::quiet_NaN();
    }

    mean = std::accumulate(samples_.begin(), samples_.end(), 0.0) / samples_.size();
    min = *std::min_element(samples_.begin(), samples_.end());
    max = *std::max_element(samples_.begin(), samples_.end());
    return getLastDuration();
  }

  double Profiler::getLastDuration() const
  {
    // Return nan if no samples
    if (samples_.empty())
      return std::numeric_limits<double>::quiet_NaN();

    return samples_[current_sample_];
  }

  double Profiler::getMeanDuration() const
  {
    // Return nan if no samples
    if (samples_.empty())
      return std::numeric_limits<double>::quiet_NaN();

    return std::accumulate(samples_.begin(), samples_.end(), 0.0) / samples_.size();
  }

  double Profiler::getMinDuration() const
  {
    // Return nan if no samples
    if (samples_.empty())
      return std::numeric_limits<double>::quiet_NaN();

    return *std::min_element(samples_.begin(), samples_.end());
  }

  double Profiler::getMaxDuration() const
  {
    // Return nan if no samples
    if (samples_.empty())
      return std::numeric_limits<double>::quiet_NaN();

    return *std::max_element(samples_.begin(), samples_.end());
  }
}  // namespace l3
