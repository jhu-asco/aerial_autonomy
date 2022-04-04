#pragma once
#include <aerial_autonomy/filters/base_filter.h>
#include <glog/logging.h>
#include <stdexcept>

/**
 * @brief Decaying exponential filter
 *
 * @tparam T type of data being filtered
 */
template <class T> class DecayingExponentialFilter : public AbstractDiscreteFilter<T> {
public:
  /**
   * @brief Constructor
   *
   * @param gain mixing gain
   * @param steps steps to take before getting to the set gain
   */
  DecayingExponentialFilter(double gain, int steps) : gain_(gain), steps_(steps), filter_data_available_(false) {
    CHECK_LE(gain_, 1.0)
        << "Exponential gain should be less than or equal to 1.0";
    CHECK_GE(gain_, 0.0)
        << "Exponential gain should be greater than or equal to 0.0";
  }
  /**
   * @brief Set filter data to specified input
   *
   * @param input value
   */
  void setFilterData(T input) {
    filter_data_ = input;
    filter_data_available_ = true;
  }
  /**
   * @brief Add sensor data
   *
   * @param sensor_data
   */
  void add(T sensor_data) {
    if (!filter_data_available_) {
      filter_data_ = sensor_data;
      filter_data_available_ = true;
    } else {
      double gain = gain_;
      if (steps_taken_ < steps_)
      {
        gain = 1.0 - (1.0 - gain_) * (steps_taken_ / steps_);
        steps_taken_ += 1;
      }
      filter_data_ = filter_data_ + (sensor_data - filter_data_) * gain;
    }
  }
  /**
   * @brief get filtered data
   *
   * @return filtered data
   */
  T getFilterData() const {
    if (!filter_data_available_)
      throw std::runtime_error("No sensor data to initialize filter yet!");
    return filter_data_;
  }

  /**
  * @brief Reset the filter
  */
  void reset() { filter_data_available_ = false; }

  /**
  * @brief Check if filter data is available
  * @return True if available, false otherwise
  */
  bool isDataAvailable() const { return filter_data_available_; }

  bool isDecayComplete()
  {
    if (steps_taken_ < steps_)
    {
      return false;
    }
    else 
    {
      return true;
    }
  }

private:
  double gain_;                ///< Mixing gain
  int steps_;                  ///< Steps to take to get to gain
  int steps_taken_ = 0;        ///< Number of steps taken so far
  T filter_data_;              ///< Filtered sensor data
  bool filter_data_available_; ///< Flag to initialize filtered data
};
