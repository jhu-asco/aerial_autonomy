#pragma once
#include <aerial_autonomy/filters/base_filter.h>
#include <glog/logging.h>
#include <stdexcept>

/**
 * @brief Exponential filter
 *
 * @tparam T type of data being filtered
 */
template <class T> class ExponentialFilter : public AbstractDiscreteFilter<T> {
public:
  /**
   * @brief Constructor
   *
   * @param gain mixing gain
   */
  ExponentialFilter(double gain) : gain_(gain), filter_data_available_(false) {
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
      filter_data_ = filter_data_ + (sensor_data - filter_data_) * gain_;
    }
  }
  /**
   * @brief get filtered data
   *
   * @return filtered data
   */
  T getFilterData() {
    if (!filter_data_available_)
      throw std::runtime_error("No sensor data to initialize filter yet!");
    return filter_data_;
  }

  void reset() { filter_data_available_ = false; }

private:
  double gain_;                ///< Mixing gain
  T filter_data_;              ///< Filtered sensor data
  bool filter_data_available_; ///< Flag to initialize filtered data
};
