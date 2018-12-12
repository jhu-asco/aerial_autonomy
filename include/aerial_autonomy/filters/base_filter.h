#pragma once

/**
 * @brief Abstract filter
 *
 * @tparam T type of element being filtered
 */
template <class T> class AbstractDiscreteFilter {
public:
  /**
   * @brief Add sensor data
   *
   * @param sensor_data
   */
  virtual void add(T sensor_data) = 0;
  /**
   * @brief get filtered data
   *
   * @return filtered data
   */
  virtual T getFilterData() = 0;
  /**
   * @brief Andd sensor data, filter
   *
   * @param sensor_data input sensor data
   *
   * @return filter sensor data
   */
  T addAndFilter(T sensor_data) {
    add(sensor_data);
    return getFilterData();
  }
  /**
   * @brief Reset filter by clearing buffers
   *
   * and removing any dependencies on previous
   * data
   */
  virtual void reset() = 0;
};
