#pragma once

#include <boost/thread/mutex.hpp>

/**
 * @brief Template class to create thread-safe variables with internal lock
 * management.
 */
template <class T> class Atomic {
public:
  /**
   * @brief Default constructor
   */
  Atomic() = default;

  /**
   * @brief Constructor that sets member data
   * @param data Value to set member data to
   */
  Atomic(const T &data) { this->set(data); }

  /**
   * @brief Copy constructor
   * @param a Instance to copy
   */
  Atomic(const Atomic<T> &a) { this->set(a.get()); }

  /**
   * @brief Set the data
   * @param data Value to set member data to
   */
  void set(const T &data) {
    boost::mutex::scoped_lock lock(mutex_);
    data_ = data;
  }

  /**
   * @brief Get the data
   * @return The data
   */
  T get() const {
    boost::mutex::scoped_lock lock(mutex_);
    T data_copy = data_;
    return data_copy;
  }

  /**
   * @brief Assignment operator
   * @param a Atomic class whose data we are copying
   */
  void operator=(const Atomic<T> &a) { this->set(a.get()); }

  /**
   * @brief Assignment operator for data
   * @param d Data we are copying
   */
  void operator=(const T &d) { this->set(d); }

  /**
   * @brief Conversion operator
   * @return The data
   */
  operator T() const { return this->get(); }

private:
  T data_;                     ///< Data being stored
  mutable boost::mutex mutex_; ///< Synchronize access to data
};
