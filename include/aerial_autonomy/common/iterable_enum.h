#pragma once

/**
 * @brief Iterable wrapper class for iterating over enums.  Only works for enums
 * that are contiguous.
 * @tparam T enum to iterate over
 */
template <typename T> class IterableEnum {
public:
  /**
   * @brief Iterator for the wrapper class
   */
  class Iterator {
  public:
    /**
     * @brief Constructor
     */
    Iterator(int value) : m_value_(value) {}

    /**
     * @brief Returns underlying iterator value
     */
    T operator*(void)const { return (T)m_value_; }

    /**
     * @brief Increment iterator
     */
    void operator++(void) { ++m_value_; }

    /**
     * @brief Compare iterator using underlying value
     */
    bool operator!=(Iterator rhs) { return m_value_ != rhs.m_value_; }

  private:
    int m_value_; ///< The current enum value
  };
};

/**
 * @brief Iterator corresponding to the beginning of the enum
 * @tparam T enum type
 */
template <typename T>
typename IterableEnum<T>::Iterator begin(IterableEnum<T>) {
  return typename IterableEnum<T>::Iterator((int)T::First);
}

/**
 * @brief Iterator corresponding to the last value of the enum
 * @tparam T enum type
 */
template <typename T> typename IterableEnum<T>::Iterator end(IterableEnum<T>) {
  return typename IterableEnum<T>::Iterator(((int)T::Last) + 1);
}