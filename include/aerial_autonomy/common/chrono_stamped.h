#pragma once
#include <chrono>

namespace common {
template <typename T> class Stamped : public T {
public:
  using time_point =
      std::chrono::time_point<std::chrono::high_resolution_clock>;
  time_point stamp;
  Stamped() = default;
  Stamped(const T &input, const time_point &timestamp)
      : T(input), stamp_(timestamp) {}
  Stamped(const T &input)
      : Stamped(input, std::chrono::high_resolution_clock::now()) {}
  void setData(const T &input) { *static_cast<T *>(this) = input; };
};
}
