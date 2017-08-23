#pragma once

// Copyright (c) 2012, Sebastian Jeltsch (sjeltsch@kip.uni-heidelberg.de)
// Distributed under the terms of the GPLv2 or newer

#include "fab/factory.h"
#include <type_traits>

namespace fab {

template <typename T, typename... Args> T *delegate(Args... args) {
  return new T(std::forward<Args>(args)...);
}

template <typename T, typename Base> T &cast(std::unique_ptr<Base> &ptr) {
  return dynamic_cast<T &>(*ptr);
}

template <typename T, typename Base>
T const &cast(std::unique_ptr<Base> const &ptr) {
  return dynamic_cast<T const &>(*ptr);
}

} // fab
