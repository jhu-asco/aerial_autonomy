#pragma once

// Copyright (c) 2012, Sebastian Jeltsch (sjeltsch@kip.uni-heidelberg.de)
// Distributed under the terms of the GPLv2 or newer

#include <stdexcept>

namespace fab {
namespace exception {

struct UnknownKey : public std::runtime_error {
  UnknownKey() : std::runtime_error("Factory: unknown key") {}
  virtual ~UnknownKey() throw() {}
};

struct BadArguments : public std::runtime_error {
  BadArguments() : std::runtime_error("Factory: non-matching arguments") {}
  virtual ~BadArguments() throw() {}
};

} // exception
} // fab
