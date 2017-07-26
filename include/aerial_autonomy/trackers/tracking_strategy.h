#pragma once

#include "aerial_autonomy/types/position.h"

#include <tuple>
#include <vector>

class TrackingStrategy {
public:
  virtual bool initialize(
      const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors) = 0;
  virtual bool getTrackingVector(
      const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors,
      std::tuple<uint32_t, Position> &tracking_vector) = 0;
};
