#pragma once
#include "aerial_autonomy/types/snap.h"

/**
* @brief Store flat output space
* controls of a quad
*/
struct QuadFlatSpaceControls {
  /**
  * @brief Implicit Constructor
  */
  QuadFlatSpaceControls() : ga2(0) {}
  /**
  * @brief Explicit Constructor
  */
  QuadFlatSpaceControls(Snap snap, double yaw_acceleration)
      : s(snap), ga2(yaw_acceleration) {}
  Snap s;     ///< snap
  double ga2; ///< yaw acceleration
};