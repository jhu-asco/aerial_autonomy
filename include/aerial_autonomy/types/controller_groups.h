#pragma once

/**
* @brief Controller groups. Enum ID must be
* contiguous.
*/
enum class ControllerGroup {
  UAV,         ///< Only aerial vehicle
  HighLevel,   ///< Controller that outputs to another controller (For example
               /// High level MPC/ planners)
  Arm,         ///< Only arm
  First = UAV, // This should always point to the first in the list
  Last = Arm   // This should always point to the last in the list
};
