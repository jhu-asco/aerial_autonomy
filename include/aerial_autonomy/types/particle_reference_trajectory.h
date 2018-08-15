#pragma once

#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "particle_reference_config.pb.h"
#include <tf/tf.h>

/**
* @brief A reference trajectory that used a exponential controller
* to propagate a particle state
*
* @tparam StateT   type of state
* @tparam ControlT type of control
*/
template <class StateT, class ControlT>
class ParticleTrajectory : public ReferenceTrajectory<StateT, ControlT> {
public:
  /**
  * @brief Constructor
  * @param goal_state The goal state of the waypoint
  * @param goal_control The goal control of the waypoint
  */
  ParticleTrajectory(PositionYaw goal_state, PositionYaw start_state,
                     ParticleReferenceConfig config)
      : goal_state_(goal_state), start_state_(start_state), config_(config) {}
  /**
   * @brief Update goal state
   *
   * @param goal_state goal
   */
  void setGoal(PositionYaw goal_state) { goal_state_ = goal_state; }

protected:
  PositionYaw goal_state_;         ///< Goal state
  PositionYaw start_state_;        ///< start state
  ParticleReferenceConfig config_; ///< Reference config
};
