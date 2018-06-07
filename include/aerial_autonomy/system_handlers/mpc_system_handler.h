#pragma once
#include "aerial_autonomy/system_handlers/uav_arm_system_handler.h"

/**
 * @brief Owns all of the autonomous system components and is responsible for
 * thread management. Also owns a common system handler object to
 * handle creation of state machine and connecting it to GUI.
 *
 * @tparam LogicStateMachineT Logic state machine to use
 * @tparam EventManagerT Event manager to use
 */
template <class LogicStateMachineT, class EventManagerT>
class MPCSystemHandler
    : public UAVArmSystemHandler<LogicStateMachineT, EventManagerT> {
  /**
   * @brief Base class for MPC system handler
   */
  using BaseClass = UAVArmSystemHandler<LogicStateMachineT, EventManagerT>;

public:
  /**
   * @brief Constructor
   * @param config Proto configuration parameters
   */
  MPCSystemHandler(UAVSystemHandlerConfig &config,
                   const BaseStateMachineConfig &state_machine_config)
      : BaseClass(config, state_machine_config),
        mpc_visualization_timer_(
            std::bind(&UAVArmSystem::visualizeMPC, std::ref(this->uav_system_)),
            std::chrono::milliseconds(
                config.uav_arm_system_handler_config()
                    .mpc_visualization_timer_duration())) {
    if (config.uav_system_config()
            .uav_vision_system_config()
            .uav_arm_system_config()
            .visualize_mpc_trajectories()) {
      mpc_visualization_timer_.start();
    }
  }

private:
  AsyncTimer mpc_visualization_timer_; ///< Timer for running arm controller
};
