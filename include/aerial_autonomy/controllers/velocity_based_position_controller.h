#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "velocity_based_position_controller_config.pb.h"

/**
 * @brief A position controller that sends velocity commands to the hardware
 */
class VelocityBasedPositionController
    : public Controller<PositionYaw, PositionYaw, VelocityYaw> {
public:
  /**
  * @brief Constructor with default configuration
  */
  VelocityBasedPositionController()
      : config_(VelocityBasedPositionControllerConfig()) {}
  /**
  * @brief Constructor which takes a configuration
  */
  VelocityBasedPositionController(VelocityBasedPositionControllerConfig config)
      : config_(config) {}
  /**
   * @brief Destructor
   */
  virtual ~VelocityBasedPositionController() {}

protected:
  /**
   * @brief Run the control loop.  Uses a velocity controller to achieve the
   * desired position.
   * @param sensor_data Current position
   * @param goal Goal position
   * @return Velocity command to send to hardware
   */
  virtual VelocityYaw runImplementation(PositionYaw sensor_data,
                                        PositionYaw goal, ControllerStatus &);
  /**
  * @brief Check if velocity based position controller converged
  *
  * @param sensor_data Current position yaw
  * @param goal Goal position yaw
  *
  * @return  True if sensor data is close to goal
  */
  virtual bool isConvergedImplementation(PositionYaw sensor_data,
                                         PositionYaw goal);
  VelocityBasedPositionControllerConfig config_; ///< Controller configuration
};
