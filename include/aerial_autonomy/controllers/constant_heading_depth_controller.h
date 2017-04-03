#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "constant_heading_depth_controller_config.pb.h"

/**
 * @brief A position controller that keeps a constant heading while attempting
 * to keep a certain distance from the feedback position
 */
class ConstantHeadingDepthController
    : public Controller<PositionYaw, Position, VelocityYawRate> {
public:
  /**
  * @brief Constructor with default configuration
  */
  ConstantHeadingDepthController()
      : config_(ConstantHeadingDepthControllerConfig()) {}
  /**
  * @brief Constructor which takes a configuration
  */
  ConstantHeadingDepthController(ConstantHeadingDepthControllerConfig config)
      : config_(config) {}
  /**
   * @brief Destructor
   */
  virtual ~ConstantHeadingDepthController() {}

protected:
  /**
   * @brief Run the control loop.  Uses a velocity controller to keep a desired
   * distance from a tracked point.
   * @param sensor_data Current direction vector multiplied times distance
   * @param goal Goal direction vector multiplied times distance
   * @return Velocity command to send to hardware
   */
  virtual VelocityYawRate runImplementation(PositionYaw sensor_data,
                                            Position goal);
  ConstantHeadingDepthControllerConfig config_; ///< Controller configuration
};
