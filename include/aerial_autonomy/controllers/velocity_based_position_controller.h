#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "velocity_based_position_controller_config.pb.h"
#include <aerial_autonomy/log/log.h>

/**
 * @brief A position controller that sends velocity commands to the hardware
 */
class VelocityBasedPositionController
    : public Controller<PositionYaw, PositionYaw, VelocityYawRate> {
public:
  /**
  * @brief Constructor with default configuration
  */
  VelocityBasedPositionController()
      : VelocityBasedPositionController(
            VelocityBasedPositionControllerConfig()) {}
  /**
  * @brief Constructor which takes a configuration
  *
  * @param config specifies the gains on position, yaw and integrator gains etc
  * @param dt_ specifies the time difference between two runs
  */
  VelocityBasedPositionController(VelocityBasedPositionControllerConfig config,
                                  double dt_ = 0.02)
      : config_(config), cumulative_error(0, 0, 0, 0), dt(dt_) {
    DATA_HEADER("velocity_based_position_controller") << "x_diff"
                                                      << "y_diff"
                                                      << "z_diff"
                                                      << "yaw_diff"
                                                      << "xi_diff"
                                                      << "yi_diff"
                                                      << "zi_diff"
                                                      << "yawi_diff"
                                                      << "cmd_vx"
                                                      << "cmd_vy"
                                                      << "cmd_vz"
                                                      << "cmd_yawrate"
                                                      << DataStream::endl;
  }
  /**
   * @brief Destructor
   */
  virtual ~VelocityBasedPositionController() {}

  /**
   * @brief Compute the integrator internally based on
   * back calculation
   */
  void resetIntegrator();

  /**
   * @brief If the command (p_command + integrator) is saturated, the function
   * resets the integrator to ensure p_command + integrator = saturation.
   *
   * @param integrator
   * @param p_command
   * @param saturation
   *
   * @return the command after resetting integrator
   */
  inline double backCalculate(double &integrator, const double &p_command,
                              const double &saturation);

  /**
   * @brief Getter for internal cumulative error stored
   *
   * @return cumulative position_yaw error multiplied by dt and i gain
   */
  PositionYaw getCumulativeError() const { return cumulative_error; }

protected:
  /**
   * @brief Run the control loop.  Uses a velocity controller to achieve the
   * desired position.
   * @param sensor_data Current position
   * @param goal Goal position
   * @param control Velocity command to send to hardware
   * @return true if velocity command to reach goal is found
   */
  virtual bool runImplementation(PositionYaw sensor_data, PositionYaw goal,
                                 VelocityYawRate &control);
  /**
  * @brief Check if velocity based position controller converged
  *
  * @param sensor_data Current position yaw
  * @param goal Goal position yaw
  *
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(PositionYaw sensor_data,
                                                     PositionYaw goal);
  VelocityBasedPositionControllerConfig config_; ///< Controller configuration
  PositionYaw cumulative_error; ///< Error integrated over multiple runs
  double dt; ///< Time diff between different successive runImplementation calls
};
