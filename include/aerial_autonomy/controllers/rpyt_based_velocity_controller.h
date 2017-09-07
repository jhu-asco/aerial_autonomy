#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <glog/logging.h>
/**
 * @brief A velocity controller that provides rpyt commands to
 * achieve desired velocity
 */

class RPYTBasedVelocityController
    : public Controller<VelocityYaw, VelocityYaw, RollPitchYawThrust> {
public:
  /**
  * @brief Constructor which takes in a config
  *
  * @param config Controller config
  *
  * @param controller_timer_duration Timestep in seconds
  */
  RPYTBasedVelocityController(RPYTBasedVelocityControllerConfig &config,
                              double controller_timer_duration)
      : config_(config), controller_timer_duration_(controller_timer_duration) {
    RPYTBasedVelocityControllerConfig check_config = config_;
    CHECK_GE(check_config.kp(), 0) << "negative kp ! exiting";
    CHECK_GE(check_config.ki(), 0) << "negative ki ! exiting";
    CHECK_GE(check_config.kt(), 0) << "negative kt ! exiting";

    DATA_HEADER("rpyt_based_velocity_controller") << "Errorx"
                                                  << "Errory"
                                                  << "Errorz"
                                                  << "Erroryaw"
                                                  << "Goalvx"
                                                  << "Goalvy"
                                                  << "Goalvz"
                                                  << "Goalvyaw"
                                                  << DataStream::endl;
  }
  /**
  *
  * @brief reset cumulative_error
  *
  */
  void resetCumulativeError() {
    cumulative_error.x = 0.0;
    cumulative_error.y = 0.0;
    cumulative_error.z = 0.0;
    cumulative_error.yaw = 0.0;
  }
  /**
  *
  */
  void updateConfig(RPYTBasedVelocityControllerConfig &config) {
    config_ = config;
  }

protected:
  /**
   * @brief Run the control loop.  Uses a rpyt controller to achieve the
   * desired velocity.
   * @param sensor_data Current velocity
   * @param goal Goal velocity
   * @param control RPYT command to send to hardware
   * @return true if rpyt command to reach goal is found
   */
  virtual bool runImplementation(VelocityYaw sensor_data, VelocityYaw goal,
                                 RollPitchYawThrust &control);
  /**
  * @brief Check if RPYT based velocity controller converged
  *
  * @param sensor_data Current velocity yaw
  * @param goal Goal velocity yaw
  *
  * @return  True if sensor data is close to goal
  */
  virtual ControllerStatus isConvergedImplementation(VelocityYaw sensor_data,
                                                     VelocityYaw goal);
  Atomic<RPYTBasedVelocityControllerConfig>
      config_; ///< Controller configuration
  VelocityYaw cumulative_error;
  double controller_timer_duration_;
};
