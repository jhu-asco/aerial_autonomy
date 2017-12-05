#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <glog/logging.h>

#include <chrono>

/**
 * @brief A velocity controller that provides rpyt commands to
 * achieve desired velocity
 */
class RPYTBasedVelocityController
    : public Controller<std::tuple<VelocityYawRate, double>, VelocityYawRate,
                        RollPitchYawRateThrust> {
public:
  /**
  * @brief Constructor which takes in a config
  *
  * @param config Controller config
  *
  * @param controller_timer_duration Timestep in seconds
  */
  RPYTBasedVelocityController(
      RPYTBasedVelocityControllerConfig config,
      std::chrono::duration<double> controller_timer_duration)
      : config_(config), controller_timer_duration_(controller_timer_duration) {
    RPYTBasedVelocityControllerConfig check_config = config_;
    CHECK_GE(check_config.kp_xy(), 0) << "negative kp_xy ! exiting";
    CHECK_GE(check_config.kp_z(), 0) << "negative kp_z ! exiting";
    CHECK_GE(check_config.ki_xy(), 0) << "negative ki_xy ! exiting";
    CHECK_GE(check_config.ki_z(), 0) << "negative ki_z ! exiting";
    CHECK_GE(check_config.kt(), 0) << "negative kt ! exiting";
    CHECK_GE(controller_timer_duration_.count(), 0)
        << "negative controller rate ! exiting";

    DATA_HEADER("rpyt_based_velocity_controller") << "Errorx"
                                                  << "Errory"
                                                  << "Errorz"
                                                  << "Erroryawrate"
                                                  << "Vx"
                                                  << "Vy"
                                                  << "Vz"
                                                  << "Yawrate"
                                                  << "Goalvx"
                                                  << "Goalvy"
                                                  << "Goalvz"
                                                  << "Goalvyawrate"
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
    cumulative_error.yaw_rate = 0.0;
  }
  /**
  * @brief update config
  */
  void updateConfig(RPYTBasedVelocityControllerConfig &config) {
    config_ = config;
  }
  /**
  * @brief get config
  */
  RPYTBasedVelocityControllerConfig getConfig() {
    RPYTBasedVelocityControllerConfig config = config_;
    return config;
  }

protected:
  /**
   * @brief Run the control loop.  Uses a rpyt controller to achieve the
   * desired velocity.
   * @param sensor_data Current velocity, yaw_rate, yaw
   * @param goal Goal velocity
   * @param control RPYT command to send to hardware
   * @return true if rpyt command to reach goal is found
   */
  virtual bool
  runImplementation(std::tuple<VelocityYawRate, double> sensor_data,
                    VelocityYawRate goal, RollPitchYawRateThrust &control);
  /**
  * @brief Check if RPYT based velocity controller converged
  *
  * @param sensor_data Current velocity yaw
  * @param goal Goal velocity yaw
  *
  * @return  True if sensor data is close to goal
  */
  virtual ControllerStatus
  isConvergedImplementation(std::tuple<VelocityYawRate, double> sensor_data,
                            VelocityYawRate goal);
  Atomic<RPYTBasedVelocityControllerConfig>
      config_; ///< Controller configuration
  VelocityYawRate cumulative_error;
  const std::chrono::duration<double> controller_timer_duration_;
};
