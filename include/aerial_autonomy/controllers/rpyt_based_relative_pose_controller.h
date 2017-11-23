#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "rpyt_based_relative_pose_controller_config.pb.h"

#include <tuple>

#include <tf/tf.h>

/**
 * @brief A pose controller that keeps a pose relative to some feedback
 * pose using a velocity controller and a rpyt controller.
 * Note 1: Only the yaw of the desired pose is tracked
 * since a quadrotor is underactuated and cannot arbitrarily control roll/pitch
 * while hovering
 */
class RPYTBasedRelativePoseController
    : public Controller<
          std::tuple<tf::Transform, tf::Transform, VelocityYawRate>,
          PositionYaw, RollPitchYawRateThrust> {
public:
  /**
  * @brief Constructor
  */
  RPYTBasedRelativePoseController(RPYTBasedRelativePoseControllerConfig config,
                                  double controller_timer_duration)
      : config_(config),
        velocity_based_relative_pose_controller_(
            config.velocity_based_relative_pose_controller_config(),
            controller_timer_duration),
        rpyt_based_velocity_controller_(
            config.rpyt_based_velocity_controller_config(),
            controller_timer_duration) {}
  /**
   * @brief Destructor
   */
  virtual ~RPYTBasedRelativePoseController() {}

  void resetIntegrator() {
    velocity_based_relative_pose_controller_.resetIntegrator();
    rpyt_based_velocity_controller_.resetCumulativeError();
  }

protected:
  /**
   * @brief Run the control loop.  Uses a velocity controller to keep a desired
   * pose relative to a tracked pose.
   * @param sensor_data Pose of controlled point and tracked
   * pose, along with current velocity and yawrate.
   * NOTE: Both poses need to be given in the frame in which the
   * velocity command is executed
   *
   * @param goal Goal relative position and yaw in tracked pose frame
   * @param control Velocity command
   * @return True if controller is successful in running
   */
  virtual bool runImplementation(
      std::tuple<tf::Transform, tf::Transform, VelocityYawRate> sensor_data,
      PositionYaw goal, RollPitchYawRateThrust &control);
  /**
  * @brief Check if controller converged
  *
  * @param sensor_data Current control pose, tracked pose, current velocity
  * and yaw rate
  * @param goal Goal relative position and yaw in tracked pose frame
  * NOTE: Both poses need to be given in the frame in which the
  * velocity command is executed
  *
  * @return controller status that contains an enum and debug information.
  */
  virtual ControllerStatus isConvergedImplementation(
      std::tuple<tf::Transform, tf::Transform, VelocityYawRate> sensor_data,
      PositionYaw goal);

private:
  /**
   * @brief Config that stores velocity based relative pose controller config
   */
  RPYTBasedRelativePoseControllerConfig config_;
  VelocityBasedRelativePoseController velocity_based_relative_pose_controller_;
  RPYTBasedVelocityController rpyt_based_velocity_controller_;
};
