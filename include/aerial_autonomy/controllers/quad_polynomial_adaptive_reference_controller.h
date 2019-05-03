#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/particle_state_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"
#include "polynomial_reference_config.pb.h"
#include "aerial_autonomy/controllers/quad_polynomial_reference_controller.h"
#include <tf/tf.h>
#include <utility>

/**
 * @brief Controller that generates a reference trajectory based on
 *
 * a goal waypoint
 */
class QuadPolynomialAdaptiveReferenceController
    : public Controller<
          std::pair<PositionYaw, tf::Transform>, PositionYaw,
          ReferenceTrajectoryPtr<ParticleStateYaw, Snap>> {
public:
  /**
   * @brief constructor
   *
   * @param config polynomial reference config
   */
  QuadPolynomialAdaptiveReferenceController(PolynomialReferenceConfig config);

  /**
   * @brief Specify whether to add noise at end
   *
   * @param flag true if we want to add noise at end of reference
   */
  void useNoise(bool flag);

  /**
   * @brief reset controller status
   */
  void reset();

protected:
  /**
   * @brief generate reference trajectory based on sensor data, goal
   *
   * @param sensor_data current positionyaw, object transform in quad frame
   * @param goal Goal in object frame
   * @param control Reference in global frame
   *
   * @return true if able to generate the trajectory
   */
  virtual bool runImplementation(
      std::pair<PositionYaw, tf::Transform> sensor_data, PositionYaw goal,
      ReferenceTrajectoryPtr<ParticleStateYaw, Snap> &control);

  /**
   * @brief Status of the controller
   *
   * Always active
   *
   * @param std::pair   sensor data
   * @param PositionYaw goal
   *
   * @return  status of the controller
   */
  virtual ControllerStatus
      isConvergedImplementation(std::pair<PositionYaw, tf::Transform>,
                                PositionYaw) {
    return ControllerStatus(ControllerStatus::Status::Active);
  }

private:
  PolynomialReferenceConfig config_; ///< Config about the reference trajectory
};
