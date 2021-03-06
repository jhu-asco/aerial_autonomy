#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "particle_reference_config.pb.h"

#include <tf/tf.h>
#include <utility>

/**
 * @brief Controller that generates a reference trajectory based on
 *
 * a goal waypoint
 */
class QuadParticleReferenceController
    : public Controller<
          std::pair<PositionYaw, tf::Transform>, PositionYaw,
          ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>> {
public:
  /**
   * @brief constructor
   *
   * @param config particle reference config
   */
  QuadParticleReferenceController(ParticleReferenceConfig config);

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
      ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> &control);

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
  ParticleReferenceConfig config_; ///< Config about the reference trajectory
};
