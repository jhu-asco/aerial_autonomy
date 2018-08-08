#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "particle_reference_config.pb.h"

#include <tf/tf.h>
#include <utility>

class QuadParticleReferenceController
    : public Controller<
          std::pair<PositionYaw, tf::Transform>, PositionYaw,
          ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>> {
public:
  /**
  * @brief Constructor
  */
  QuadParticleReferenceController(ParticleReferenceConfig config);

protected:
  virtual bool runImplementation(
      std::pair<PositionYaw, tf::Transform> sensor_data, PositionYaw goal,
      ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> &control);

  virtual ControllerStatus
      isConvergedImplementation(std::pair<PositionYaw, tf::Transform>,
                                PositionYaw) {
    return ControllerStatus(ControllerStatus::Status::Active);
  }

private:
  ParticleReferenceConfig config_;
};
