#pragma once
#include "state_estimator_backend.h"
#include "state_estimator_frontend.h"
#include <aerial_autonomy/common/chrono_stamped.h>

class StateEstimator {
public:
  StateEstimator(StateEstimatorConfig config,
                 parsernode::Parser &drone_hardware);
  void process();
  // Getters for accessing latest data
  common::Stamped<Transform> getPose();
  common::Stamped<Transform> getTargetPose();
  common::Stamped<Eigen::Vector3d> getBodyVelocity();
  common::Stamped<Eigen::Vector3d> getSpatialVelocity();
  common::Stamped<Eigen::Vector3d> getBodyAngularVelocity();
  StateEstimatorStatus getStatus();

private:
  StateEstimatorBackend backend_;
  StateEstimatorFrontend frontend_;
  StateEstimatorStatus status_;
};
