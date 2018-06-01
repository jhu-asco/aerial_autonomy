#pragma once
#include "aerial_autonomy/types/reference_trajectory.h"
#include "arm_sine_controller_config.pb.h"
#include "spiral_reference_trajectory_config.pb.h"
#include <Eigen/Dense>

class SpiralReferenceTrajectory
    : public ReferenceTrajectory<Eigen::VectorXd, Eigen::VectorXd> {
public:
  using StateT = Eigen::VectorXd;
  using ControlT = Eigen::VectorXd;
  SpiralReferenceTrajectory(SpiralReferenceTrajectoryConfig config,
                            ArmSineControllerConfig arm_config,
                            Eigen::Vector3d current_position,
                            double current_yaw, double kt);
  std::pair<StateT, ControlT> atTime(double t);
  void getRP(double &roll, double &pitch, double yaw,
             Eigen::Vector3d acceleration_vector);

private:
  SpiralReferenceTrajectoryConfig config_;
  ArmSineControllerConfig arm_config_;
  Eigen::Vector3d current_position_;
  double current_yaw_;
  double kt_;
  static constexpr double epsilon = 1e-6;
};
