#include "aerial_autonomy/controllers/obstacle_avoidance_reference_controller.h"

bool ObstacleAvoidanceReferenceController::runImplementation(
    std::pair<PositionYaw, MultipleSphericalObstacles> sensor_data,
    PositionYaw goal, ReferenceTrajectoryPtr<ParticleState, Snap> control) {
  // PositionYaw start = std::get<0>(sensor_data);
  // int const dim_way_points = num_way_points_*3;

  // Build the problem.
  ceres::Problem problem;
  double way_points[] = {1.05, 1.01, 0.98};
  double time_intervals[] = {1, 1};

  ceres::CostFunction *cost_function =
      new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 3, 2, 1>(
          new CostFunctor(sensor_data, goal));

  problem.AddResidualBlock(cost_function, NULL, way_points, time_intervals);
  ceres::Solver::Options options;
  // options.linear_solver_type = ceres::DENSE_SCHUR;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return true;
}

ControllerStatus
ObstacleAvoidanceReferenceController::isConvergedImplementation(
    std::pair<PositionYaw, MultipleSphericalObstacles> sensor_data,
    PositionYaw goal) {
  ControllerStatus controller_status = ControllerStatus::Active;

  controller_status.setStatus(ControllerStatus::Completed);
  return controller_status;
}
