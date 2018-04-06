#include "aerial_autonomy/controllers/quad_higher_level_controller.h"

bool QuadHigherLevelController::runImplementation(
    std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data,
    Trajectory<QuadFlatOutput> goal, Trajectory<QuadFlatOutput> &control) {
  AcadoHigherLevelController controller(config_);
  if (!controller.solve(sensor_data, goal, control)) {
    LOG(WARNING) << "Optimization failed";
    return false;
  }
  QuadFlatOutput goal_state = goal.getAtTime(goal.horizon());
  if (!controller.checkTrajectoryFeasibility(control, std::get<1>(sensor_data),
                                             goal_state)) {
    LOG(WARNING) << "Trajectory does not converge to goal";
    return false;
  }
  return true;
}

ControllerStatus QuadHigherLevelController::isConvergedImplementation(
    std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data,
    Trajectory<QuadFlatOutput> goal) {
  ControllerStatus result = ControllerStatus::Active;
  QuadFlatOutput goal_state = goal.getAtTime(goal.horizon());
  QuadFlatOutput error = goal_state - final_state_;
  double e = config_.tolerance();
  if (abs(error.p.x) < e || abs(error.p.y) < e || abs(error.p.z) < e ||
      abs(error.p.yaw) < e || abs(error.v.x) < e || abs(error.v.y) < e ||
      abs(error.v.z) < e || abs(error.v.yaw_rate) < e || abs(error.a.x) < e ||
      abs(error.a.y) < e || abs(error.a.z) < e || abs(error.j.x) < e ||
      abs(error.j.y) < e || abs(error.j.z) < e)
    result = ControllerStatus::Completed;
  return result;
}
