#include "aerial_autonomy/controllers/relative_position_controller.h"
#include <glog/logging.h>

bool RelativePositionController::runImplementation(
    std::tuple<Position, Position> sensor_data, Position goal,
    Position &control) {
  control = std::get<1>(sensor_data) + goal;
  return true;
}

bool RelativePositionController::isConvergedImplementation(
    std::tuple<Position, Position> sensor_data, Position goal) {
  Position current_position = std::get<0>(sensor_data);
  Position tracked_position = std::get<1>(sensor_data);

  Position relative_position = current_position - tracked_position;

  const double &tolerance_pos = config_.goal_position_tolerance();
  if (std::abs(relative_position.x - goal.x) < tolerance_pos &&
      std::abs(relative_position.y - goal.y) < tolerance_pos &&
      std::abs(relative_position.z - goal.z) < tolerance_pos) {
    VLOG(1) << "Reached goal";
    return true;
  }
  return false;
}
