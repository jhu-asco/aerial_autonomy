#include <aerial_autonomy/controllers/rpyt_based_position_controller.h>

bool RPYTBasedPositionController::runImplementation(
  std::tuple<PositionYaw, VelocityYaw> sensor_data,
  PositionYaw goal,
  RollPitchYawThrust &control){
  
  // Get a goal velocity for given sensor data and goal
  vel_pos_ctlr.setGoal(goal);
  VelocityYaw goal_vel;
  vel_pos_ctlr.run(std::get<0>(sensor_data), goal_vel);

  rpyt_vel_ctlr.setGoal(goal_vel);
  rpyt_vel_ctlr.run(std::get<1>(sensor_data), control);

  return true;
}

bool RPYTBasedPositionController::isConvergedImplementation(
  std::tuple<PositionYaw, VelocityYaw> sensor_data, PositionYaw goal) {

  vel_pos_ctlr.setGoal(goal);
  return vel_pos_ctlr.isConverged(std::get<0>(sensor_data));
}
