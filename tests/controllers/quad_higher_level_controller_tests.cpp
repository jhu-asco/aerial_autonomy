#include "aerial_autonomy/controllers/quad_higher_level_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include <aerial_autonomy/tests/test_utils.h>
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <tf/transform_datatypes.h>
#include <thread>

using namespace quad_simulator;

TEST(QuadHigherLevelControllerTests, Constructor) {
  ASSERT_NO_THROW(QuadHigherLevelController());
}

TEST(QuadHigherLevelControllerTests, Failure) {
  AcadoConfig config;
  QuadHigherLevelController controller(config);

  // Obstacle at goal
  QuadFlatOutput initial(PositionYaw(0.0, 0.0, 0.0, 0.0));
  Position goal(1.0, 1.0, 1.0);
  QuadFlatOutput fsgoal(PositionYaw(goal, 0.0));

  Obstacle obs(goal, 0.5);
  std::vector<Obstacle> obstacle_list;
  obstacle_list.push_back(obs);
  std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data;
  sensor_data = make_tuple(initial, obstacle_list);

  controller.setGoal(fsgoal);
  Trajectory<QuadFlatOutput> output_traj(0.1);
  bool result = controller.run(sensor_data, output_traj);
  ASSERT_FALSE(result);
}

TEST(QuadHigherLevelControllerTests, ControllerRun) {
  AcadoConfig config;
  QuadHigherLevelController controller(config);

  QuadFlatOutput goal(PositionYaw(3.0, 0.0, 3.0, 0.0));

  Obstacle obs(1.5, 0.01, 1.5, 0.5);
  QuadFlatOutput initial(PositionYaw(0.0, 0.0, 0.0, 0.0));

  std::vector<Obstacle> obstacle_list;
  obstacle_list.push_back(obs);
  std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data;
  sensor_data = make_tuple(initial, obstacle_list);

  Trajectory<QuadFlatOutput> state_traj(0.1);

  controller.setGoal(goal);
  bool result = controller.run(sensor_data, state_traj);
  ASSERT_TRUE(result);

  ControllerStatus check = controller.isConverged(sensor_data);
  ASSERT_TRUE(bool(check));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}