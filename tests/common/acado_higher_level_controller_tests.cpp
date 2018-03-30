#include "aerial_autonomy/common/acado_higher_level_controller.h"
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>

TEST(AcadoHigherLevelControllerTests, Constructor) {
  ASSERT_NO_THROW(AcadoHigherLevelController());
}

TEST(AcadoHigherLevelControllerTests, Failure) {
  AcadoConfig config;
  AcadoHigherLevelController controller(config);

  // Obstacle at goal
  QuadFlatOutput initial(PositionYaw(0.0, 0.0, 0.0, 0.0));
  Position goal(1.0, 1.0, 1.0);
  QuadFlatOutput fsgoal(PositionYaw(goal, 0.0));
  Obstacle obs(goal, 0.5);
  std::vector<Obstacle> obstacle_list;
  obstacle_list.push_back(obs);
  std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data;
  sensor_data = make_tuple(initial, obstacle_list);

  Trajectory<QuadFlatOutput> output_traj(0.1);
  bool result = controller.solve(sensor_data, fsgoal, output_traj);
  ASSERT_FALSE(result);
}

TEST(AcadoHigherLevelControllerTests, Solve) {
  AcadoConfig config;
  AcadoHigherLevelController mpc_controller(config);

  QuadFlatOutput goal(PositionYaw(3.0, 0.0, 3.0, 0.0));

  Obstacle obs(1.5, 0.01, 1.5, 0.5);
  QuadFlatOutput initial(PositionYaw(0.0, 0.0, 0.0, 0.0));
  std::vector<Obstacle> obstacle_list;
  obstacle_list.push_back(obs);
  std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data;
  sensor_data = make_tuple(initial, obstacle_list);

  Trajectory<QuadFlatOutput> state_traj(0.1);
  bool result = mpc_controller.solve(sensor_data, goal, state_traj);

  ASSERT_TRUE(result);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}