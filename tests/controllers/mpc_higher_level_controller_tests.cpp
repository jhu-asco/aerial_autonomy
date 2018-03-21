#include "aerial_autonomy/controllers/mpc_higher_level_controller.h"
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>

TEST(MPCHigherLevelControllerTests, Constructor) {
  ASSERT_NO_THROW(new MPCHigherLevelController());
}

TEST(MPCHigherLevelControllerTests, CheckOCP) {
  MPCHigherLevelControllerConfig config;
  MPCHigherLevelController mpc_controller(config);

  const ACADO::OCP &ocp = mpc_controller.getOCP();

  ASSERT_EQ(ocp.getStartTime(), 0.0);
  ASSERT_EQ(ocp.getEndTime(), config.te());

  ASSERT_TRUE(ocp.hasObjective());

  ASSERT_TRUE(ocp.hasConstraint());
}

TEST(MPCHigherLevelControllerTests, Failure) {
  MPCHigherLevelControllerConfig config;
  MPCHigherLevelController mpc_controller(config);

  Position goal(1.0, 1.0, 1.0);
  mpc_controller.setGoal(goal);
  Position exp_goal = mpc_controller.getGoal();
  ASSERT_EQ(goal, exp_goal);

  // Obstacle at goal
  Obstacle obs(goal, 0.5);
  PositionYaw initial(0.0, 0.0, 0.0, 0.0);
  std::vector<Obstacle> obstacle_list;
  obstacle_list.push_back(obs);
  std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data;
  sensor_data = make_tuple(initial, obstacle_list);

  Trajectory<QuadFlatOutput> dummy_state_traj(0.1);
  Trajectory<QuadFlatSpaceControls> dummy_control_traj(0.1);

  std::tuple<Trajectory<QuadFlatOutput>, Trajectory<QuadFlatSpaceControls>>
      control;
  control = std::make_tuple(dummy_state_traj, dummy_control_traj);
  bool result = mpc_controller.run(sensor_data, control);

  ASSERT_FALSE(result);
}

TEST(MPCHigherLevelControllerTests, ControllerRun) {
  MPCHigherLevelControllerConfig config;
  MPCHigherLevelController mpc_controller(config);

  Position goal(3.0, 0.0, 3.0);
  mpc_controller.setGoal(goal);
  Position exp_goal = mpc_controller.getGoal();
  ASSERT_EQ(goal, exp_goal);

  Obstacle obs(1.5, 0.01, 1.5, 0.5);
  PositionYaw initial(0.0, 0.0, 0.0, 0.0);
  std::vector<Obstacle> obstacle_list;
  obstacle_list.push_back(obs);
  std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data;
  sensor_data = make_tuple(initial, obstacle_list);

  Trajectory<QuadFlatOutput> state_traj(0.1);
  Trajectory<QuadFlatSpaceControls> control_traj(0.1);

  std::tuple<Trajectory<QuadFlatOutput>, Trajectory<QuadFlatSpaceControls>>
      control;
  control = std::make_tuple(state_traj, control_traj);
  bool result = mpc_controller.run(sensor_data, control);

  ASSERT_TRUE(result);
  ASSERT_TRUE(mpc_controller.checkTrajectoryFeasibility(sensor_data, control));
  ASSERT_TRUE(bool(mpc_controller.isConverged(sensor_data)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}