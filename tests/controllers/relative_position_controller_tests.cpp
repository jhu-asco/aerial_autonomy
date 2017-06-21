#include "aerial_autonomy/controllers/relative_position_controller.h"

#include <gtest/gtest.h>

TEST(RelativePositionControllerTests, Constructor) {
  PositionControllerConfig config;
  ASSERT_NO_THROW(new RelativePositionController(config));
}

TEST(RelativePositionControllerTests, ConvergedNoOffset) {
  PositionControllerConfig config;
  RelativePositionController controller(config);
  Position current_position(0, 0, 0);
  Position tracked_position(0, 0, 0);
  Position goal(0, 0, 0);
  Position global_goal = tracked_position + goal;
  auto sensor_data = std::make_tuple(current_position, tracked_position);

  controller.setGoal(goal);

  Position controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.x, global_goal.x, 1e-8);
  ASSERT_NEAR(controls.y, global_goal.y, 1e-8);
  ASSERT_NEAR(controls.z, global_goal.z, 1e-8);
  ASSERT_TRUE(result);
  ASSERT_TRUE(controller.isConverged(sensor_data));
}

TEST(RelativePositionControllerTests, NotConvergedNoOffset) {
  PositionControllerConfig config;
  RelativePositionController controller(config);
  Position current_position(-1, -1, 2);
  Position tracked_position(0, 0, 0);
  Position goal(0, 0, 0);
  Position global_goal = tracked_position + goal;
  auto sensor_data = std::make_tuple(current_position, tracked_position);

  controller.setGoal(goal);

  Position controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.x, global_goal.x, 1e-8);
  ASSERT_NEAR(controls.y, global_goal.y, 1e-8);
  ASSERT_NEAR(controls.z, global_goal.z, 1e-8);
  ASSERT_TRUE(result);
  ASSERT_FALSE(controller.isConverged(sensor_data));
}

TEST(RelativePositionControllerTests, ConvergedOffset) {
  PositionControllerConfig config;
  RelativePositionController controller(config);
  Position current_position(9, 0, -1);
  Position tracked_position(10, 0, 0);
  Position goal(-1, 0, -1);
  Position global_goal = tracked_position + goal;
  auto sensor_data = std::make_tuple(current_position, tracked_position);

  controller.setGoal(goal);

  Position controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.x, global_goal.x, 1e-8);
  ASSERT_NEAR(controls.y, global_goal.y, 1e-8);
  ASSERT_NEAR(controls.z, global_goal.z, 1e-8);
  ASSERT_TRUE(result);
  ASSERT_TRUE(controller.isConverged(sensor_data));
}

TEST(RelativePositionControllerTests, NotConvergedOffset) {
  PositionControllerConfig config;
  RelativePositionController controller(config);
  Position current_position(0, 1, 2);
  Position tracked_position(10, 0, 0);
  Position goal(-1, 0, -1);
  Position global_goal = tracked_position + goal;
  auto sensor_data = std::make_tuple(current_position, tracked_position);

  controller.setGoal(goal);

  Position controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.x, global_goal.x, 1e-8);
  ASSERT_NEAR(controls.y, global_goal.y, 1e-8);
  ASSERT_NEAR(controls.z, global_goal.z, 1e-8);
  ASSERT_TRUE(result);
  ASSERT_FALSE(controller.isConverged(sensor_data));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
