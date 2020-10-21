#include "aerial_autonomy/controllers/relative_pose_controller.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <gtest/gtest.h>

using namespace test_utils;

class RelativePoseControllerTests : public ::testing::Test {
public:
  RelativePoseControllerTests() {
    auto tolerance = config_.mutable_goal_position_tolerance();
    tolerance->set_x(0.1);
    tolerance->set_y(0.2);
    tolerance->set_z(0.1);
  }

protected:
  PoseControllerConfig config_;
};

TEST_F(RelativePoseControllerTests, Constructor) {
  ASSERT_NO_THROW(new RelativePoseController(config_));
}

TEST_F(RelativePoseControllerTests, ConvergedNoOffset) {
  RelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform goal(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform global_goal = tracked_pose * goal;
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  tf::Transform controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_TF_NEAR(controls, global_goal);
  ASSERT_TRUE((bool)controller.isConverged(sensor_data));
}

TEST_F(RelativePoseControllerTests, NotConvergedNoOffset) {
  RelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1),
                             tf::Vector3(-1, -1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform goal(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform global_goal = tracked_pose * goal;
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  tf::Transform controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_TF_NEAR(controls, global_goal);
  ASSERT_FALSE((bool)controller.isConverged(sensor_data));
}

TEST_F(RelativePoseControllerTests, ConvergedOffset) {
  RelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(9, 0, -1));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(10, 0, 0));
  tf::Transform goal(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, -1));
  tf::Transform global_goal = tracked_pose * goal;
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  tf::Transform controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_TF_NEAR(controls, global_goal);
  ASSERT_TRUE((bool)controller.isConverged(sensor_data));
}

TEST_F(RelativePoseControllerTests, ConvergedWithinTolerance) {
  RelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(9, 0, -1));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1),
                             tf::Vector3(10.09, .19, 0.09));
  tf::Transform goal(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, -1));
  tf::Transform global_goal = tracked_pose * goal;
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  tf::Transform controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_TF_NEAR(controls, global_goal);
  ASSERT_TRUE((bool)controller.isConverged(sensor_data));
}

TEST_F(RelativePoseControllerTests, NotConvergedOffset) {
  RelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(10, 0, 0));
  tf::Transform goal(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, -1));
  tf::Transform global_goal = tracked_pose * goal;
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  tf::Transform controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_TF_NEAR(controls, global_goal);
  ASSERT_FALSE((bool)controller.isConverged(sensor_data));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
