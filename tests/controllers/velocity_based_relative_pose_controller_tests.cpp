#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <gtest/gtest.h>

class VelocityBasedRelativePoseControllerTests : public ::testing::Test {
public:
  VelocityBasedRelativePoseControllerTests() : position_gain_(0.1) {
    auto vel_controller_config =
        config_.mutable_velocity_based_position_controller_config();
    vel_controller_config->set_position_gain(position_gain_);
    auto tolerance = vel_controller_config->mutable_position_controller_config()
                         ->mutable_goal_position_tolerance();
    tolerance->set_x(0.1);
    tolerance->set_y(0.1);
    tolerance->set_z(0.1);
  }

protected:
  double position_gain_;
  VelocityBasedRelativePoseControllerConfig config_;
};

TEST_F(VelocityBasedRelativePoseControllerTests, Constructor) {
  ASSERT_NO_THROW(new VelocityBasedRelativePoseController(config_));
}

TEST_F(VelocityBasedRelativePoseControllerTests, ConvergedNoOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform goal(tf::Quaternion(), tf::Vector3(0, 0, 0));
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  VelocityYawRate controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_NEAR(controls.x, 0, 1e-6);
  ASSERT_NEAR(controls.y, 0, 1e-6);
  ASSERT_NEAR(controls.z, 0, 1e-6);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-6);
  ASSERT_TRUE(controller.isConverged(sensor_data));
}

TEST_F(VelocityBasedRelativePoseControllerTests, ConvergedOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform goal(tf::Quaternion(), tf::Vector3(-1, 0, 0));
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  VelocityYawRate controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_NEAR(controls.x, 0, 1e-6);
  ASSERT_NEAR(controls.y, 0, 1e-6);
  ASSERT_NEAR(controls.z, 0, 1e-6);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-6);
  ASSERT_TRUE(controller.isConverged(sensor_data));
}

TEST_F(VelocityBasedRelativePoseControllerTests, NotConvergedNoOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform goal(tf::Quaternion(), tf::Vector3(0, 0, 0));
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  VelocityYawRate controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_NEAR(controls.x, 1 * position_gain_, 1e-6);
  ASSERT_NEAR(controls.y, -1 * position_gain_, 1e-6);
  ASSERT_NEAR(controls.z, -2 * position_gain_, 1e-6);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-6);
  ASSERT_FALSE(controller.isConverged(sensor_data));
}

TEST_F(VelocityBasedRelativePoseControllerTests, NotConvergedOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform goal(tf::Quaternion(), tf::Vector3(-1, 3, 4));
  auto sensor_data = std::make_tuple(current_pose, tracked_pose);

  controller.setGoal(goal);

  VelocityYawRate controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_TRUE(result);
  ASSERT_NEAR(controls.x, 0 * position_gain_, 1e-6);
  ASSERT_NEAR(controls.y, 2 * position_gain_, 1e-6);
  ASSERT_NEAR(controls.z, 2 * position_gain_, 1e-6);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-6);
  ASSERT_FALSE(controller.isConverged(sensor_data));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
