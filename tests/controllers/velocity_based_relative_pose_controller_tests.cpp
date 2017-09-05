#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <gtest/gtest.h>

using namespace conversions;

class VelocityBasedRelativePoseControllerTests : public ::testing::Test {
public:
  VelocityBasedRelativePoseControllerTests()
      : position_gain_(0.1), yaw_gain_(0.1) {
    auto vel_controller_config =
        config_.mutable_velocity_based_position_controller_config();
    vel_controller_config->set_position_gain(position_gain_);
    vel_controller_config->set_yaw_gain(yaw_gain_);
    auto tolerance = vel_controller_config->mutable_position_controller_config()
                         ->mutable_goal_position_tolerance();
    tolerance->set_x(0.1);
    tolerance->set_y(0.1);
    tolerance->set_z(0.1);
  }

  void checkControls(const tf::Transform &current_pose,
                     const tf::Transform &tracked_pose, const PositionYaw &goal,
                     bool should_converge) {
    VelocityBasedRelativePoseController controller(config_);
    tf::Transform goal_tf;
    conversions::positionYawToTf(goal, goal_tf);
    auto sensor_data = std::make_tuple(current_pose, tracked_pose);

    tf::Transform global_goal = tracked_pose * goal_tf;
    tf::Vector3 position_diff =
        global_goal.getOrigin() - current_pose.getOrigin();
    double goal_roll, goal_pitch, goal_yaw; // roll/pitch unused
    global_goal.getBasis().getRPY(goal_roll, goal_pitch, goal_yaw);
    double cur_roll, cur_pitch, cur_yaw; // roll/pitch unused
    current_pose.getBasis().getRPY(cur_roll, cur_pitch, cur_yaw);

    controller.setGoal(goal);

    VelocityYawRate controls;
    bool result = controller.run(sensor_data, controls);

    ASSERT_TRUE(result);
    ASSERT_NEAR(controls.x, position_diff.x() * position_gain_, 1e-6);
    ASSERT_NEAR(controls.y, position_diff.y() * position_gain_, 1e-6);
    ASSERT_NEAR(controls.z, position_diff.z() * position_gain_, 1e-6);
    ASSERT_NEAR(controls.yaw_rate,
                math::angleWrap(goal_yaw - cur_yaw) * yaw_gain_, 1e-6);
    if (should_converge) {
      ASSERT_TRUE(controller.isConverged(sensor_data));
    } else {
      ASSERT_FALSE(controller.isConverged(sensor_data));
    }
  }

protected:
  double position_gain_;
  double yaw_gain_;
  VelocityBasedRelativePoseControllerConfig config_;
};

TEST_F(VelocityBasedRelativePoseControllerTests, Constructor) {
  ASSERT_NO_THROW(new VelocityBasedRelativePoseController(config_));
}

TEST_F(VelocityBasedRelativePoseControllerTests, ConvergedNoOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  PositionYaw goal(0, 0, 0, 0);
  checkControls(current_pose, tracked_pose, goal, true);
}

TEST_F(VelocityBasedRelativePoseControllerTests, ConvergedOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  PositionYaw goal(-1, 0, 0, 0);
  checkControls(current_pose, tracked_pose, goal, true);
}

TEST_F(VelocityBasedRelativePoseControllerTests, NotConvergedNoOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  PositionYaw goal(0, 0, 0, 0);
  checkControls(current_pose, tracked_pose, goal, false);
}

TEST_F(VelocityBasedRelativePoseControllerTests, NotConvergedOffset) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  PositionYaw goal(-1, 3, 4, 0);
  checkControls(current_pose, tracked_pose, goal, false);
}

TEST_F(VelocityBasedRelativePoseControllerTests, NotConvergedOffsetYaw) {
  VelocityBasedRelativePoseController controller(config_);
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, 0.1),
                             tf::Vector3(0, 0, 0));
  PositionYaw goal(-1, 3, 4, 0);
  checkControls(current_pose, tracked_pose, goal, false);
}

TEST_F(VelocityBasedRelativePoseControllerTests, ConvergedOffsetYaw) {
  VelocityBasedRelativePoseController controller(config_);
  double tracked_yaw = 0.1;
  tf::Transform current_pose(
      tf::createQuaternionFromRPY(0, 0, tracked_yaw),
      tf::Vector3(-1 * cos(tracked_yaw), -1 * sin(tracked_yaw), 2));
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, tracked_yaw),
                             tf::Vector3(0, 0, 0));
  PositionYaw goal(-1, 0, 2, 0);
  checkControls(current_pose, tracked_pose, goal, true);
}

TEST_F(VelocityBasedRelativePoseControllerTests, ConvergedGoalOffsetYaw) {
  VelocityBasedRelativePoseController controller(config_);
  double tracked_yaw = 0.1;
  tf::Transform current_pose(
      tf::createQuaternionFromRPY(0, 0, 0.2),
      tf::Vector3(-1 * cos(tracked_yaw) - 3 * sin(tracked_yaw),
                  -1 * sin(tracked_yaw) + 3 * cos(tracked_yaw), 2));
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, tracked_yaw),
                             tf::Vector3(0, 0, -2));
  PositionYaw goal(-1, 3, 4, 0.1);
  checkControls(current_pose, tracked_pose, goal, true);
}

TEST_F(VelocityBasedRelativePoseControllerTests,
       ConvergedGoalOffsetYawNonzeroTrackedRollPitch) {
  VelocityBasedRelativePoseController controller(config_);
  double tracked_yaw = 0.1;
  tf::Transform current_pose(tf::createQuaternionFromRPY(0, 0, 0.2),
                             tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(
      tf::createQuaternionFromRPY(0.1, -0.1, tracked_yaw),
      tf::Vector3(0, 0, 0));
  PositionYaw goal(0, 0, 0, 0.1);
  checkControls(current_pose, tracked_pose, goal, true);
}

TEST_F(VelocityBasedRelativePoseControllerTests,
       ConvergedGoalOffsetYawNonzeroTrackedAndCurrentRollPitch) {
  VelocityBasedRelativePoseController controller(config_);
  double tracked_yaw = 0.1;
  tf::Transform current_pose(tf::createQuaternionFromRPY(-0.3, 0.2, 0.2),
                             tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(
      tf::createQuaternionFromRPY(0.2, -0.1, tracked_yaw),
      tf::Vector3(0, 0, 0));
  PositionYaw goal(0, 0, 0, 0.1);
  checkControls(current_pose, tracked_pose, goal, true);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
