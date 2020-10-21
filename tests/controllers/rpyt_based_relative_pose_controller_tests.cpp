#include "aerial_autonomy/controllers/rpyt_based_relative_pose_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <gtest/gtest.h>

using namespace conversions;

class RPYTBasedRelativePoseControllerTests : public ::testing::Test {
public:
  RPYTBasedRelativePoseControllerTests() : position_gain_(0.1), yaw_gain_(0.1) {
    auto vel_position_controller_config =
        (config_.mutable_velocity_based_relative_pose_controller_config())
            ->mutable_velocity_based_position_controller_config();
    vel_position_controller_config->set_position_i_gain(0.0);
    vel_position_controller_config->set_yaw_i_gain(0.0);
    vel_position_controller_config->set_position_gain(position_gain_);
    vel_position_controller_config->set_yaw_gain(yaw_gain_);
    auto tolerance =
        vel_position_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    auto rpyt_controller_config =
        config_.mutable_rpyt_based_velocity_controller_config();
    auto rpyt_vel_controller_config =
        rpyt_controller_config->mutable_velocity_controller_config();
    rpyt_vel_controller_config->mutable_goal_velocity_tolerance()->set_vx(0.01);
    rpyt_vel_controller_config->mutable_goal_velocity_tolerance()->set_vy(0.01);
    rpyt_vel_controller_config->mutable_goal_velocity_tolerance()->set_vz(0.01);
    thrust_gain_ = rpyt_controller_config->kt();
    tolerance->set_x(0.1);
    tolerance->set_y(0.1);
    tolerance->set_z(0.1);
  }

  tf::Vector3 getDesiredVelocity(const tf::Transform &current_pose,
                                 const tf::Transform &tracked_pose,
                                 const PositionYaw &goal) {
    tf::Transform goal_tf;
    positionYawToTf(goal, goal_tf);
    tf::Transform global_goal = tracked_pose * goal_tf;
    tf::Vector3 position_diff =
        global_goal.getOrigin() - current_pose.getOrigin();
    tf::Vector3 desired_velocity = position_diff * position_gain_;
    return desired_velocity;
  }

  void checkControls(const tf::Transform &current_pose,
                     const tf::Transform &tracked_pose,
                     const VelocityYawRate &current_velocity_yawrate,
                     const PositionYaw &goal, bool should_converge) {
    RPYTBasedRelativePoseController controller(config_,
                                               std::chrono::milliseconds(20));
    tf::Transform goal_tf;
    positionYawToTf(goal, goal_tf);
    auto sensor_data =
        std::make_tuple(current_pose, tracked_pose, current_velocity_yawrate);

    tf::Transform global_goal = tracked_pose * goal_tf;
    tf::Vector3 position_diff =
        global_goal.getOrigin() - current_pose.getOrigin();
    double goal_roll, goal_pitch, goal_yaw; // roll/pitch unused
    global_goal.getBasis().getRPY(goal_roll, goal_pitch, goal_yaw);
    double cur_roll, cur_pitch, cur_yaw; // roll/pitch unused
    current_pose.getBasis().getRPY(cur_roll, cur_pitch, cur_yaw);

    controller.setGoal(goal);

    RollPitchYawRateThrust controls;
    bool result = controller.run(sensor_data, controls);
    // Get acceleration vector using rpyt commands
    tf::Transform rotation;
    transformRPYToTf(controls.r, controls.p, cur_yaw, rotation);
    tf::Vector3 current_acceleration =
        (rotation * tf::Vector3(0, 0, 1)) * (controls.t * thrust_gain_) +
        tf::Vector3(0, 0, -9.81);
    tf::Vector3 desired_velocity = position_diff * position_gain_;
    tf::Vector3 current_velocity(current_velocity_yawrate.x,
                                 current_velocity_yawrate.y,
                                 current_velocity_yawrate.z);
    tf::Vector3 desired_acceleration_direction =
        desired_velocity - current_velocity;

    ASSERT_TRUE(result);
    for (int i = 0; i < 3; ++i) {
      if (desired_acceleration_direction.length() > 1e-6 &&
          current_acceleration.length() > 1e-6) {
        checkVectors(desired_acceleration_direction, current_acceleration);
      }
    }
    ASSERT_NEAR(controls.y, math::angleWrap(goal_yaw - cur_yaw) * yaw_gain_,
                1e-6);
    if (should_converge) {
      ASSERT_TRUE((bool)controller.isConverged(sensor_data));
    } else {
      ASSERT_FALSE((bool)controller.isConverged(sensor_data));
    }
  }

  void checkVectors(tf::Vector3 vector1, tf::Vector3 vector2) {
    tf::Vector3 vector1_normalized = vector1.normalize();
    tf::Vector3 vector2_normalized = vector2.normalize();
    for (int i = 0; i < 3; ++i) {
      ASSERT_NEAR(vector1_normalized[i], vector2_normalized[i], 1e-6);
    }
  }

protected:
  double position_gain_;
  double yaw_gain_;
  double thrust_gain_;
  RPYTBasedRelativePoseControllerConfig config_;
};

TEST_F(RPYTBasedRelativePoseControllerTests, Constructor) {
  ASSERT_NO_THROW(new RPYTBasedRelativePoseController(
      config_, std::chrono::milliseconds(20)));
}

TEST_F(RPYTBasedRelativePoseControllerTests, ConvergedNoOffset) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(0, 0, 0, 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                true);
}

TEST_F(RPYTBasedRelativePoseControllerTests, ConvergedOffset) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(-1, 0, 0, 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                true);
}

TEST_F(RPYTBasedRelativePoseControllerTests,
       PositionConvergedVelocityNotConverged) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(1, 2, 3, 0);
  PositionYaw goal(-1, 0, 0, 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                false);
}

TEST_F(RPYTBasedRelativePoseControllerTests,
       PositionNotConvergedVelocityConverged) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  PositionYaw goal(-1, 0, 0, 0);
  tf::Vector3 desired_velocity =
      getDesiredVelocity(current_pose, tracked_pose, goal);
  VelocityYawRate current_velocity_yawrate(
      desired_velocity.x(), desired_velocity.y(), desired_velocity.z(), 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                false);
}

TEST_F(RPYTBasedRelativePoseControllerTests, NotConvergedNoOffset) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(0, 0, 0, 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                false);
}

TEST_F(RPYTBasedRelativePoseControllerTests, NotConvergedOffset) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(-1, 3, 4, 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                false);
}

TEST_F(RPYTBasedRelativePoseControllerTests, NotConvergedOffsetYaw) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  tf::Transform current_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1, 1, 2));
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, 0.1),
                             tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(-1, 3, 4, 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                false);
}

TEST_F(RPYTBasedRelativePoseControllerTests, ConvergedOffsetYaw) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  double tracked_yaw = 0.1;
  tf::Transform current_pose(
      tf::createQuaternionFromRPY(0, 0, tracked_yaw),
      tf::Vector3(-1 * cos(tracked_yaw), -1 * sin(tracked_yaw), 2));
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, tracked_yaw),
                             tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(-1, 0, 2, 0);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                true);
}

TEST_F(RPYTBasedRelativePoseControllerTests, ConvergedGoalOffsetYaw) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  double tracked_yaw = 0.1;
  tf::Transform current_pose(
      tf::createQuaternionFromRPY(0, 0, 0.2),
      tf::Vector3(-1 * cos(tracked_yaw) - 3 * sin(tracked_yaw),
                  -1 * sin(tracked_yaw) + 3 * cos(tracked_yaw), 2));
  tf::Transform tracked_pose(tf::createQuaternionFromRPY(0, 0, tracked_yaw),
                             tf::Vector3(0, 0, -2));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(-1, 3, 4, 0.1);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                true);
}

TEST_F(RPYTBasedRelativePoseControllerTests,
       ConvergedGoalOffsetYawNonzeroTrackedRollPitch) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  double tracked_yaw = 0.1;
  tf::Transform current_pose(tf::createQuaternionFromRPY(0, 0, 0.2),
                             tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(
      tf::createQuaternionFromRPY(0.1, -0.1, tracked_yaw),
      tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(0, 0, 0, 0.1);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                true);
}

TEST_F(RPYTBasedRelativePoseControllerTests,
       ConvergedGoalOffsetYawNonzeroTrackedAndCurrentRollPitch) {
  RPYTBasedRelativePoseController controller(config_,
                                             std::chrono::milliseconds(20));
  double tracked_yaw = 0.1;
  tf::Transform current_pose(tf::createQuaternionFromRPY(-0.3, 0.2, 0.2),
                             tf::Vector3(0, 0, 0));
  tf::Transform tracked_pose(
      tf::createQuaternionFromRPY(0.2, -0.1, tracked_yaw),
      tf::Vector3(0, 0, 0));
  VelocityYawRate current_velocity_yawrate(0, 0, 0, 0);
  PositionYaw goal(0, 0, 0, 0.1);
  checkControls(current_pose, tracked_pose, current_velocity_yawrate, goal,
                true);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
