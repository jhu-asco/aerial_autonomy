#include "aerial_autonomy/controllers/rpyt_based_position_controller.h"
#include <aerial_autonomy/tests/test_utils.h>

#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

class RPYTBasedPositionControllerTests : public ::testing::Test {
public:
  RPYTBasedPositionControllerTests()
      : config_(RPYTBasedPositionControllerConfig()) {
    // Set tolerances
    auto velocity_tolerance =
        config_.mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    velocity_tolerance->set_vx(0.01);
    velocity_tolerance->set_vy(0.01);
    velocity_tolerance->set_vz(0.01);
    auto position_tolerance =
        config_.mutable_velocity_based_position_controller_config()
            ->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    position_tolerance->set_x(0.05);
    position_tolerance->set_y(0.05);
    position_tolerance->set_z(0.05);
  }

  void runUntilConvergence(PositionYaw goal) {
    std::chrono::duration<double> dt = std::chrono::milliseconds(20);

    RPYTBasedPositionController controller(config_, dt);
    auto sensor_data =
        std::make_tuple(VelocityYawRate(0, 0, 0, 0), PositionYaw(0, 0, 0, 0));
    auto &velocity_yaw_rate = std::get<0>(sensor_data);
    auto &position_yaw = std::get<1>(sensor_data);
    auto convergence = [&]() {
      RollPitchYawRateThrust controls;

      controller.run(sensor_data, controls);
      tf::Transform tf;
      tf.setOrigin(tf::Vector3(0, 0, 0));
      tf.setRotation(
          tf::createQuaternionFromRPY(controls.r, controls.p, controls.y));

      double ext_z_acc = 0.1;
      tf::Vector3 body_acc(
          0, 0,
          controls.t * config_.rpyt_based_velocity_controller_config().kt() +
              ext_z_acc);
      tf::Vector3 global_acc = tf * body_acc;

      velocity_yaw_rate.x = velocity_yaw_rate.x + global_acc[0] * dt.count();
      velocity_yaw_rate.y = velocity_yaw_rate.y + global_acc[1] * dt.count();
      velocity_yaw_rate.z =
          velocity_yaw_rate.z + (global_acc[2] - 9.81) * dt.count();
      velocity_yaw_rate.yaw_rate = controls.y;
      position_yaw.x = velocity_yaw_rate.x * dt.count();
      position_yaw.y = velocity_yaw_rate.y * dt.count();
      position_yaw.z = velocity_yaw_rate.z * dt.count();
      position_yaw.yaw = position_yaw.yaw + controls.y * dt.count();

      return bool(controller.isConverged(sensor_data));

    };

    ASSERT_TRUE(test_utils::waitUntilTrue()(
        convergence, std::chrono::seconds(1), std::chrono::milliseconds(0)));
  }

  RPYTBasedPositionControllerConfig config_;
};

TEST_F(RPYTBasedPositionControllerTests, SetGetGoal) {
  RPYTBasedPositionController controller(config_,
                                         std::chrono::milliseconds(20));

  PositionYaw goal(1, -2, 3, 0.1);
  controller.setGoal(goal);
  PositionYaw expected_goal = controller.getGoal();
  ASSERT_EQ(goal, expected_goal);
}

TEST_F(RPYTBasedPositionControllerTests, Converged) {
  RPYTBasedPositionController controller(config_,
                                         std::chrono::milliseconds(20));

  PositionYaw goal(0, 0, 0, 0);
  controller.setGoal(goal);

  ASSERT_TRUE((bool)controller.isConverged(
      std::make_tuple(VelocityYawRate(0, 0, 0, 0), PositionYaw(0, 0, 0, 0))));
}

TEST_F(RPYTBasedPositionControllerTests, NotConverged) {
  RPYTBasedPositionController controller(config_,
                                         std::chrono::milliseconds(20));

  PositionYaw goal(0, 0, 0, 0);
  controller.setGoal(goal);

  // Not converged for non-zero position
  ASSERT_FALSE((bool)controller.isConverged(
      std::make_tuple(VelocityYawRate(0, 0, 0, 0), PositionYaw(1, 0, 0, 0))));
  // Not converged for non-zero velocity
  ASSERT_FALSE((bool)controller.isConverged(
      std::make_tuple(VelocityYawRate(1, 0, 0, 0), PositionYaw(0, 0, 0, 0))));
}

TEST_F(RPYTBasedPositionControllerTests, RunUntilConvergenceAlreadyConverged) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0));
}

TEST_F(RPYTBasedPositionControllerTests, RunUntilConvergenceNoYaw) {
  runUntilConvergence(PositionYaw(2.0, 3.0, -1.0, 0));
}

TEST_F(RPYTBasedPositionControllerTests, RunUntilConvergenceYaw) {
  runUntilConvergence(PositionYaw(-1.0, -1.0, 1.0, M_PI / 2.));
}

TEST_F(RPYTBasedPositionControllerTests, RunUntilConvergenceNegYaw) {
  runUntilConvergence(PositionYaw(-1.0, -1.0, 1.0, -M_PI / 2.));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
