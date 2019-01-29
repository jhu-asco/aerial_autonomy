#include "aerial_autonomy/controllers/rpyt_based_reference_controller.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/spiral_reference_trajectory.h"

#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

class RPYTBasedReferenceControllerEigenTests : public ::testing::Test {
public:
  RPYTBasedReferenceControllerEigenTests()
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

  void runUntilConvergence(
      ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> goal) {
    RPYTBasedReferenceControllerEigen controller(config_);
    controller.setGoal(goal);
    double dt = 0.02;
    auto sensor_data =
        std::make_tuple(0.0, 0.16, Velocity(0, 0, 0), PositionYaw(0, 0, 0, 0));
    double &time = std::get<0>(sensor_data);
    double kt = std::get<1>(sensor_data);
    auto &velocity = std::get<2>(sensor_data);
    auto &position_yaw = std::get<3>(sensor_data);
    auto convergence = [&]() {
      RollPitchYawRateThrust controls;
      controller.run(sensor_data, controls);
      tf::Transform tf;
      tf.setOrigin(tf::Vector3(0, 0, 0));
      tf.setRotation(
          tf::createQuaternionFromRPY(controls.r, controls.p, controls.y));
      // std::cout<<"Controls: "<<controls.r<<", "<<controls.p<<",
      // "<<controls.y<<", "<<controls.t<<std::endl;
      tf::Vector3 body_acc(0, 0, controls.t * kt);
      tf::Vector3 global_acc = tf * body_acc;

      velocity.x += global_acc[0] * dt;
      velocity.y += global_acc[1] * dt;
      velocity.z += (global_acc[2] - 9.81) * dt;
      position_yaw.x += velocity.x * dt;
      position_yaw.y += velocity.y * dt;
      position_yaw.z += velocity.z * dt;
      position_yaw.yaw = position_yaw.yaw + controls.y * dt;
      time = time + dt;

      return bool(controller.isConverged(sensor_data));

    };

    ASSERT_TRUE(test_utils::waitUntilTrue()(
        convergence, std::chrono::seconds(1), std::chrono::milliseconds(0)));
  }

  void runUntilConvergence(PositionYaw goal) {
    runUntilConvergence(conversions::createWaypoint(goal));
  }
  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_log_rate(1000);
    data_config.set_stream_id("rpyt_reference_controller");
    Log::instance().addDataStream(data_config);
  }

  RPYTBasedPositionControllerConfig config_;
};

TEST_F(RPYTBasedReferenceControllerEigenTests, Converged) {
  RPYTBasedReferenceControllerEigen controller(config_);

  PositionYaw goal(0, 0, 0, 0);
  controller.setGoal(conversions::createWaypoint(goal));

  ASSERT_TRUE(controller.isConverged(
      std::make_tuple(1.0, 0.16, Velocity(0, 0, 0), PositionYaw(0, 0, 0, 0))));
}

TEST_F(RPYTBasedReferenceControllerEigenTests, NotConverged) {
  RPYTBasedReferenceControllerEigen controller(config_);

  PositionYaw goal(0, 0, 0, 0);
  controller.setGoal(conversions::createWaypoint(goal));

  // Not converged for non-zero position
  ASSERT_FALSE(controller.isConverged(std::make_tuple(
      0.5, 0.16, VelocityYawRate(0, 0, 0, 0), PositionYaw(1, 0, 0, 0))));
  // Not converged for non-zero velocity
  ASSERT_FALSE(controller.isConverged(std::make_tuple(
      0.5, 0.16, VelocityYawRate(1, 0, 0, 0), PositionYaw(0, 0, 0, 0))));
}

TEST_F(RPYTBasedReferenceControllerEigenTests,
       RunUntilConvergenceAlreadyConverged) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0));
}

TEST_F(RPYTBasedReferenceControllerEigenTests, RunUntilConvergenceNoYaw) {
  runUntilConvergence(PositionYaw(2.0, 3.0, -1.0, 0));
}

TEST_F(RPYTBasedReferenceControllerEigenTests, RunUntilConvergenceYaw) {
  runUntilConvergence(PositionYaw(-1.0, -1.0, 1.0, M_PI / 2.));
}

TEST_F(RPYTBasedReferenceControllerEigenTests, RunUntilConvergenceNegYaw) {
  runUntilConvergence(PositionYaw(-1.0, -1.0, 1.0, -M_PI / 2.));
}

TEST_F(RPYTBasedReferenceControllerEigenTests, RunUntilConvergenceSpiral) {
  SpiralReferenceTrajectoryConfig spiral_config;
  ArmSineControllerConfig arm_reference_config;
  for (int i = 0; i < 2; ++i) {
    arm_reference_config.add_joint_config();
  }
  std::shared_ptr<SpiralReferenceTrajectory> reference_trajectory(
      new SpiralReferenceTrajectory(spiral_config, arm_reference_config,
                                    Eigen::Vector3d(0, 0, 0), 0));
  runUntilConvergence(reference_trajectory);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
