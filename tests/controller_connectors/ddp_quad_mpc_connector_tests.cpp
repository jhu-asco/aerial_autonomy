#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/controller_connectors/mpc_controller_quad_connector.h"
#include "aerial_autonomy/controllers/ddp_quad_mpc_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/waypoint.h"

#include <Eigen/Dense>

#include <quad_simulator_parser/quad_simulator.h>

#include <gtest/gtest.h>

/**
* @brief MPC Controller testing fixture.
*
* Provides functionality to create MPC Controller
*
*/
class MPCControllerQuadConnectorTests : public ::testing::Test {
public:
  /**
  * @brief Constructor
  */
  MPCControllerQuadConnectorTests()
      : thrust_gain_estimator_(0.2),
        controller_config_(test_utils::createQuadMPCConfig()) {
    controller_.reset(new DDPQuadMPCController(controller_config_,
                                               std::chrono::milliseconds(20)));
    controller_connector_.reset(new MPCControllerQuadConnector(
        drone_hardware_, *controller_, thrust_gain_estimator_));
    drone_hardware_.usePerfectTime();
    drone_hardware_.set_delay_send_time(0.02);
    controller_connector_->usePerfectTimeDiff(0.02);
  }

  /**
  * @brief Configure logger
  */
  static void SetUpTestCase() {
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("ddp_quad_mpc_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("quad_mpc_state_estimator");
    Log::instance().addDataStream(data_config);
  }

  /**
  * @brief run the MPC controller until its either converged/crashed
  *
  * @param initial_state initial robot state
  * @param goal Goal point for robot
  * @param check_thrust_gain Flag whether to verify convergence of thrust gain
  * estimator
  */
  void runUntilConvergence(const PositionYaw &initial_state,
                           const PositionYaw &goal,
                           bool check_thrust_gain = true) {
    // Fly quadrotor which sets the altitude to 0.5
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_state.x;
    init_position.y = initial_state.y;
    init_position.z = initial_state.z;
    drone_hardware_.cmdwaypoint(init_position, initial_state.yaw);
    controller_connector_->setGoal(conversions::createWayPoint(goal));
    auto runController = [&]() {
      controller_connector_->run();
      return controller_connector_->getStatus() == ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(
        runController, std::chrono::seconds(20), std::chrono::milliseconds(0)));
    // Check position is close to goal position
    parsernode::common::quaddata sensor_data;
    drone_hardware_.getquaddata(sensor_data);
    tf::Transform quad_transform(
        tf::createQuaternionFromRPY(0, 0, sensor_data.rpydata.z),
        tf::Vector3(sensor_data.localpos.x, sensor_data.localpos.y,
                    sensor_data.localpos.z));
    tf::Transform goal_transform(tf::createQuaternionFromRPY(0, 0, goal.yaw),
                                 tf::Vector3(goal.x, goal.y, goal.z));
    test_utils::ASSERT_VEC_NEAR(quad_transform.getOrigin(),
                                goal_transform.getOrigin(),
                                controller_config_.goal_position_tolerance());
    tf::Vector3 zero_vec(0, 0, 0);
    test_utils::ASSERT_VEC_NEAR(
        tf::Vector3(sensor_data.linvel.x, sensor_data.linvel.y,
                    sensor_data.linvel.z),
        zero_vec, controller_config_.goal_velocity_tolerance());
    ControllerStatus status = controller_connector_->getStatus();
    ASSERT_EQ(status, ControllerStatus::Completed) << status.statusAsText();
    if (check_thrust_gain) {
      ASSERT_NEAR(thrust_gain_estimator_.getThrustGain(), 0.16, 1e-2);
    }
  }

protected:
  quad_simulator::QuadSimulator drone_hardware_;     ///< Quad simulator
  std::unique_ptr<DDPQuadMPCController> controller_; ///< MPC controller
  std::unique_ptr<MPCControllerQuadConnector>
      controller_connector_;                  ///< MPC connector
  ThrustGainEstimator thrust_gain_estimator_; ///< Thrust gain estimator
  QuadMPCControllerConfig controller_config_; ///< Config for MPC controller
};

TEST_F(MPCControllerQuadConnectorTests, GoalIsStart) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(0, 0, 0, 0), false);
}

TEST_F(MPCControllerQuadConnectorTests, ConvergenceZeroStartNoYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(1, -1, 1, 0));
}

TEST_F(MPCControllerQuadConnectorTests, ConvergenceZeroStartPosYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(-0.1, -1, 0.1, 0.1));
}

TEST_F(MPCControllerQuadConnectorTests, ConvergenceZeroStartNegYaw) {
  runUntilConvergence(PositionYaw(0, 0, 0, 0), PositionYaw(1, -1, -1, -0.5));
}

TEST_F(MPCControllerQuadConnectorTests, ConvergenceNonZeroStartNoYaw) {
  runUntilConvergence(PositionYaw(-1, 0.3, 2, 0), PositionYaw(-0.5, 0.5, 1, 0));
}

TEST_F(MPCControllerQuadConnectorTests, ConvergenceNonZeroStartPosYaw) {
  runUntilConvergence(PositionYaw(-2.2, -1, 3.2, -0.1),
                      PositionYaw(-2, -1.5, 3.5, 0.5));
}

TEST_F(MPCControllerQuadConnectorTests, ConvergenceNonZeroStartNegYaw) {
  runUntilConvergence(PositionYaw(-.1, 0.2, 1, 0.4),
                      PositionYaw(-0.5, -0.2, -0.1, -0.5));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
