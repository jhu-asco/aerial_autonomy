#pragma once
#include "aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h"
#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/waypoint.h"

#include <Eigen/Dense>

#include "arm_parsers/arm_simulator.h"
#include <quad_simulator_parser/quad_simulator.h>

#include <gtest/gtest.h>

/**
* @brief MPC Controller testing fixture.
*
* Provides functionality to create MPC Controller
*
*/
class MPCControllerAirmConnectorTests : public ::testing::Test {
public:
  /**
  * @brief Constructor
  */
  MPCControllerAirmConnectorTests()
      : thrust_gain_estimator_(0.2),
        controller_config_(test_utils::createMPCConfig()) {
    controller_.reset(new DDPAirmMPCController(controller_config_,
                                               std::chrono::milliseconds(20)));
    controller_connector_.reset(new MPCControllerAirmConnector(
        drone_hardware_, arm_simulator_, *controller_, thrust_gain_estimator_));
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
  }

  /**
  * @brief run the MPC controller until its either converged/crashed
  *
  * @param initial_state initial robot state
  * @param goal Goal point for robot
  * @param goal_joint_angles Initial joint angles
  * @param init_joint_angles Goal joint angles
  * @param check_thrust_gain Flag whether to verify convergence of thrust gain
  * estimator
  */
  void runUntilConvergence(const PositionYaw &initial_state,
                           const PositionYaw &goal,
                           std::vector<double> &&goal_joint_angles,
                           std::vector<double> &&init_joint_angles = {0, 0},
                           bool check_thrust_gain = true) {
    // Fly quadrotor which sets the altitude to 0.5
    drone_hardware_.setBatteryPercent(60);
    drone_hardware_.takeoff();
    geometry_msgs::Vector3 init_position;
    init_position.x = initial_state.x;
    init_position.y = initial_state.y;
    init_position.z = initial_state.z;
    drone_hardware_.cmdwaypoint(init_position, initial_state.yaw);
    arm_simulator_.setJointAngles(init_joint_angles);
    controller_connector_->setGoal(
        createWayPoint(goal, goal_joint_angles[0], goal_joint_angles[1]));
    auto runController = [&]() {
      controller_connector_->run();
      return controller_connector_->getStatus() == ControllerStatus::Active;
    };
    ASSERT_FALSE(test_utils::waitUntilFalse()(
        runController, std::chrono::seconds(5), std::chrono::milliseconds(0)));
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

  /**
  * @brief create a waypoint reference trajectory from a goal point and joint
  * angles
  *
  * @param goal Goal position and yaw
  * @param desired_joint_angle_1 Desired joint angle for first joint
  * @param desired_joint_angle_2 Desired joint angle for second joint
  *
  * @return Waypoint Reference trajectory
  */
  std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>
  createWayPoint(PositionYaw goal, double desired_joint_angle_1,
                 double desired_joint_angle_2) {
    std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>> waypoint;
    Eigen::VectorXd goal_control(6);
    goal_control << 1, 0, 0, 0, desired_joint_angle_1, desired_joint_angle_2;
    Eigen::VectorXd goal_state(21);
    goal_state << goal.x, goal.y, goal.z, 0, 0, goal.yaw, 0, 0, 0, 0, 0, 0, 0,
        0, goal.yaw, desired_joint_angle_1, desired_joint_angle_2, 0, 0,
        desired_joint_angle_1, desired_joint_angle_2;
    waypoint.reset(new Waypoint<Eigen::VectorXd, Eigen::VectorXd>(
        goal_state, goal_control));
    return waypoint;
  }

protected:
  quad_simulator::QuadSimulator drone_hardware_;     ///< Quad simulator
  ArmSimulator arm_simulator_;                       ///< Arm simulator
  std::unique_ptr<DDPAirmMPCController> controller_; ///< MPC controller
  std::unique_ptr<MPCControllerAirmConnector>
      controller_connector_;                  ///< MPC connector
  ThrustGainEstimator thrust_gain_estimator_; ///< Thrust gain estimator
  AirmMPCControllerConfig controller_config_; ///< Config for MPC controller
};
