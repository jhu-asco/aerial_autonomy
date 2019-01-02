#include "aerial_autonomy/controllers/ddp_quad_mpc_controller.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/quad_particle_reference_trajectory.h"
#include "aerial_autonomy/types/spiral_reference_trajectory.h"
#include "aerial_autonomy/types/waypoint.h"
#include "ddp_mpc_controller_config.pb.h"

#include <gtest/gtest.h>

class DDPQuadMPCControllerTests : public ::testing::Test {
public:
  using QuadGcopWaypoint = Waypoint<Eigen::VectorXd, Eigen::VectorXd>;

  DDPQuadMPCControllerTests() {
    // DDP Related
    auto lb = config_.mutable_lower_bound_control();
    lb->Resize(4, 0);
    auto ub = config_.mutable_upper_bound_control();
    ub->Resize(4, 0);
    config_.set_lower_bound_control(0, 0.8);
    config_.set_lower_bound_control(1, -0.3);
    config_.set_lower_bound_control(2, -0.3);
    config_.set_lower_bound_control(3, -0.3);

    config_.set_upper_bound_control(0, 1.2);
    config_.set_upper_bound_control(1, 0.3);
    config_.set_upper_bound_control(2, 0.3);
    config_.set_upper_bound_control(3, 0.3);
    DDPMPCControllerConfig *ddp_config = config_.mutable_ddp_config();
    ddp_config->set_look_ahead_time(0.02);
    auto q = ddp_config->mutable_q();
    q->Resize(15, 0.0);
    auto qf = ddp_config->mutable_qf();
    qf->Resize(15, 0.1);
    auto r = ddp_config->mutable_r();
    r->Resize(4, 0.1);
    r->Set(0, 10.0); // Reduce thrust control
    for (int i = 0; i < 3; ++i) {
      // Pos cost
      ddp_config->set_qf(i, 100.0);
      // Rpy cost
      ddp_config->set_qf(i + 3, 50.0);
      // Vel cost
      ddp_config->set_qf(i + 6, 10.0);
      ddp_config->set_q(i + 6, 1.0);
      // Rpydot cost
      ddp_config->set_qf(i + 9, 5.0);
    }
    ddp_config->set_debug(false);
    ddp_config->set_max_iters(100);
  }

  std::shared_ptr<DDPQuadMPCController>
  createController(double controller_duration = 0.02) {
    std::shared_ptr<DDPQuadMPCController> controller(new DDPQuadMPCController(
        config_, std::chrono::duration<double>(controller_duration)));
    return controller;
  }

  void checkState(Eigen::VectorXd &state, Eigen::VectorXd &goal_state) {
    ASSERT_EQ(state.rows(), 15);
    ASSERT_EQ(goal_state.rows(), 15);
    // Check positions are close enough
    for (int i = 0; i < 3; ++i) {
      ASSERT_NEAR(state[i], goal_state[i], 1e-1);         // pos
      ASSERT_NEAR(state[i + 3], goal_state[i + 3], 5e-2); // rpy
      ASSERT_NEAR(state[i + 6], goal_state[i + 6], 0.2);  // vel
      ASSERT_NEAR(state[i + 9], goal_state[i + 9], 0.1);  // rpydot
    }
  }

  void checkState(Eigen::VectorXd &state) {
    ASSERT_EQ(state.rows(), 15);
    for (int i = 0; i < 3; ++i) {
      ASSERT_FALSE(std::isnan(state[i]));      // position
      ASSERT_LT(std::abs(state[i + 6]), 3.0);  // velocity
      ASSERT_LT(std::abs(state[i + 9]), M_PI); // rpydot
    }
    for (int i = 0; i < 2; ++i) {
      ASSERT_LT(std::abs(state[i + 3]), M_PI / 4);  // rp
      ASSERT_LT(std::abs(state[i + 12]), M_PI / 4); // rp_cmd
    }
    ASSERT_FALSE(std::isnan(state[5])); // yaw
  }

  void checkControl(Eigen::VectorXd &control) {
    ASSERT_EQ(control.rows(), 4);
    ASSERT_LT(control[0], 2.0);
    ASSERT_GT(control[0], 0.1);
    for (int i = 0; i < 3; ++i) {
      ASSERT_LT(std::abs(control[i + 1]), 2 * M_PI); // rpy_dot
    }
  }

  void checkOutControl(Eigen::VectorXd &control) {
    ASSERT_EQ(control.rows(), 4);
    ASSERT_LT(control[0], 2.0 * 9.81 / 0.16);
    ASSERT_GT(control[0], 0.1 * 9.81 / 0.16);
    for (int i = 0; i < 2; ++i) {
      ASSERT_LT(std::abs(control[i + 1]), M_PI / 4); // rp
    }
    ASSERT_LT(std::abs(control[3]), M_PI); // yaw rate
  }

protected:
  QuadMPCControllerConfig config_;
};

TEST_F(DDPQuadMPCControllerTests, Constructor) {
  ASSERT_NO_THROW(createController());
}

TEST_F(DDPQuadMPCControllerTests, SingleRun) {
  config_.mutable_ddp_config()->set_n(100);
  config_.mutable_ddp_config()->set_max_cost(20);
  config_.mutable_ddp_config()->set_min_cost_decrease(1e-3);
  std::shared_ptr<DDPQuadMPCController> controller = createController();
  VLOG(1) << "Creating current sensor data";
  MPCInputs<Eigen::VectorXd> sensor_data;
  sensor_data.initial_state.setConstant(15, 1, 0);
  VLOG(1) << "Creating parameters";
  sensor_data.parameters.setConstant(1, 1, 0.16); // kt
  sensor_data.time_since_goal = 0.1; // Some random value does not matter
  // Set Goal
  Eigen::VectorXd goal_state(15);
  VLOG(1) << "Creating goal";
  goal_state.setZero();
  goal_state[0] = goal_state[1] = goal_state[2] = 0.1;
  goal_state[5] = goal_state[14] = 0.5; // Yaw, yawd
  Eigen::VectorXd goal_control(4);
  goal_control.setZero();
  goal_control[0] = 1.0;
  std::shared_ptr<QuadGcopWaypoint> way_point(
      new QuadGcopWaypoint(goal_state, goal_control));
  controller->setGoal(way_point);
  Eigen::VectorXd out_control;
  VLOG(1) << "Running Controller";
  controller->run(sensor_data, out_control);
  VLOG(1) << "Done optimizing";
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;
  controller->getTrajectory(xs_out, us_out);
  unsigned long N = us_out.size();
  // Check states and controls are meaningful
  for (unsigned int i = 0; i < N; ++i) {
    checkState(xs_out[i]);
    checkControl(us_out[i]);
  }
  VLOG(1) << "xs out: " << xs_out;
  // Terminal state
  checkState(xs_out[N], goal_state);
  checkOutControl(out_control);
  // Check control based on command
  ASSERT_GT(out_control[0], 1.0 * 9.81 / 0.16);
  ASSERT_LT(out_control[1], 0);
  ASSERT_GT(out_control[2], 0);
  ASSERT_GT(out_control[3], 0);
}

TEST_F(DDPQuadMPCControllerTests, Convergence) {
  config_.mutable_ddp_config()->set_n(100);
  config_.mutable_ddp_config()->set_max_cost(20);
  config_.mutable_ddp_config()->set_min_cost_decrease(1e-2);
  config_.mutable_ddp_config()->set_max_iters(5);
  std::shared_ptr<DDPQuadMPCController> controller = createController();
  VLOG(1) << "Creating current sensor data";
  MPCInputs<Eigen::VectorXd> sensor_data;
  sensor_data.initial_state.setConstant(15, 1, 0);
  VLOG(1) << "Creating parameters";
  sensor_data.parameters.setConstant(1, 1, 0.16); // kt
  sensor_data.time_since_goal = 0.0; // Some random value does not matter
  // Set Goal
  Eigen::VectorXd goal_state(15);
  VLOG(1) << "Creating goal";
  goal_state.setZero();
  goal_state[0] = goal_state[1] = goal_state[2] = 0.5;
  goal_state[5] = goal_state[14] = 0.5; // Yaw, yawd
  Eigen::VectorXd goal_control(4);
  goal_control.setZero();
  goal_control[0] = 1.0;
  std::shared_ptr<QuadGcopWaypoint> way_point(
      new QuadGcopWaypoint(goal_state, goal_control));
  controller->setGoal(way_point);
  Eigen::VectorXd out_control;
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;
  // Try following the trajectory perfectly
  while (sensor_data.time_since_goal < 10 &&
         !controller->isConverged(sensor_data)) {
    controller->run(sensor_data, out_control);
    controller->getTrajectory(xs_out, us_out);
    sensor_data.time_since_goal += 0.02;
    sensor_data.initial_state = xs_out[1];
    VLOG(1) << "Loop period: " << controller->getLoopTime();
  }
  ASSERT_EQ(controller->isConverged(sensor_data), ControllerStatus::Completed);
  controller->getTrajectory(xs_out, us_out);
  checkState(xs_out[0], goal_state);
}

TEST_F(DDPQuadMPCControllerTests, ExpReference) {
  config_.mutable_ddp_config()->set_n(50);
  config_.mutable_ddp_config()->set_max_cost(50);
  config_.mutable_ddp_config()->set_min_cost_decrease(1e-2);
  config_.mutable_ddp_config()->set_max_iters(5);
  config_.set_goal_position_tolerance(0.01);
  config_.set_goal_velocity_tolerance(0.005);
  std::shared_ptr<DDPQuadMPCController> controller = createController();
  VLOG(1) << "Creating current sensor data";
  MPCInputs<Eigen::VectorXd> sensor_data;
  sensor_data.initial_state.setConstant(15, 1, 0);
  VLOG(1) << "Creating parameters";
  sensor_data.parameters.setConstant(1, 1, 0.16); // kt
  sensor_data.time_since_goal = 0.0;
  // Reference config
  ParticleReferenceConfig reference_config;
  reference_config.set_kp_x(1);
  reference_config.set_kp_y(1);
  reference_config.set_kp_z(1);
  reference_config.set_kp_yaw(1);
  reference_config.set_max_velocity(0.2);
  reference_config.set_max_yaw_rate(0.2);

  PositionYaw start_position_yaw(0, 0, 0, 0);
  PositionYaw goal_position_yaw(0.2, 0.5, 0.2, 0.2);
  std::shared_ptr<QuadParticleTrajectory> reference_trajectory(
      new QuadParticleTrajectory(goal_position_yaw, start_position_yaw,
                                 reference_config));

  controller->setGoal(reference_trajectory);
  Eigen::VectorXd out_control;
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;
  // Initialize by running DDP
  controller->run(sensor_data, out_control);
  // Try following the trajectory perfectly
  while (sensor_data.time_since_goal < 10 &&
         !controller->isConverged(sensor_data)) {
    controller->run(sensor_data, out_control);
    controller->getTrajectory(xs_out, us_out);
    sensor_data.time_since_goal += 0.02;
    sensor_data.initial_state = xs_out[1];
    VLOG(1) << "Loop period: " << controller->getLoopTime();
  }
  ASSERT_EQ(controller->isConverged(sensor_data), ControllerStatus::Completed);
  controller->getTrajectory(xs_out, us_out);
  std::pair<Eigen::VectorXd, Eigen::VectorXd> goal_state_control_pair =
      reference_trajectory->atTime(sensor_data.time_since_goal);
  checkState(xs_out[0], goal_state_control_pair.first);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
