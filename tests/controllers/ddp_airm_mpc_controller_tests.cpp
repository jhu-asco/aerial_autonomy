#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "aerial_autonomy/types/waypoint.h"
#include "ddp_mpc_controller_config.pb.h"

#include <gtest/gtest.h>

class DDPAirmMPCControllerTests : public ::testing::Test {
public:
  using AirmGcopWaypoint = Waypoint<Eigen::VectorXd, Eigen::VectorXd>;

  DDPAirmMPCControllerTests() {
    DDPMPCControllerConfig *ddp_config = config_.mutable_ddp_config();
    ddp_config->set_look_ahead_time(0.02);
    auto q = ddp_config->mutable_q();
    q->Resize(21, 0.0);
    auto qf = ddp_config->mutable_qf();
    qf->Resize(21, 0.1);
    auto r = ddp_config->mutable_r();
    r->Resize(6, 1.0);
    r->Set(0, 1e-4); // Reduce thrust control
    for (int i = 0; i < 3; ++i) {
      // Pos cost
      ddp_config->set_qf(i, 10.0);
      // Rpy cost
      ddp_config->set_qf(i + 3, 5.0);
      // Vel cost
      ddp_config->set_qf(i + 6, 1.0);
      ddp_config->set_q(i + 6, 1.0);
      // Rpydot cost
      ddp_config->set_qf(i + 9, 5.0);
    }
    for (int i = 0; i < 2; ++i) {
      // Joint angle cost
      ddp_config->set_qf(i + 15, 5.0);
      // Joint velocity cost
      ddp_config->set_qf(i + 17, 2.0);
    }
    ddp_config->set_debug(false);
    ddp_config->set_max_iters(100);
    config_.set_weights_folder(
        std::string(PROJECT_SOURCE_DIR) +
        "/neural_network_model_data/tensorflow_model_vars_16_8_tanh/");
  }

  std::shared_ptr<DDPAirmMPCController>
  createController(double controller_duration = 0.02) {
    std::shared_ptr<DDPAirmMPCController> controller(
        new DDPAirmMPCController(config_, controller_duration));
    return controller;
  }

  /*void testConvergence(
      const shared_ptr<ReferenceTrajectory<ParticleState, Snap>> &ref,
      DDPAirmMPCState x0) {
    DDPAirmMPCController controller(config_);

    auto sensor_data = std::make_pair(0.0, x0);
    double &time = std::get<0>(sensor_data);
    auto &qrotor_state = std::get<1>(sensor_data);
    qrotor_state.thrust = config_.mass() * config_.acc_gravity();

    std::chrono::duration<double> dt = std::chrono::milliseconds(20);

    controller.setGoal(ref);

    auto convergence = [&]() {
      DDPAirmMPCControl controls;
      if (!controller.run(sensor_data, controls)) {
        return false;
      }

      qrotor_state.thrust_dot += controls.thrust_ddot * dt.count();
      qrotor_state.thrust += qrotor_state.thrust_dot * dt.count();

      Body3dState xa;
      tf::matrixTFToEigen(qrotor_state.pose.getBasis(), xa.R);
      tf::vectorTFToEigen(qrotor_state.pose.getOrigin(), xa.p);
      tf::vectorTFToEigen(qrotor_state.w, xa.w);
      tf::vectorTFToEigen(qrotor_state.v, xa.v);

      Body3dState xb;
      Eigen::Vector4d controls_eig(controls.torque.x(), controls.torque.y(),
                                   controls.torque.z(), qrotor_state.thrust);
      sys.Step(xb, time, xa, controls_eig, dt.count());
      time += dt.count();

      tf::matrixEigenToTF(xb.R, qrotor_state.pose.getBasis());
      tf::vectorEigenToTF(xb.p, qrotor_state.pose.getOrigin());
      tf::vectorEigenToTF(xb.w, qrotor_state.w);
      tf::vectorEigenToTF(xb.v, qrotor_state.v);

      return bool(controller.isConverged(sensor_data));
    };

    ASSERT_TRUE(test_utils::waitUntilTrue()(
        convergence, std::chrono::seconds(5), std::chrono::milliseconds(0)));
  }
  */

protected:
  AirmMPCControllerConfig config_;
};

TEST_F(DDPAirmMPCControllerTests, ConstructorWithoutResidualDynamics) {
  ASSERT_NO_THROW(createController());
}

TEST_F(DDPAirmMPCControllerTests, ConstructorWithResidualDynamics) {
  config_.set_use_residual_dynamics(false);
  ASSERT_NO_THROW(createController());
}

TEST_F(DDPAirmMPCControllerTests, SingleRunWithResidualDynamics) {
  // config_.set_use_residual_dynamics(false);
  config_.mutable_ddp_config()->set_n(100);
  std::shared_ptr<DDPAirmMPCController> controller = createController();
  VLOG(1) << "Creating current sensor data";
  MPCInputs<Eigen::VectorXd> sensor_data;
  sensor_data.initial_state.setConstant(21, 1, 0);
  sensor_data.initial_state[15] = -0.8;
  sensor_data.initial_state[16] = 1.4;
  sensor_data.initial_state[19] = sensor_data.initial_state[15];
  sensor_data.initial_state[20] = sensor_data.initial_state[16];
  VLOG(1) << "Creating parameters";
  sensor_data.parameters.setConstant(1, 1, 0.16); // kt
  sensor_data.time_since_goal = 0.1; // Some random value does not matter
  // Set Goal
  Eigen::VectorXd goal_state(21);
  VLOG(1) << "Creating goal";
  goal_state.setZero();
  goal_state[0] = goal_state[1] = goal_state[2] = 1.0;
  goal_state[5] = goal_state[14] = 0.5; // Yaw, yawd
  // Joint angles
  goal_state[15] = -0.3;
  goal_state[16] = 0.7;
  goal_state[19] = goal_state[15]; // Desired joint angles
  goal_state[20] = goal_state[16];
  Eigen::VectorXd goal_control(6);
  goal_control.setZero();
  goal_control[0] = 9.81 / sensor_data.parameters[0];
  std::shared_ptr<AirmGcopWaypoint> way_point(
      new AirmGcopWaypoint(goal_state, goal_control));
  controller->setGoal(way_point);
  Eigen::VectorXd out_control;
  VLOG(1) << "Running Controller";
  controller->run(sensor_data, out_control);
  VLOG(1) << "Done optimizing";
  std::vector<Eigen::VectorXd> xs_out;
  std::vector<Eigen::VectorXd> us_out;
  controller->getTrajectory(xs_out, us_out);
  int N = us_out.size();
  for (int i = 0; i < N; ++i) {
    std::cout << "Xs[" << i << "]: " << xs_out.at(i).transpose() << std::endl;
    std::cout << "Us[" << i << "]: " << us_out.at(i).transpose() << std::endl;
  }
  std::cout << "Xf: " << xs_out.back().transpose() << std::endl;
  // Check Xf is close enough to desired goal by adjusting cost function
  ASSERT_EQ(out_control.rows(), 6);
  ASSERT_GT(out_control[0], 9.81 / sensor_data.parameters[0]);
  ASSERT_LT(out_control[1], 0);
  ASSERT_GT(out_control[2], 0);
  ASSERT_GT(out_control[3], 0);
  // Joint des vel
  ASSERT_GT(out_control[4], 0);
  ASSERT_LT(out_control[5], 0);
}

/*TEST_F(DDPAirmMPCControllerTests, Convergence) {
  shared_ptr<DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>> ref(
      new DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>());
  for (double t = 0; t < 200; t += 0.05) {
    ref->ts.push_back(t);
    ref->states.push_back(ParticleState());
    ref->controls.push_back(Snap());
  }

  DDPAirmMPCState qrotor_state;
  qrotor_state.pose.setOrigin(tf::Vector3(3, -3, 1));
  testConvergence(ref, qrotor_state);

  // Non-zero linear velocity
  qrotor_state.v = tf::Vector3(1, 1, -1);
  testConvergence(ref, qrotor_state);

  // Non-zero angular velocity
  qrotor_state.w = tf::Vector3(M_PI / 6, -M_PI / 4, 0);
  testConvergence(ref, qrotor_state);
}

TEST_F(DDPAirmMPCControllerTests, ConvergenceSpiral) {
  shared_ptr<DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>> ref(
      new DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>());
  double w_xy = 2 * M_PI * 0.2;
  double w_z = 2 * M_PI * 0.2;
  for (double t = 0; t < 200; t += 0.05) {
    ref->ts.push_back(t);
    ParticleState desired_state;
    desired_state.p.x = cos(w_xy * t);
    desired_state.p.y = sin(w_xy * t);
    desired_state.p.z = sin(w_z * t) + 3;
    desired_state.v.x = w_xy * -sin(w_xy * t);
    desired_state.v.y = w_xy * cos(w_xy * t);
    desired_state.v.z = w_z * cos(w_z * t);
    desired_state.a.x = std::pow(w_xy, 2) * -cos(w_xy * t);
    desired_state.a.y = std::pow(w_xy, 2) * -sin(w_xy * t);
    desired_state.a.z = std::pow(w_z, 2) * -sin(w_z * t);
    desired_state.j.x = std::pow(w_xy, 3) * sin(w_xy * t);
    desired_state.j.y = std::pow(w_xy, 3) * -cos(w_xy * t);
    desired_state.j.z = std::pow(w_z, 3) * -cos(w_z * t);

    Snap desired_control(std::pow(w_xy, 4) * cos(w_xy * t),
                         std::pow(w_xy, 4) * sin(w_xy * t),
                         std::pow(w_z, 4) * sin(w_z * t));

    ref->states.push_back(desired_state);
    ref->controls.push_back(desired_control);
  }

  DDPAirmMPCState qrotor_state;
  testConvergence(ref, qrotor_state);

  // Non-zero linear velocity
  qrotor_state.v = tf::Vector3(1, 1, -1);
  testConvergence(ref, qrotor_state);

  // Non-zero angular velocity
  qrotor_state.w = tf::Vector3(M_PI / 6, -M_PI / 4, 0);
  testConvergence(ref, qrotor_state);
}

TEST_F(DDPAirmMPCControllerTests, SmallThrust) {
  shared_ptr<DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>> ref(
      new DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>());
  ref->ts.push_back(0);
  ref->states.push_back(ParticleState());
  ref->controls.push_back(Snap());
  DDPAirmMPCController controller(config_);

  controller.setGoal(ref);

  auto sensor_data = std::make_pair(0.0, DDPAirmMPCState());
  auto &qrotor_state = std::get<1>(sensor_data);
  qrotor_state.thrust = 0;
  qrotor_state.pose.setOrigin(tf::Vector3(3, 2, 1));

  DDPAirmMPCControl controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.torque.x(), 0);
  ASSERT_EQ(controls.torque.y(), 0);

  qrotor_state.thrust = config_.thrust_eps() + 1e-5;
  controller.run(sensor_data, controls);

  ASSERT_NE(controls.torque.x(), 0);
  ASSERT_NE(controls.torque.y(), 0);
}

TEST_F(DDPAirmMPCControllerTests, ReferenceTooShort) {
  shared_ptr<DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>> ref(
      new DiscreteReferenceTrajectoryInterpolate<ParticleState, Snap>());
  DDPAirmMPCController controller(config_);

  controller.setGoal(ref);
  auto sensor_data = std::make_pair(0.0, DDPAirmMPCState());

  DDPAirmMPCControl controls;
  ASSERT_FALSE(controller.run(sensor_data, controls));
}
*/

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
