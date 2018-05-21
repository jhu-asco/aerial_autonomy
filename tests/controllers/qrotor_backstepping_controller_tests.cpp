#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "qrotor_backstepping_controller_config.pb.h"

#include <chrono>
#include <gcop/hrotor.h>
#include <tf_conversions/tf_eigen.h>

#include <gtest/gtest.h>

using namespace gcop;

class QrotorBacksteppingControllerTests : public ::testing::Test {
public:
  QrotorBacksteppingControllerTests() : sys() {
    config_.set_mass(sys.m);
    config_.set_jx(sys.J(0));
    config_.set_jy(sys.J(1));
    config_.set_jz(sys.J(2));
    config_.set_k2(0.1);
    config_.set_k1(0.1);
    config_.set_kp_xy(0.1);
    config_.set_kp_z(0.1);
    config_.set_kd_xy(0.05);
    config_.set_kd_z(0.05);
    auto vel_tolerance = config_.mutable_goal_velocity_tolerance();
    vel_tolerance->set_vx(0.01);
    vel_tolerance->set_vy(0.01);
    vel_tolerance->set_vz(0.01);

    auto pos_tolerance = config_.mutable_goal_position_tolerance();
    pos_tolerance->set_x(0.05);
    pos_tolerance->set_y(0.05);
    pos_tolerance->set_z(0.05);
  }

  void testConvergence(QrotorBSState x0) {
    ReferenceTrajectory<ParticleState, Snap> ref;
    ref.ts.push_back(0);
    ref.states.push_back(ParticleState());
    ref.controls.push_back(Snap());
    QrotorBacksteppingController controller(config_);

    auto sensor_data = std::make_tuple(0.0, x0);
    double &time = std::get<0>(sensor_data);
    auto &qrotor_state = std::get<1>(sensor_data);
    qrotor_state.thrust = config_.mass() * config_.acc_gravity();

    std::chrono::duration<double> dt = std::chrono::milliseconds(20);

    controller.setGoal(ref);

    auto convergence = [&]() {
      QrotorBSControl controls;
      controller.run(sensor_data, controls);

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

      tf::matrixEigenToTF(xb.R, qrotor_state.pose.getBasis());
      tf::vectorEigenToTF(xb.p, qrotor_state.pose.getOrigin());
      tf::vectorEigenToTF(xb.w, qrotor_state.w);
      tf::vectorEigenToTF(xb.v, qrotor_state.v);

      return bool(controller.isConverged(sensor_data));
    };

    ASSERT_TRUE(test_utils::waitUntilTrue()(
        convergence, std::chrono::seconds(1), std::chrono::milliseconds(0)));
  }

protected:
  Hrotor sys;
  QrotorBacksteppingControllerConfig config_;
};

TEST_F(QrotorBacksteppingControllerTests, Constructor) {
  ASSERT_NO_THROW(new QrotorBacksteppingController(config_));
}

TEST_F(QrotorBacksteppingControllerTests, Convergence) {
  QrotorBSState qrotor_state;
  qrotor_state.pose.setOrigin(tf::Vector3(3, -3, 1));
  testConvergence(qrotor_state);

  // Non-zero linear velocity
  qrotor_state.v = tf::Vector3(1, 1, -1);
  testConvergence(qrotor_state);

  // Non-zero angular velocity
  qrotor_state.w = tf::Vector3(M_PI / 6, -M_PI / 4, 0);
  testConvergence(qrotor_state);
}

TEST_F(QrotorBacksteppingControllerTests, SmallThrust) {
  ReferenceTrajectory<ParticleState, Snap> ref;
  ref.ts.push_back(0);
  ref.states.push_back(ParticleState());
  ref.controls.push_back(Snap());
  QrotorBacksteppingController controller(config_);

  controller.setGoal(ref);

  auto sensor_data = std::make_tuple(0.0, QrotorBSState());
  auto &qrotor_state = std::get<1>(sensor_data);
  qrotor_state.thrust = 0;
  qrotor_state.pose.setOrigin(tf::Vector3(3, 2, 1));

  QrotorBSControl controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.torque.x(), 0);
  ASSERT_EQ(controls.torque.y(), 0);

  qrotor_state.thrust = config_.thrust_eps() + 1e-5;
  controller.run(sensor_data, controls);

  ASSERT_NE(controls.torque.x(), 0);
  ASSERT_NE(controls.torque.y(), 0);
}

// TODO Matt: add test for actual trajectory tracking once trajectory generation
// is implemented

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
