#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "qrotor_backstepping_controller_config.pb.h"

#include <chrono>
#include <cmath>
#include <gcop/hrotor.h>
#include <tf_conversions/tf_eigen.h>

#include <gtest/gtest.h>

using namespace gcop;

class QrotorBacksteppingControllerTests : public ::testing::Test {
public:
  QrotorBacksteppingControllerTests() : sys() {
    config_.set_mass(sys.m);
    config_.set_jxx(sys.J(0));
    config_.set_jyy(sys.J(1));
    config_.set_jzz(sys.J(2));
    config_.set_k2(0.1);
    config_.set_k1(0.1);
    config_.set_kp_xy(0.4);
    config_.set_kp_z(0.4);
    config_.set_kd_xy(0.2);
    config_.set_kd_z(0.2);
    auto vel_tolerance = config_.mutable_goal_velocity_tolerance();
    vel_tolerance->set_vx(0.05);
    vel_tolerance->set_vy(0.05);
    vel_tolerance->set_vz(0.05);

    auto pos_tolerance = config_.mutable_goal_position_tolerance();
    pos_tolerance->set_x(0.05);
    pos_tolerance->set_y(0.05);
    pos_tolerance->set_z(0.05);
  }

  void testConvergence(ReferenceTrajectory<ParticleState, Snap> ref, QrotorBacksteppingState x0) {
    QrotorBacksteppingController controller(config_);

    auto sensor_data = std::make_pair(0.0, x0);
    double &time = std::get<0>(sensor_data);
    auto &qrotor_state = std::get<1>(sensor_data);
    qrotor_state.thrust = config_.mass() * config_.acc_gravity();

    std::chrono::duration<double> dt = std::chrono::milliseconds(20);

    controller.setGoal(ref);

    auto convergence = [&]() {
      QrotorBacksteppingControl controls;
      if(!controller.run(sensor_data, controls)) {
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

protected:
  Hrotor sys;
  QrotorBacksteppingControllerConfig config_;
};

TEST_F(QrotorBacksteppingControllerTests, Constructor) {
  ASSERT_NO_THROW(new QrotorBacksteppingController(config_));
}

TEST_F(QrotorBacksteppingControllerTests, Convergence) {
  ReferenceTrajectory<ParticleState, Snap> ref;
  for(double t = 0; t < 200; t+= 0.05) {
    ref.ts.push_back(t);
    ref.states.push_back(ParticleState());
    ref.controls.push_back(Snap());
  }

  QrotorBacksteppingState qrotor_state;
  qrotor_state.pose.setOrigin(tf::Vector3(3, -3, 1));
  testConvergence(ref, qrotor_state);

  // Non-zero linear velocity
  qrotor_state.v = tf::Vector3(1, 1, -1);
  testConvergence(ref, qrotor_state);

  // Non-zero angular velocity
  qrotor_state.w = tf::Vector3(M_PI / 6, -M_PI / 4, 0);
  testConvergence(ref, qrotor_state);
}

TEST_F(QrotorBacksteppingControllerTests, ConvergenceSpiral) {
  ReferenceTrajectory<ParticleState, Snap> ref;
  double w_xy = 2 * M_PI * 0.2;
  double w_z = 2 * M_PI * 0.2;
  for(double t = 0; t < 200; t+= 0.05) {
    ref.ts.push_back(t);
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

    ref.states.push_back(desired_state);
    ref.controls.push_back(desired_control);
  }

  QrotorBacksteppingState qrotor_state;
  testConvergence(ref, qrotor_state);

  // Non-zero linear velocity
  qrotor_state.v = tf::Vector3(1, 1, -1);
  testConvergence(ref, qrotor_state);

  // Non-zero angular velocity
  qrotor_state.w = tf::Vector3(M_PI / 6, -M_PI / 4, 0);
  testConvergence(ref, qrotor_state);
}

TEST_F(QrotorBacksteppingControllerTests, SmallThrust) {
  ReferenceTrajectory<ParticleState, Snap> ref;
  ref.ts.push_back(0);
  ref.states.push_back(ParticleState());
  ref.controls.push_back(Snap());
  QrotorBacksteppingController controller(config_);

  controller.setGoal(ref);

  auto sensor_data = std::make_pair(0.0, QrotorBacksteppingState());
  auto &qrotor_state = std::get<1>(sensor_data);
  qrotor_state.thrust = 0;
  qrotor_state.pose.setOrigin(tf::Vector3(3, 2, 1));

  QrotorBacksteppingControl controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.torque.x(), 0);
  ASSERT_EQ(controls.torque.y(), 0);

  qrotor_state.thrust = config_.thrust_eps() + 1e-5;
  controller.run(sensor_data, controls);

  ASSERT_NE(controls.torque.x(), 0);
  ASSERT_NE(controls.torque.y(), 0);
}

TEST_F(QrotorBacksteppingControllerTests, ReferenceTooShort) {
  ReferenceTrajectory<ParticleState, Snap> ref;
  QrotorBacksteppingController controller(config_);

  controller.setGoal(ref);
  auto sensor_data = std::make_pair(0.0, QrotorBacksteppingState());

  QrotorBacksteppingControl controls;
  ASSERT_FALSE(controller.run(sensor_data, controls));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
