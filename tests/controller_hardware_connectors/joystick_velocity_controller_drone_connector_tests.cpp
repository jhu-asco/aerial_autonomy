#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controllers/joystick_velocity_controller.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

using namespace quad_simulator;

TEST(JoystickVelocityControllerDroneConnectorTests, Constructor) {
  double dt = 0.02;
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  JoystickVelocityController controller(joystick_config, dt);
  VelocitySensorConfig vel_config;
  std::shared_ptr<Sensor<Velocity>> sensor;
  ASSERT_NO_THROW(new JoystickVelocityControllerDroneConnector(
      drone_hardware, controller, sensor));
}

TEST(JoystickVelocityControllerDroneConnectorTests, Run) {
  double dt = 0.02;
  RPYTBasedVelocityControllerConfig rpyt_config_;
  rpyt_config_.set_kp_xy(2.0);
  rpyt_config_.set_kp_z(2.0);
  rpyt_config_.set_ki_xy(0.01);
  rpyt_config_.set_ki_z(0.01);
  auto vel_ctlr_config = rpyt_config_.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();

  tolerance->set_vx(1e-3);
  tolerance->set_vy(1e-3);
  tolerance->set_vz(1e-3);
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  drone_hardware.usePerfectTime();
  VelocitySensorConfig vel_config;
  std::shared_ptr<Sensor<Velocity>> sensor;
  JoystickVelocityController controller(joystick_config, dt);
  controller.updateRPYTConfig(rpyt_config_);

  // Set stick commands
  int16_t channels[4] = {150, 100, -150, 100};
  drone_hardware.setRC(channels);

  JoystickVelocityControllerDroneConnector connector(drone_hardware, controller,
                                                     sensor);

  connector.setGoal(EmptyGoal());

  auto controller_run = [&] {
    connector.run();
    return (connector.getStatus() == ControllerStatus::Completed);
  };

  ASSERT_TRUE(test_utils::waitUntilTrue()(
      controller_run, std::chrono::seconds(1), std::chrono::milliseconds(0)));

  parsernode::common::quaddata sensor_data;
  drone_hardware.getquaddata(sensor_data);

  std::cout << "yaw = " << sensor_data.rpydata.z << std::endl;
  ASSERT_NEAR(sensor_data.linvel.x, channels[0] / 1e4, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.y, channels[1] / 1e4, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.z, channels[2] / 1e4, 1e-3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
