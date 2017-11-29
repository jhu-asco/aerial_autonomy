#include <aerial_autonomy/controller_hardware_connectors/manual_rpyt_controller_drone_connector.h>
#include <aerial_autonomy/controllers/manual_rpyt_controller.h>
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/// \brief Test ManualRPYTController
TEST(ManualRPYTControllerTests, TestMapInputOutOfBounds) {
  ManualRPYTController manual_rpyt_controller;
  Joystick input(15000, -15000, 0, 0);
  RollPitchYawRateThrust out_controls;
  bool result = manual_rpyt_controller.run(input, out_controls);
  ASSERT_NEAR(out_controls.r, M_PI / 6, 1e-8);
  ASSERT_NEAR(out_controls.p, -M_PI / 6, 1e-8);
  ASSERT_NEAR(out_controls.y, 0, 1e-8);
  ASSERT_TRUE(result);
}

TEST(ManualRPYTControllerTests, TestYawGreaterThanPi) {
  ManualRPYTController manual_rpyt_controller;
  Joystick input(0, 0, 0, 15000);
  RollPitchYawRateThrust out_controls;
  manual_rpyt_controller.run(input, out_controls);
  ASSERT_NEAR(out_controls.y, M_PI, 1e-8);
}

TEST(ManualRPYTControllerTests, TestYawLessThanNegativePi) {
  ManualRPYTController manual_rpyt_controller;
  Joystick input(0, 0, 0, -15000);
  RollPitchYawRateThrust out_controls;
  manual_rpyt_controller.run(input, out_controls);
  ASSERT_NEAR(out_controls.y, -M_PI, 1e-8);
}
///

/// \brief Test ManualRPYTControllerDroneConnectorTests
TEST(ManualRPYTControllerDroneConnectorTests, Constructor) {
  QuadSimulator drone_hardware;

  ManualRPYTController manual_rpyt_controller;

  ASSERT_NO_THROW(new ManualRPYTControllerDroneConnector(
      drone_hardware, manual_rpyt_controller));
}

TEST(ManualRPYTControllerDroneConnectorTests, Run) {
  QuadSimulator drone_hardware;
  drone_hardware.usePerfectTime();
  drone_hardware.takeoff();
  drone_hardware.set_delay_send_time(0.02);

  // Set stick commands
  int16_t channels[4] = {100, 50, 25, 100};
  drone_hardware.setRC(channels);

  // Create controller and connector
  ManualRPYTController manual_rpyt_controller;
  ManualRPYTControllerDroneConnector manual_rpyt_controller_connector(
      drone_hardware, manual_rpyt_controller);

  // Test set goal
  manual_rpyt_controller_connector.setGoal(EmptyGoal());

  // Test run
  parsernode::common::quaddata sensor_data;
  for (int i = 0; i < 100; ++i) {
    manual_rpyt_controller_connector.run();
  }
  ASSERT_EQ(manual_rpyt_controller_connector.getStatus(),
            ControllerStatus::Completed);
  drone_hardware.getquaddata(sensor_data);
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00523599, 1e-4);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00261799, 1e-4);
  // Verify Omegaz
  ASSERT_NEAR(sensor_data.omega.z, M_PI / 100.0, 1e-3);
}

TEST(ManualRPYTControllerDroneConnectorTests, RunConstantAcceleration) {
  QuadSimulator drone_hardware;
  drone_hardware.takeoff();
  drone_hardware.set_delay_send_time(0.02);

  // Create controller and connector
  ManualRPYTController manual_rpyt_controller;
  ManualRPYTControllerDroneConnector manual_rpyt_controller_connector(
      drone_hardware, manual_rpyt_controller);

  // Test set goal
  manual_rpyt_controller_connector.setGoal(EmptyGoal());

  // Set stick commands
  int16_t channels[4] = {0, 0, 2000, 0};
  drone_hardware.setRC(channels);

  parsernode::common::quaddata first_data, second_data, third_data;
  // Test run
  drone_hardware.getquaddata(first_data);
  for (int i = 0; i < 50; ++i) {
    manual_rpyt_controller_connector.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  drone_hardware.getquaddata(second_data);
  ASSERT_NEAR(second_data.localpos.x, first_data.localpos.x, 1e-4);
  ASSERT_NEAR(second_data.localpos.y, first_data.localpos.y, 1e-4);
  // for 1 more second
  for (int i = 0; i < 50; ++i) {
    manual_rpyt_controller_connector.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  drone_hardware.getquaddata(third_data);
  ASSERT_NEAR(third_data.localpos.x, first_data.localpos.x, 1e-4);
  ASSERT_NEAR(third_data.localpos.y, first_data.localpos.y, 1e-4);
  double zdiff_third = third_data.localpos.z - first_data.localpos.z;
  double zdiff_second = second_data.localpos.z - first_data.localpos.z;
  ASSERT_NEAR(zdiff_third, 4 * zdiff_second, 5e-2);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
