#include <aerial_autonomy/controller_hardware_connectors/manual_rpyt_controller_drone_connector.h>
#include <aerial_autonomy/controllers/manual_rpyt_controller.h>
#include <aerial_autonomy/tests/sample_parser.h>
#include <gtest/gtest.h>

/// \brief Test ManualRPYTController
TEST(ManualRPYTControllerTests, TestMapInputOutOfBounds) {
  ManualRPYTController manual_rpyt_controller;
  JoysticksYaw input(15000, -15000, 0, 0, 0);
  RollPitchYawThrust out_controls = manual_rpyt_controller.run(input);
  ASSERT_NEAR(out_controls.r, M_PI / 6, 1e-8);
  ASSERT_NEAR(out_controls.p, -M_PI / 6, 1e-8);
}

TEST(ManualRPYTControllerTests, TestYawGreaterThanPi) {
  ManualRPYTController manual_rpyt_controller;
  JoysticksYaw input(0, 0, 0, 0, 1.5 * M_PI);
  RollPitchYawThrust out_controls = manual_rpyt_controller.run(input);
  ASSERT_NEAR(out_controls.y, -0.5 * M_PI, 1e-8);
}

TEST(ManualRPYTControllerTests, TestYawLessThanNegativePi) {
  ManualRPYTController manual_rpyt_controller;
  JoysticksYaw input(0, 0, 0, 0, -1.5 * M_PI);
  RollPitchYawThrust out_controls = manual_rpyt_controller.run(input);
  ASSERT_NEAR(out_controls.y, 0.5 * M_PI, 1e-8);
}
///

/// \brief Test ManualRPYTControllerDroneConnectorTests
TEST(ManualRPYTControllerDroneConnectorTests, Constructor) {
  SampleParser drone_hardware;

  ManualRPYTController manual_rpyt_controller;

  ASSERT_NO_THROW(new ManualRPYTControllerDroneConnector(
      drone_hardware, manual_rpyt_controller));
}

TEST(ManualRPYTControllerDroneConnectorTests, Run) {
  SampleParser drone_hardware;

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
  manual_rpyt_controller_connector.run();
  drone_hardware.getquaddata(sensor_data);
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00523599, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00261799, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.z, -0.00062831, 1e-8);

  manual_rpyt_controller_connector.run();
  drone_hardware.getquaddata(sensor_data);
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00523599, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00261799, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.z, -0.00125663, 1e-8);

  // TODO(matt) : Need to test thrust.  Could test if we have a parser with
  // dynamics
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
