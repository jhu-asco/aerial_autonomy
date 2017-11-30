#include <aerial_autonomy/robot_systems/uav_system.h>
//#include <aerial_autonomy/tests/sample_parser.h>
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;
using ParserPtr = std::shared_ptr<parsernode::Parser>;

/// \brief Test UAV System
TEST(UAVSystemTests, Constructor) {
  ASSERT_NO_THROW(new UAVSystem(ParserPtr(new QuadSimulator)));
}

TEST(UAVSystemTests, Takeoff) {
  UAVSystem uav_system{ParserPtr(new QuadSimulator)};
  uav_system.takeOff();
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_STREQ(data.quadstate.c_str(), "ARMED ENABLE_CONTROL ");
}

TEST(UAVSystemTests, Land) {
  UAVSystem uav_system(ParserPtr(new QuadSimulator));
  uav_system.land();
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_STREQ(data.quadstate.c_str(), "ENABLE_CONTROL ");
}

TEST(UAVSystemTests, EnableSDK) {
  std::shared_ptr<QuadSimulator> drone_hardware(new QuadSimulator);
  UAVSystem uav_system{
      std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware)};
  // Disable SDK
  drone_hardware->flowControl(false);
  // Enable SDK
  uav_system.enableAutonomousMode();
  // Check status is updated in data
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_TRUE(data.rc_sdk_control_switch);
}

TEST(UAVSystemTests, SetGetGoal) {
  UAVSystem uav_system(ParserPtr(new QuadSimulator));
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);
  PositionYaw goal =
      uav_system.getGoal<PositionControllerDroneConnector, PositionYaw>();
  ASSERT_EQ(goal, position_yaw);
}

TEST(UAVSystemTests, runActiveController) {
  UAVSystem uav_system(ParserPtr(new QuadSimulator));
  uav_system.takeOff();
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  PositionYaw data_position_yaw(data.localpos.x, data.localpos.y,
                                data.localpos.z, data.rpydata.z);
  ASSERT_EQ(data_position_yaw, position_yaw);
}

TEST(UAVSystemTests, runVelocityController) {
  UAVSystem uav_system(ParserPtr(new QuadSimulator));
  uav_system.takeOff();
  VelocityYaw velocity_yaw(1, 1, 1, 1);
  uav_system.setGoal<BuiltInVelocityControllerDroneConnector>(velocity_yaw);
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  VelocityYaw data_velocity_yaw(data.linvel.x, data.linvel.y, data.linvel.z,
                                data.rpydata.z);
  ASSERT_EQ(data_velocity_yaw, velocity_yaw);
}

TEST(UAVSystemTests, runRPYTController) {
  QuadSimulator *drone_hardware = new QuadSimulator;
  drone_hardware->usePerfectTime();
  UAVSystem uav_system{ParserPtr(drone_hardware)};
  uav_system.takeOff();
  // set rc channels
  int16_t channels[4] = {100, 50, 25, 100};
  drone_hardware->setRC(channels);
  drone_hardware->set_delay_send_time(0.02);
  // set goal
  uav_system.setGoal<ManualRPYTControllerDroneConnector>(EmptyGoal());
  // Run 10 iterations
  for (int i = 0; i < 100; ++i) {
    uav_system.runActiveController(HardwareType::UAV);
  }
  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00523599, 1e-4);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00261799, 1e-4);
  // Verify Omegaz
  ASSERT_NEAR(sensor_data.omega.z, M_PI / 100, 1e-3);
}

TEST(UAVSystemTests, getActiveControllerStatus) {
  UAVSystemConfig config;
  auto position_tolerance = config.mutable_position_controller_config()
                                ->mutable_goal_position_tolerance();
  position_tolerance->set_x(0.5);
  position_tolerance->set_y(0.5);
  position_tolerance->set_z(0.5);
  UAVSystem uav_system(config, ParserPtr(new QuadSimulator));
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);

  ASSERT_EQ(uav_system.getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::Active);

  uav_system.runActiveController(HardwareType::UAV);
  uav_system.runActiveController(HardwareType::UAV);
  ASSERT_TRUE(uav_system.getActiveControllerStatus(HardwareType::UAV));

  uav_system.abortController(HardwareType::UAV);
  ASSERT_EQ(uav_system.getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::NotEngaged);
}

TEST(UAVSystemTests, abortController) {
  UAVSystem uav_system(ParserPtr(new QuadSimulator));
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);
  uav_system.abortController(HardwareType::UAV);
  // Once aborted will not run position controller
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  PositionYaw data_position_yaw(data.localpos.x, data.localpos.y,
                                data.localpos.z, data.rpydata.z);
  ASSERT_NE(data_position_yaw, position_yaw);
}

///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
