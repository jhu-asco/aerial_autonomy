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

/// \brief Test UAV System
TEST(UAVSystemTests, Constructor) {
  QuadSimulator drone_hardware;

  ASSERT_NO_THROW(new UAVSystem(drone_hardware));
}

TEST(UAVSystemTests, Takeoff) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  uav_system.takeOff();
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_STREQ(data.quadstate.c_str(), "ARMED ENABLE_CONTROL ");
}

TEST(UAVSystemTests, Land) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  uav_system.land();
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_STREQ(data.quadstate.c_str(), "ENABLE_CONTROL ");
}

TEST(UAVSystemTests, EnableSDK) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  // Disable SDK
  drone_hardware.flowControl(false);
  // Enable SDK
  uav_system.enableAutonomousMode();
  // Check status is updated in data
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_TRUE(data.rc_sdk_control_switch);
}

TEST(UAVSystemTests, SetGetGoal) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);
  PositionYaw goal =
      uav_system.getGoal<PositionControllerDroneConnector, PositionYaw>();
  ASSERT_EQ(goal, position_yaw);
}

TEST(UAVSystemTests, runActiveController) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
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
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
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
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  uav_system.takeOff();
  // set rc channels
  int16_t channels[4] = {100, 50, 25, 100};
  drone_hardware.setRC(channels);
  drone_hardware.set_delay_send_time(0.02);
  // set goal
  uav_system.setGoal<ManualRPYTControllerDroneConnector>(EmptyGoal());
  // Run 10 iterations
  for (int i = 0; i < 100; ++i) {
    uav_system.runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00523599, 1e-4);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00261799, 1e-4);
  // Verify Omegaz
  ASSERT_NEAR(sensor_data.omega.z, -0.00314, 1e-3);
}

TEST(UAVSystemTests, runManualVelocityController) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  uav_system.takeOff();
  
  // set rc channels
  int16_t channels[4] = {0, 0, 0, 0};
  drone_hardware.setRC(channels);
  drone_hardware.set_delay_send_time(0.02);
  // set goal
  uav_system.setGoal<ManualVelocityControllerDroneConnector>(EmptyGoal());
  // Run 10 iterations
  for (int i = 0; i < 100; ++i) {
    uav_system.runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00, 1e-4);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00, 1e-4);
  // Verify Omegaz
  ASSERT_NEAR(sensor_data.omega.z, 0.00, 1e-3);
}


TEST(UAVSystemTests, getActiveControllerStatus) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);

  ControllerStatus status;
  ASSERT_TRUE(uav_system.getActiveControllerStatus(HardwareType::UAV, status));
  ASSERT_EQ(status, ControllerStatus::Active);

  uav_system.runActiveController(HardwareType::UAV);
  uav_system.runActiveController(HardwareType::UAV);
  ASSERT_TRUE(uav_system.getActiveControllerStatus(HardwareType::UAV, status));
  ASSERT_EQ(status, ControllerStatus::Completed);

  uav_system.abortController(HardwareType::UAV);
  ASSERT_FALSE(uav_system.getActiveControllerStatus(HardwareType::UAV, status));
}

TEST(UAVSystemTests, abortController) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(drone_hardware);
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
  ros::init(argc, argv, "uav_system_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
