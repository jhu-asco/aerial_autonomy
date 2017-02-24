#include <gtest/gtest.h>
#include <aerial_autonomy/robot_systems/quadrotor_system.h>
#include <aerial_autonomy/tests/sample_parser.h>

/// \brief Test UAV System
TEST(UAVSystemTests, Constructor) {
  SampleParser drone_hardware;

  ASSERT_NO_THROW(new UAVSystem(drone_hardware));
}

TEST(UAVSystemTests, Takeoff) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  uav_system.takeOff();
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_STREQ(data.quadstate.c_str(), "takeoff");
}

TEST(UAVSystemTests, Land) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  uav_system.land();
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_STREQ(data.quadstate.c_str(), "land");
}

TEST(UAVSystemTests, SetGetGoal) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);
  PositionYaw goal =
      uav_system.getGoal<PositionControllerDroneConnector, PositionYaw>();
  ASSERT_EQ(goal.x, position_yaw.x);
  ASSERT_EQ(goal.y, position_yaw.y);
  ASSERT_EQ(goal.z, position_yaw.z);
  ASSERT_EQ(goal.yaw, position_yaw.yaw);
}

TEST(UAVSystemTests, runActiveController) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_EQ(data.localpos.x, position_yaw.x);
  ASSERT_EQ(data.localpos.y, position_yaw.y);
  ASSERT_EQ(data.localpos.z, position_yaw.z);
  ASSERT_EQ(data.rpydata.z, position_yaw.yaw);
}

TEST(UAVSystemTests, runVelocityController) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  VelocityYaw velocity_yaw(1, 1, 1, 1);
  uav_system.setGoal<BuiltInVelocityControllerDroneConnector>(velocity_yaw);
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_EQ(data.linvel.x, velocity_yaw.x);
  ASSERT_EQ(data.linvel.y, velocity_yaw.y);
  ASSERT_EQ(data.linvel.z, velocity_yaw.z);
  ASSERT_EQ(data.rpydata.z, velocity_yaw.yaw);
}

TEST(UAVSystemTests, runRPYTController) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  // set rc channels
  int16_t channels[4] = {100, 50, 25, 100};
  drone_hardware.setRC(channels);
  // set goal
  uav_system.setGoal<ManualRPYTControllerDroneConnector>(EmptyGoal());
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00523599, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00261799, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.z, -0.00062831, 1e-8);

  // Verify Yaw rate
  uav_system.runActiveController(HardwareType::UAV);
  sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.rpydata.x, 0.00523599, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.y, 0.00261799, 1e-8);
  ASSERT_NEAR(sensor_data.rpydata.z, -0.00125663, 1e-8);
}

TEST(UAVSystemTests, abortController) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  PositionYaw position_yaw(1, 1, 1, 1);
  uav_system.setGoal<PositionControllerDroneConnector>(position_yaw);
  uav_system.abortController(HardwareType::UAV);
  // Once aborted will not run position controller
  uav_system.runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system.getUAVData();
  ASSERT_NE(data.localpos.x, position_yaw.x);
  ASSERT_NE(data.localpos.y, position_yaw.y);
  ASSERT_NE(data.localpos.z, position_yaw.z);
  ASSERT_NE(data.rpydata.z, position_yaw.yaw);
}

///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
