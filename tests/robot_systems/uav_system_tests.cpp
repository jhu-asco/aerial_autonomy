#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/sensors/guidance.h>
//#include <aerial_autonomy/tests/sample_parser.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/tests/test_utils.h>
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

  ManualRPYTControllerConfig config_ =
      uav_system.getConfiguration().manual_rpyt_controller_config();
  RollPitchYawThrust rpyt;
  rpyt.r =
      math::map(channels[0], -config_.max_channel1(), config_.max_channel1(),
                -config_.max_roll(), config_.max_roll());
  rpyt.p =
      math::map(channels[1], -config_.max_channel2(), config_.max_channel2(),
                -config_.max_pitch(), config_.max_pitch());
  rpyt.t =
      math::map(channels[2], -config_.max_channel3(), config_.max_channel3(),
                config_.min_thrust(), config_.max_thrust());

  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.rpydata.x, rpyt.r, 1e-4);
  ASSERT_NEAR(sensor_data.rpydata.y, rpyt.p, 1e-4);
  // Verify Omegaz
  ASSERT_NEAR(sensor_data.omega.z, -0.00314, 1e-3);
}

TEST(UAVSystemTests, getActiveControllerStatus) {
  QuadSimulator drone_hardware;
  UAVSystemConfig config;
  auto position_tolerance = config.mutable_position_controller_config()
                                ->mutable_goal_position_tolerance();
  position_tolerance->set_x(0.5);
  position_tolerance->set_y(0.5);
  position_tolerance->set_z(0.5);
  UAVSystem uav_system(drone_hardware, config);
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

TEST(UAVSystemTests, ExplicitConstructor) {
  QuadSimulator drone_hardware;
  ASSERT_NO_THROW(
      new UAVSystem(drone_hardware, UAVSystemConfig(),
                    std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
                        new Guidance(drone_hardware)),
                    0.02));
}

TEST(UAVSystemTests, runJoystickVelocityController) {
  UAVSystemConfig uav_system_config;
  QuadSimulator drone_hardware;

  RPYTBasedVelocityControllerConfig rpyt_config;

  UAVSystem uav_system(
      drone_hardware, uav_system_config,
      std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
          new Guidance(drone_hardware)),
      0.02);

  RPYTBasedVelocityControllerConfig rpyt_config_;
  rpyt_config_.set_kp(2.0);
  rpyt_config_.set_ki(0.01);

  auto vel_ctlr_config = rpyt_config_.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
  tolerance->set_vx(1e-4);
  tolerance->set_vy(1e-4);
  tolerance->set_vz(1e-4);

  uav_system.updateRPYTVelocityControllerConfig(rpyt_config_);
  uav_system.takeOff();

  // set rc channels
  int16_t channels[4] = {1000, 1000, -1000, 0};
  drone_hardware.setRC(channels);
  drone_hardware.set_delay_send_time(0.02);
  // set goal
  uav_system.setGoal<JoystickVelocityControllerDroneConnector>(EmptyGoal());

  ASSERT_EQ(uav_system.getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::Active);

  auto controller_run = [&]() {
    uav_system.runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return (bool(uav_system.getActiveControllerStatus(HardwareType::UAV)));
  };

  ASSERT_TRUE(test_utils::waitUntilTrue()(
      controller_run, std::chrono::seconds(50), std::chrono::milliseconds(10)));

  VelocityYaw vel_goal(0.1, 0.1, -0.1, 0);

  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.linvel.x, vel_goal.x, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.y, vel_goal.y, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.z, vel_goal.z, 1e-3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
