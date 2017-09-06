#include "uav_system_config.pb.h"
#include <aerial_autonomy/robot_systems/uav_sensor_system.h>
#include <aerial_autonomy/sensors/guidance.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <chrono>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/// \brief Test UAV Sensor System
TEST(UAVSensorSystemTests, Constructor) {
  UAVSystemConfig uav_system_config;
  QuadSimulator drone_hardware;
  Guidance sensor(drone_hardware);
  Atomic<RPYTBasedVelocityControllerConfig> rpyt_config;
  ASSERT_NO_THROW(new UAVSensorSystem(sensor, drone_hardware, uav_system_config,
                                      rpyt_config, 0.02));
}

TEST(UAVSensorSystemTests, runManualVelocityController) {
  UAVSystemConfig uav_system_config;
  QuadSimulator drone_hardware;
  Guidance sensor(drone_hardware);
  RPYTBasedVelocityControllerConfig rpyt_config_;
  rpyt_config_.set_kp(2.0);
  rpyt_config_.set_ki(0.01);

  auto vel_ctlr_config = rpyt_config_.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
  tolerance->set_vx(1e-4);
  tolerance->set_vy(1e-4);
  tolerance->set_vz(1e-4);

  Atomic<RPYTBasedVelocityControllerConfig> rpyt_config;
  rpyt_config = rpyt_config_;

  UAVSensorSystem uav_system(sensor, drone_hardware, uav_system_config,
                             rpyt_config, 0.02);
  uav_system.takeOff();

  // set rc channels
  int16_t channels[4] = {1000, 1000, -1000, 1000};
  drone_hardware.setRC(channels);
  drone_hardware.set_delay_send_time(0.02);
  // set goal
  uav_system.setGoal<JoystickVelocityControllerDroneConnector>(EmptyGoal());

  ASSERT_EQ(uav_system.getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::Active);

  // run controller till it converges
  while (uav_system.getActiveControllerStatus(HardwareType::UAV) ==
         ControllerStatus::Active) {
    uav_system.runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  };

  // Verify controller status is completed
  ASSERT_TRUE(bool(uav_system.getActiveControllerStatus(HardwareType::UAV)));

  VelocityYaw vel_goal(0.1, 0.1, -0.1,
                       -0.1 *
                           uav_system_config.uav_sensor_system_config()
                               .joystick_velocity_controller_config()
                               .max_yaw_rate() *
                           0.02);
  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.linvel.x, vel_goal.x, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.y, vel_goal.y, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.z, vel_goal.z, 1e-3);
  ASSERT_NEAR(sensor_data.rpydata.z, vel_goal.yaw, 1e-3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}