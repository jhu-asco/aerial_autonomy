#include <aerial_autonomy/robot_systems/uav_sensor_system.h>
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
  QuadSimulator drone_hardware;
  Sensor<VelocityYaw> sensor;
  RPYTBasedVelocityControllerConfig rpyt_config;

  ASSERT_NO_THROW(new UAVSensorSystem(sensor, 
    drone_hardware,
    rpyt_config));
}

TEST(UAVSystemTests, runManualVelocityController) {
  QuadSimulator drone_hardware;
  Sensor<VelocityYaw> sensor;
  RPYTBasedVelocityControllerConfig rpyt_config;

  UAVSensorSystem uav_system(sensor,
   drone_hardware,
   rpyt_config);
  uav_system.takeOff();
  
  // set rc channels
  int16_t channels[4] = {100, 100, -100, 0};
  drone_hardware.setRC(channels);
  drone_hardware.set_delay_send_time(0.02);
  // set goal
  uav_system.setGoal<JoystickVelocityControllerDroneConnector>(EmptyGoal());
  // Run 10 iterations
  for (int i = 0; i < 100; ++i) {
    uav_system.runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.servo_in[0] , channels[0] , 1e-4);
  ASSERT_NEAR(sensor_data.servo_in[1] , channels[1] , 1e-4);
  ASSERT_NEAR(sensor_data.servo_in[2] , channels[2] , 1e-4);
  ASSERT_NEAR(sensor_data.servo_in[3] , channels[3] , 1e-4);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}