#include "rpyt_based_velocity_controller_config.pb.h"
#include <aerial_autonomy/actions_guards/joystick_control_states_actions.h>
#include <aerial_autonomy/joystick_control_events.h>
#include <aerial_autonomy/sensors/base_sensor.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <typeindex>

/**
* @brief Namespace for UAV Simulator
*/
using namespace quad_simulator;

/**
* @brief Namespace for Joystick states and actions
* for UAVLogicStateMachine
*/
using jcsa = JoystickControlStatesActions<UAVSensorLogicStateMachine>;

/**
* @brief Namesapce for joystick control events
*/
namespace jce = joystick_control_events;

TEST(JoystickControlTests, Constructor) {
  ASSERT_NO_THROW(new jcsa::JoystickControlAction());
  ASSERT_NO_THROW(new jcsa::JoystickControlState());
}


TEST(JoystickControlTests, TransitionGuardTest) {
  Sensor<VelocityYaw> velocity_sensor;
  QuadSimulator drone_hardware;
  RPYTBasedVelocityControllerConfig config;
  UAVSensorSystem uav_system(velocity_sensor, drone_hardware, config);
  UAVSensorLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::JoystickControlGuard transition_guard_functor;
  int dummy_start_state, dummy_target_state;

  bool result = transition_guard_functor(jce::JoystickControlEvent(),
                            sample_logic_state_machine, dummy_start_state,
                            dummy_target_state);

  ASSERT_FALSE(result);

  velocity_sensor.setSensorStatus(SensorStatus::VALID);

  result = transition_guard_functor(jce::JoystickControlEvent(),
                            sample_logic_state_machine, dummy_start_state,
                            dummy_target_state);

  ASSERT_TRUE(result);
}

TEST(JoystickControlTests, TransitionActionTest) {
  Sensor<VelocityYaw> velocity_sensor;
  QuadSimulator drone_hardware;
  RPYTBasedVelocityControllerConfig config;
  UAVSensorSystem uav_system(velocity_sensor, drone_hardware, config);
  UAVSensorLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::JoystickControlAction transition_action_functor;
  int dummy_start_state, dummy_target_state;

  transition_action_functor(jce::JoystickControlEvent(),
                            sample_logic_state_machine, dummy_start_state,
                            dummy_target_state);
  uav_system.runActiveController(HardwareType::UAV);

  ASSERT_EQ(uav_system.getStatus<JoystickVelocityControllerDroneConnector>(),
    ControllerStatus::Completed);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}