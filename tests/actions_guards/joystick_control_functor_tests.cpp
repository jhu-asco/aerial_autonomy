#include <aerial_autonomy/actions_guards/joystick_control_states_actions.h>
#include <aerial_autonomy/joystick_control_events.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <gtest/gtest.h>
#include <aerial_autonomy/sensors/base_sensor.h>
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


TEST(JoystickControlTests, Constructor)
{
  ASSERT_NO_THROW(new jcsa::JoystickControlAction());
  ASSERT_NO_THROW(new jcsa::JoystickControlState());
}

TEST(JoystickControlTests, TransitionAction){
  Sensor<VelocityYaw> velocity_sensor;
  QuadSimulator drone_hardware;
  UAVSensorSystem uav_system(velocity_sensor, drone_hardware);
  UAVSensorLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::JoystickControlAction transition_action_functor;
  int dummy_start_state, dummy_target_state;
  transition_action_functor(jce::JoystickControlEvent(), sample_logic_state_machine,
    dummy_start_state, dummy_target_state);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}