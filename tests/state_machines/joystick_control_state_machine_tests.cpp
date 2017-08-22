#include <aerial_autonomy/state_machines/joystick_control_state_machine.h>
#include <aerial_autonomy/joystick_control_events.h>
#include <gtest/gtest.h>
// Thread stuff
#include <boost/optional/optional_io.hpp>
#include <boost/thread/thread.hpp>
// Quad Simulator
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/**
* @brief  Namespace for joystick control events
*/
namespace jce = joystick_control_events;
namespace be = uav_basic_events;

class JoystickControlStateMachineTests : public ::testing::Test{
public:
  JoystickControlStateMachineTests(){
    drone_hardware.setTakeoffAltitude(2.0);
    uav_system.reset(new UAVSystem(drone_hardware));
    logic_state_machine.reset(new JoystickControlStateMachine(boost::ref(*uav_system)));
    logic_state_machine->start();
    // Will switch to Landed state from manual control state
    logic_state_machine->process_event(InternalTransitionEvent());
  }
protected:
  std::unique_ptr<JoystickControlStateMachine> logic_state_machine;
  std::unique_ptr<UAVSystem> uav_system;
  QuadSimulator drone_hardware;

  void GoToHoverFromLanded() {
    drone_hardware.setBatteryPercent(100);
    logic_state_machine->process_event(be::Takeoff());
    logic_state_machine->process_event(InternalTransitionEvent());
  }  
};

TEST_F(JoystickControlStateMachineTests, InitialState){
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

TEST_F(JoystickControlStateMachineTests, GoToJoystickControl){
   // First takeoff
  GoToHoverFromLanded();
  // Initialize Joystick Control
  logic_state_machine->process_event(jce::JoystickControlEvent());
  // Check controller status
  ASSERT_EQ(uav_system->getStatus<ManualVelocityControllerDroneConnector>(),
    ControllerStatus::Active);
  // Check that current state is Joyctick control state
  ASSERT_STREQ(pstate(*logic_state_machine), "JoystickControlState");
}

TEST_F(JoystickControlStateMachineTests, Abort){
     // First takeoff
  GoToHoverFromLanded();
  // Initialize Joystick Control
  logic_state_machine->process_event(jce::JoystickControlEvent());
  // Check that current state is Joyctick control state
  ASSERT_STREQ(pstate(*logic_state_machine), "JoystickControlState");
  logic_state_machine->process_event(be::Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(JoystickControlStateMachineTests, ManualControlMode){
  // First takeoff
  GoToHoverFromLanded();
  // Initialize Joystick Control
  logic_state_machine->process_event(jce::JoystickControlEvent());
  // Check that current state is Joyctick control state
  ASSERT_STREQ(pstate(*logic_state_machine), "JoystickControlState");
  // Disable SDK
  drone_hardware.flowControl(false);
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlState");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
