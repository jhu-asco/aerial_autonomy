#include <aerial_autonomy/state_machines/visual_servoing_state_machine.h>
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

class VisualServoingStateMachineTests : public ::testing::Test {
protected:
  std::unique_ptr<VisualServoingStateMachine> logic_state_machine;
  std::unique_ptr<UAVSystem> uav_system;
  QuadSimulator drone_hardware;

  virtual void SetUp() {
    drone_hardware.setTakeoffAltitude(2.0);
    uav_system.reset(new UAVSystem(drone_hardware));
    logic_state_machine.reset(
        new VisualServoingStateMachine(boost::ref(*uav_system)));
    logic_state_machine->start();
  }

  virtual void TearDown() {
    logic_state_machine->stop();
    uav_system.reset();
    logic_state_machine.reset();
  }

  void GoToHoverFromLanded() {
    drone_hardware.setBatteryPercent(100);
    logic_state_machine->process_event(Takeoff());
    logic_state_machine->process_event(InternalTransitionEvent());
  }
};

TEST_F(VisualServoingStateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
