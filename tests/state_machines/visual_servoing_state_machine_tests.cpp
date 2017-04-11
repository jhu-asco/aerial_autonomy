#include <aerial_autonomy/state_machines/visual_servoing_state_machine.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
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
public:
  VisualServoingStateMachineTests()
      : tracker(drone_hardware, config.uav_vision_system_config()),
        uav_system(new UAVVisionSystem(tracker, drone_hardware, config)),
        logic_state_machine(
            new VisualServoingStateMachine(boost::ref(*uav_system))) {
    tracker.setTargetPositionGlobalFrame(Position(5, 0, 0));
    drone_hardware.setTakeoffAltitude(2.0);
    logic_state_machine->start();
  }

  ~VisualServoingStateMachineTests() {
    logic_state_machine->stop();
    uav_system.reset();
    logic_state_machine.reset();
  }

protected:
  SimpleTracker tracker;
  QuadSimulator drone_hardware;
  UAVSystemConfig config;
  std::unique_ptr<UAVVisionSystem> uav_system;
  std::unique_ptr<VisualServoingStateMachine> logic_state_machine;

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
