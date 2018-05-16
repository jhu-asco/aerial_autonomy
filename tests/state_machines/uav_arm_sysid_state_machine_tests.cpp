#include <aerial_autonomy/state_machines/uav_arm_sysid_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>
// Thread stuff
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <boost/optional/optional_io.hpp>
#include <boost/thread/thread.hpp>
// Quad Simulator
#include <quad_simulator_parser/quad_simulator.h>
// Arm simulator
#include <arm_parsers/arm_simulator.h>
#include <vector>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;
/**
* @brief Namespace for arm events
*/
namespace ae = arm_events;

/**
 * @brief Namespace for uav arm events
 */
namespace ue = uav_arm_sysid_events;

class StateMachineTests : public ::testing::Test {
protected:
  std::unique_ptr<UAVArmSysIDStateMachine> logic_state_machine;
  std::unique_ptr<UAVArmSystem> uav_arm_system;
  std::shared_ptr<QuadSimulator> drone_hardware;
  std::shared_ptr<ArmSimulator> arm_hardware;
  std::shared_ptr<SimpleTracker> tracker_;
  UAVSystemConfig config;
  BaseStateMachineConfig state_machine_config;

  virtual void SetUp() {
    drone_hardware.reset(new QuadSimulator);
    drone_hardware->usePerfectTime();
    arm_hardware.reset(new ArmSimulator);
    auto position_tolerance = config.mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(0.5);
    position_tolerance->set_y(0.5);
    position_tolerance->set_z(0.5);

    auto vel_based_pos_controller_config =
        config.mutable_rpyt_based_position_controller_config()
            ->mutable_velocity_based_position_controller_config();
    auto vel_based_pos_controller_tol =
        vel_based_pos_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vel_based_pos_controller_tol->set_x(0.1);
    vel_based_pos_controller_tol->set_y(0.1);
    vel_based_pos_controller_tol->set_z(0.1);
    auto rpyt_vel_controller_tol =
        config.mutable_rpyt_based_position_controller_config()
            ->mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    rpyt_vel_controller_tol->set_vx(0.1);
    rpyt_vel_controller_tol->set_vy(0.1);
    rpyt_vel_controller_tol->set_vz(0.1);

    drone_hardware->setTakeoffAltitude(2.0);
    tracker_.reset(
        new SimpleTracker(*drone_hardware, tf::Transform::getIdentity()));
    uav_arm_system.reset(new UAVArmSystem(
        config, tracker_,
        std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware),
        std::dynamic_pointer_cast<ArmSimulator>(arm_hardware)));
    logic_state_machine.reset(new UAVArmSysIDStateMachine(
        boost::ref(*uav_arm_system), boost::cref(state_machine_config)));
    logic_state_machine->start();
    // Disarm quad to begin with
    drone_hardware->disarm();
    logic_state_machine->process_event(InternalTransitionEvent());

    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
  }

  virtual void TearDown() {
    logic_state_machine->stop();
    uav_arm_system.reset();
    logic_state_machine.reset();
  }

  void GoToHover() {
    logic_state_machine->process_event(ae::PowerOn());
    logic_state_machine->process_event(ae::Fold());
    drone_hardware->setBatteryPercent(100);
    drone_hardware->takeoff();
    // Manual control should automatically switch to hovering
    logic_state_machine->process_event(InternalTransitionEvent());
    logic_state_machine->process_event(InternalTransitionEvent());
  }
};

TEST_F(StateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlArmState");
}

TEST_F(StateMachineTests, SwitchToLand) {
  // Enable quad
  drone_hardware->flowControl(true);
  drone_hardware->setBatteryPercent(100);
  // We are in landed state, and we give Land command
  logic_state_machine->process_event(be::Land());
  // No Transition so we are still in landed state
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

/// \brief Test Takeoff related events
TEST_F(StateMachineTests, LowBatteryTakeoff) {
  drone_hardware->flowControl(true);
  drone_hardware->setBatteryPercent(10);
  logic_state_machine->process_event(be::Takeoff());
  // Cannot takeoff
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlArmState");
}

TEST_F(StateMachineTests, Takeoff) {
  logic_state_machine->process_event(ae::PowerOn());
  logic_state_machine->process_event(ae::Fold());
  drone_hardware->setBatteryPercent(100);
  drone_hardware->takeoff();
  // Manual control should automatically switch to hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

///
/// \brief Test Joystick Control related events
TEST_F(StateMachineTests, JoystickControl) {
  // First takeoff
  GoToHover();
  // Takeoff value is roughly 1400 so 2000 should make it go higher
  std::vector<int16_t> rc_values({0, 0, 2000, 0});
  // Set joystick value for uav
  drone_hardware->setRCInputsWithoutMapping(rc_values);
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  auto altitude_check = [&]() {
    uav_arm_system->runActiveController(ControllerGroup::UAV);
    auto quad_data = uav_arm_system->getUAVData();
    // Takeoff altitude is 2.0
    return (quad_data.localpos.z > 3.0);
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(
      altitude_check, std::chrono::seconds(1), std::chrono::milliseconds(0)));
}

TEST_F(StateMachineTests, JoystickControlAbort) {
  // First takeoff
  GoToHover();
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  // Abort goal
  logic_state_machine->process_event(be::Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Check controller is actually aborted
  ASSERT_EQ(uav_arm_system->getActiveControllerStatus(ControllerGroup::UAV),
            ControllerStatus::NotEngaged);
}

TEST_F(StateMachineTests, JoystickControlManualControlAbort) {
  // First takeoff
  GoToHover();
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  // Disable sdk
  drone_hardware->flowControl(false);
  // Run Internal Transition
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlArmState");
  // Check controller is actually aborted
  ASSERT_EQ(uav_arm_system->getActiveControllerStatus(ControllerGroup::UAV),
            ControllerStatus::NotEngaged);
}

TEST_F(StateMachineTests, JoystickControlLowBattery) {
  // First takeoff
  GoToHover();
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  // Low battery while reaching goal
  drone_hardware->setBatteryPercent(10);
  // Run Internal Transition
  logic_state_machine->process_event(InternalTransitionEvent());
  // Check if we are aborting due to low battery
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Check controller is actually aborted
  ASSERT_EQ(uav_arm_system->getActiveControllerStatus(ControllerGroup::UAV),
            ControllerStatus::NotEngaged);
}
//\todo Gowtham Add tests specific to arm sine controller transitions

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
