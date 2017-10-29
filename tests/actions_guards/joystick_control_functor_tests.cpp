#include <aerial_autonomy/actions_guards/joystick_control_states_actions.h>
#include <aerial_autonomy/joystick_control_events.h>
#include <aerial_autonomy/sensors/guidance.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator
*/
using namespace quad_simulator;

/**
* @brief Namespace for Joystick states and actions
* for UAVLogicStateMachine
*/
using jcsa = JoystickControlStatesActions<UAVLogicStateMachine>;

/**
* @brief Namesapce for joystick control events
*/
namespace jce = joystick_control_events;

TEST(JoystickControlTests, Constructor) {
  ASSERT_NO_THROW(new jcsa::JoystickControlAction());
  ASSERT_NO_THROW(new jcsa::JoystickControlState());
  ASSERT_NO_THROW(new jcsa::SystemIdState());
}

TEST(JoystickControlTests, TransitionGuardValidTest) {

  QuadSimulator drone_hardware;
  UAVSystem uav_system(
      drone_hardware, UAVSystemConfig(),
      std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
          new Guidance(drone_hardware)));
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::JoystickControlGuard transition_guard_functor;
  int dummy_start_state, dummy_target_state;

  bool result = transition_guard_functor(jce::JoystickControlEvent(),
                                         sample_logic_state_machine,
                                         dummy_start_state, dummy_target_state);

  ASSERT_TRUE(result);
}

TEST(JoystickControlTests, TransitionActionTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(
      drone_hardware, UAVSystemConfig(),
      std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
          new Guidance(drone_hardware)));

  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::JoystickControlAction transition_action_functor;
  int dummy_start_state, dummy_target_state;

  transition_action_functor(jce::JoystickControlEvent(),
                            sample_logic_state_machine, dummy_start_state,
                            dummy_target_state);
  uav_system.runActiveController(HardwareType::UAV);

  ASSERT_EQ(uav_system.getStatus<JoystickVelocityControllerDroneConnector>(),
            ControllerStatus::Active);
}

TEST(JoystickControlTests, ControllerRunTest) {
  QuadSimulator drone_hardware;
  UAVSystemConfig uav_system_config;
  UAVSystem uav_system(
      drone_hardware, uav_system_config,
      std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
          new Guidance(drone_hardware)));

  RPYTBasedVelocityControllerConfig rpyt_config_;
  rpyt_config_.set_kp(2.0);
  rpyt_config_.set_ki(0.01);

  auto vel_ctlr_config = rpyt_config_.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
  tolerance->set_vx(1e-4);
  tolerance->set_vy(1e-4);
  tolerance->set_vz(1e-4);

  uav_system.updateRPYTVelocityControllerConfig(rpyt_config_);

  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::JoystickControlAction transition_action_functor;
  int dummy_start_state, dummy_target_state;

  transition_action_functor(jce::JoystickControlEvent(),
                            sample_logic_state_machine, dummy_start_state,
                            dummy_target_state);
  // set rc channels
  int16_t channels[4] = {1000, 1000, -1000, 1000};
  drone_hardware.setRC(channels);
  drone_hardware.set_delay_send_time(0.02);

  uav_system.runActiveController(HardwareType::UAV);

  ASSERT_EQ(uav_system.getStatus<JoystickVelocityControllerDroneConnector>(),
            ControllerStatus::Active);

  // run controller till it converges
  auto controller_run = [&]() {
    uav_system.runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return bool(
        uav_system.getStatus<JoystickVelocityControllerDroneConnector>());
  };

  ASSERT_TRUE(test_utils::waitUntilTrue()(
      controller_run, std::chrono::seconds(20), std::chrono::milliseconds(10)));

  VelocityYaw vel_goal(
      0.1, 0.1, -0.1,
      -0.1 *
          uav_system_config.joystick_velocity_controller_config()
              .max_yaw_rate() *
          0.02);

  parsernode::common::quaddata sensor_data = uav_system.getUAVData();
  ASSERT_NEAR(sensor_data.linvel.x, vel_goal.x, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.y, vel_goal.y, 1e-3);
  ASSERT_NEAR(sensor_data.linvel.z, vel_goal.z, 1e-3);
  ASSERT_NEAR(sensor_data.omega.z,
              -0.1 *
                  uav_system_config.joystick_velocity_controller_config()
                      .max_yaw_rate(),
              2e-2);
}

TEST(JoystickControlTests, TransitionGuardInvalidTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(
      drone_hardware, UAVSystemConfig(),
      std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
          new Sensor<std::tuple<VelocityYaw, Position>>()));
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::JoystickControlGuard transition_guard_functor;
  int dummy_start_state, dummy_target_state;

  bool result = transition_guard_functor(jce::JoystickControlEvent(),
                                         sample_logic_state_machine,
                                         dummy_start_state, dummy_target_state);

  ASSERT_FALSE(result);
}

TEST(JoystickControlTests, SystemIdActionTest) {
  QuadSimulator drone_hardware;
  UAVSystem uav_system(
      drone_hardware, UAVSystemConfig(),
      std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
          new Sensor<std::tuple<VelocityYaw, Position>>()));
  UAVLogicStateMachine sample_logic_state_machine(uav_system);
  jcsa::SystemIdStateAction systemid_state_action;
  int dummy_event, dummy_target_state, dummy_start_state;
  systemid_state_action(dummy_event, sample_logic_state_machine,
                        dummy_start_state, dummy_target_state);

  ASSERT_EQ(uav_system.getStatus<ManualRPYTControllerDroneConnector>(),
            ControllerStatus::Active);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}