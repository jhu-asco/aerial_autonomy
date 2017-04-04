#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_vision_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <glog/logging.h>
#include <parsernode/common.h>

/**
* @brief
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    VLOG(1) << "Selecting home location";
    robot_system.setHomeLocation();

    Position tracking_vector;
    if (!robot_system.getTrackingVector(tracking_vector)) {
      LOG(WARNING) << "Lost tracking while servoing.";
      logic_state_machine.process_event(be::Abort());
    }
    VLOG(1) << "Setting tracking vector";
    double desired_distance = robot_system.getConfiguration()
                                  .uav_vision_system_config()
                                  .desired_visual_servoing_distance();
    robot_system.setGoal<VisualServoingControllerDroneConnector, Position>(
        tracking_vector * desired_distance / tracking_vector.norm());
  }
};

/**
* @brief
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingAbortActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system, LogicStateMachineT &) {
    LOG(WARNING) << "Aborting visual servoing controller";
    robot_system.abortController(HardwareType::UAV);
  }
};

/**
* @brief Action to reach a pre designated point
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct GoHomeTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system, LogicStateMachineT &) {
    PositionYaw home_location = robot_system.getHomeLocation();
    VLOG(1) << "Going home";
    robot_system.setGoal<PositionControllerDroneConnector, PositionYaw>(
        home_location);
  }
};

template <class LogicStateMachineT>
struct GoHomeTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system, LogicStateMachineT &) {
    return robot_system.isHomeLocationSpecified();
  }
};

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  /**
  * @brief check if we reached VS goal and trigger completed event
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  virtual void run(UAVVisionSystem &robot_system,
                   LogicStateMachineT &logic_state_machine) {
    Position goal =
        robot_system
            .getGoal<VisualServoingControllerDroneConnector, Position>();
    Position tracking_vector;
    // Check if tracking is still valid and get current trackng vector
    if (!robot_system.getTrackingVector(tracking_vector)) {
      LOG(WARNING) << "Lost tracking while servoing.";
      logic_state_machine.process_event(be::Abort());
    }
    /// \todo Matt/Gowtham Instead of re-implementing controller logic, add a
    /// function to the robot system to check if the current controller has
    /// converged or has failed
    // Get current yaw
    parsernode::common::quaddata data = robot_system.getUAVData();
    double yaw = data.rpydata.z;
    double error_yaw = math::angleWrap(std::atan2(goal.x, goal.y) - yaw);
    // Define tolerance and check if reached goal
    const auto &robot_config = robot_system.getConfiguration();
    const double &tolerance_pos = robot_config.goal_position_tolerance(); // m
    const double &tolerance_yaw = robot_config.goal_yaw_tolerance();      // m
    if (data.batterypercent < robot_config.minimum_battery_percent()) {
      LOG(WARNING) << "Battery too low " << data.batterypercent
                   << "\% Landing!";
      logic_state_machine.process_event(be::Land());
    } else if ((tracking_vector - goal).norm() < tolerance_pos &&
               std::abs(error_yaw) < tolerance_yaw) {
      VLOG(1) << "Reached goal";
      logic_state_machine.process_event(Completed());
    }
  }
};

template <class LogicStateMachineT>
struct VisualServoingTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system_, LogicStateMachineT &) {
    Position tracking_vector;
    return robot_system_.getTrackingVector(tracking_vector);
  }
};

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using VisualServoing_ =
    BaseState<UAVVisionSystem, LogicStateMachineT,
              VisualServoingInternalActionFunctor_<LogicStateMachineT>>;
