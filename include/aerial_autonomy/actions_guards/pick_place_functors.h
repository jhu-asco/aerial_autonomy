#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/uav_status_functor.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <glog/logging.h>
#include <thread>

/**
* @brief Logic to grab an object, sleep for few seconds
* Abort UAV, Arm controllers
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickGuard_
    : EventAgnosticGuardFunctor<UAVArmSystem, LogicStateMachineT> {
  bool guard(UAVArmSystem &robot_system,
             LogicStateMachineT &logic_state_machine) {
    VLOG(1) << "Grip Object";
    robot_system.grip(true);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(robot_system.gripTimeout()));
    if (!robot_system.getCommandStatus()) {
      LOG(WARNING) << "Failed to grip object by timeout!";
      robot_system.grip(false);
      return false;
    }
    VLOG(1) << "Done Gripping!";
    return true;
  }
};

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickInternalActionFunctor_
    : UAVStatusActionFunctor<UAVArmSystem, LogicStateMachineT> {
  /**
  * @brief check if quad and arm reached VS goal and
  * trigger completed event
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void statusIndependentRun(UAVArmSystem &robot_system,
                            LogicStateMachineT &logic_state_machine) {
    // Check arm status before proceeding
    if (!robot_system.enabled()) {
      LOG(WARNING) << "Arm not enabled!";
      logic_state_machine.process_event(be::Abort());
      return;
    }
    ControllerStatus uav_status =
        robot_system.getStatus<VisualServoingControllerDroneConnector>();
    ControllerStatus arm_status =
        robot_system.getStatus<VisualServoingControllerArmConnector>();
    if (uav_status == ControllerStatus::Completed &&
        arm_status == ControllerStatus::Completed) {
      VLOG(1) << "Reached goal for UAV and arm";
      logic_state_machine.process_event(Completed());
    } else if (uav_status == ControllerStatus::Critical) {
      LOG(WARNING) << "Lost tracking while servoing. Aborting!";
      logic_state_machine.process_event(be::Abort());
    } else if (arm_status == ControllerStatus::Critical) {
      LOG(WARNING) << "Arm status critical. Aborting!";
      logic_state_machine.process_event(be::Abort());
    }
  }
};

/**
* @brief Logic to check arm power and manual mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/

template <class LogicStateMachineT>
struct ManualControlArmInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  /**
  * @brief The parent functor to switch to other states based on state of
  * rc switch
  */
  ManualControlInternalActionFunctor_<LogicStateMachineT> parent_functor_;
  /**
  * @brief Check if arm is enabled and print warning otherwise. Also call
  * parent functor for checking rc switch
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void run(UAVArmSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    if (!robot_system.enabled()) {
      LOG(WARNING) << "Arm not enabled! Enabling arm now.";
      robot_system.power(true); // Try to enable arm
    } else {
      // Since arm is enabled we can switch to other states
      parent_functor_.run(robot_system, logic_state_machine);
    }
  }
};

template <class LogicStateMachineT>
struct VisualServoingArmTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    VLOG(1) << "Setting Goal for visual servoing arm connector!";
    robot_system.setGoal<VisualServoingControllerArmConnector, tf::Transform>(
        robot_system.armGoalTransform());
    // Also ensure the gripper is open before going to pick objects
    robot_system.grip(false);
  }
};
/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PickState_ = BaseState<UAVArmSystem, LogicStateMachineT,
                             PickInternalActionFunctor_<LogicStateMachineT>>;
