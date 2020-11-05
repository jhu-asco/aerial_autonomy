#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/position_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/controller_connectors/relative_pose_visual_servoing_controller_drone_connector.h>
#include <aerial_autonomy/controller_connectors/orange_picking_reference_connector.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/object_id.h>
#include <aerial_autonomy/types/reset_event.h>
#include "aerial_autonomy/types/sensor_status.h"
#include <chrono>
#include <thread>
#include <glog/logging.h>
#include <parsernode/common.h>

//Forward Declarations
template <class LogicStateMachineT> class PathFollowState_;

//Functors

template <class LogicStateMachineT, class AbortEventT, bool CompleteFlagT = true>
struct PathFollowingStatus_ : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    bool result =
        ControllerStatusInternalActionFunctor_<LogicStateMachineT,
                                               OrangePickingReferenceConnector,
                                               CompleteFlagT, AbortEventT>()
            .run(robot_system, logic_state_machine);
    result &= ControllerStatusInternalActionFunctor_<
                  LogicStateMachineT,
                  RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                  CompleteFlagT, AbortEventT>()
                  .run(robot_system, logic_state_machine);
    if (robot_system.getPathSensor()->getSensorStatus() !=
        SensorStatus::VALID) {
      logic_state_machine.process_event(be::Abort());
      return false;
    }
    return result;
  }
};

template <class LogicStateMachineT>
struct ResetPathFollowingTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system) {
    // Empty for now.
  }
};

template <class LogicStateMachineT>
struct PathFollowingTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system) {
    auto goal = robot_system.getPathSensor();
    robot_system.setGoal<OrangePickingReferenceConnector,SensorPtr<PathReturnT>>(goal);
    //return true;
  }
};

template <class LogicStateMachineT>
struct PathFollowingTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    bool result = true;
    auto goal = robot_system.getPathSensor();
    if (goal != nullptr) {
      result &= (goal->getSensorStatus() == SensorStatus::VALID);
    } else {
      std::cout << "path sensor is NULL" << std::endl;
      return false;
    }
    result &= (robot_system.getPoseSensorStatus() == SensorStatus::VALID);
    std::cout << "path sensor guard returning: " << result << std::endl;
    return result;
  }
};

template <class LogicStateMachineT>
using PathFollowInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        FlyawayCheckFunctor_<LogicStateMachineT>,
        PathFollowingStatus_<LogicStateMachineT, be::Abort>>>;


//New State Definitions
/**
* @brief State that checks that the path is being followed
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PathFollowState_
    : public BaseState<UAVArmSystem, LogicStateMachineT,
          PathFollowInternalActionFunctor_<LogicStateMachineT>> {};
