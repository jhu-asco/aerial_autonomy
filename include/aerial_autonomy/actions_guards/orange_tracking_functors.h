#pragma once
#include "grip_config.pb.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/actions_guards/position_control_functors.h>
#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/logic_states/timed_state.h>
#include <aerial_autonomy/orange_tracking_events.h>
#include <aerial_autonomy/robot_systems/uav_vision_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/object_id.h>
#include <aerial_autonomy/types/polynomial_reference_trajectory.h>
#include <aerial_autonomy/types/reset_event.h>
#include <aerial_autonomy/sensors/bool_sensor.h>
#include <chrono>
#include <glog/logging.h>
#include <tf_conversions/tf_eigen.h>
#include <thread>

// Forward Declarations
template <class LogicStateMachineT> class PreOrangeTrackingState_;
template <class LogicStateMachineT> class OrangeTrackingState_;
template <class LogicStateMachineT> class ResetOrangeTracking_;


// Internal action functor to send a reset event if the state has lasted longer
// than the pick_timeout()
template <class LogicStateMachineT, class StateT>
struct TimeoutInternalActionFunctor_
    : public StateDependentInternalActionFunctor<UAVVisionSystem,
                                                 LogicStateMachineT, StateT> {
  bool run(UAVVisionSystem &robot_system, LogicStateMachineT &logic_state_machine,
           StateT &state) {
    std::chrono::milliseconds threshold =
        std::chrono::milliseconds(logic_state_machine.base_state_machine_config_
                                      .visual_servoing_state_machine_config()
                                      .orange_tracking_state_machine_config()
                                      .pick_timeout());
    if (state.timeInState() > threshold) {
      logic_state_machine.process_event(Reset());
      VLOG(1) << "Timout Reached: Resetting";
      return false;
    }
    return true;
  }
};
// Internal action functor to send a complete event if the success sensor triggers
template <class LogicStateMachineT>
struct SuccessSensorInternalActionFunctor_
    : public InternalActionFunctor<UAVSystem,LogicStateMachineT> {
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    SensorPtr<bool> success_sensor_ = robot_system.getSuccessSensor();
    if (!success_sensor_){
       VLOG(1) << "Using Null Ptr in SuccessSensorInternalActionFunctor";
       return false;
    }
    if (success_sensor_->getSensorStatus() != SensorStatus::VALID){
      VLOG(1) << "Success Sensor Timed Out";
      logic_state_machine.process_event(be::Abort());
      return false;
    }
    if (success_sensor_->getSensorData()){
      VLOG(1) << "Success Sensor Returned True!";
      logic_state_machine.process_event(Completed());
      return false;
    }
    return true;
  }
};


/**
* @brief Logic for going to the pre-place position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PreOrangeTrackingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        FlyawayCheckFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, be::Abort>>>;

/**
* @brief State for going to the picking staging position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PreOrangeTrackingState_
    : public BaseState<
          UAVVisionSystem, LogicStateMachineT,
          PreOrangeTrackingInternalActionFunctor_<LogicStateMachineT>> {};

/**
* @brief Logic to check while picking the orange
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using OrangeTrackingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        FlyawayCheckFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, Reset, false>,
        SuccessSensorInternalActionFunctor_<LogicStateMachineT>,
        TimeoutInternalActionFunctor_<LogicStateMachineT,OrangeTrackingState_<LogicStateMachineT>>>>;
/**
* @brief State that uses visual servoing to place object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class OrangeTrackingState_
    : public TimedState<UAVVisionSystem, LogicStateMachineT,
                        OrangeTrackingInternalActionFunctor_<LogicStateMachineT>> {};
