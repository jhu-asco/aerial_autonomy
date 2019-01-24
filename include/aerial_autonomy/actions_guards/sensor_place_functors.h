#pragma once
#include "grip_config.pb.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/logic_states/timed_state.h>
#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/object_id.h>
#include <aerial_autonomy/types/polynomial_reference_trajectory.h>
#include <aerial_autonomy/types/reset_event.h>
#include <Eigen>
#include <aerial_autonomy/commmon/conversions.h>
#include <chrono>
#include <glog/logging.h>
#include <thread>

// Forward Declarations
template <class LogicStateMachineT> class PlaceState_;
template <class LogicStateMachineT> class CheckingState_;

//New Functors

// Check the normal acceleration bias in the horizontal plane
// Send complete event if theshold is reached.
template <class LogicStateMachineT, bool placingFlag>
struct NormalForceThresholdInternalActionFunctor_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  bool run(UAVSystem &robot_system, LogicStateMachineT & logic_state_machine) {
    //Get the acceleration bias (in body frame)
    Eigen::Vector3d bias_acc = robot_system.getAccelerationBias();
    tf::Vector3 bias_acc_tf = tf::Vector3(bias_acc(0),bias_acc(1),bias_acc(2));
    //Transform into the yaw-compensated gravity-aligned frame (denoted as local frame)
    Eigen::Vector3d curr_rpy =  conversions::transformTfToRPY(robot_system.getPose());
    tf::Transform rotation_world_to_body;
    conversions::transformRPYToTf(curr_rpy(0),curr_rpy(1),curr_rpy(2),&rotation_world_to_body);
    tf::Transform rotation_world_to_local;
    conversions::transformRPYToTf(0,0,curr_rpy(2),&rotation_world_to_local);
    tf::Vector3 bias_acc_local_tf = rotation_world_to_local * (rotation_world_to_body.inverse() * bias_acc_tf);
    //Take the x value of the vector, which, if we're normal to the wall, is the normal direction.
    //To be more precise, we should use the relative yaw to mix the x and y components.
    double normal_acc = bias_acc_local_tf.x();
    if (placingFlag) {
      double threshold = logic_state_machine.base_state_machine_config_.visual_servoing_state_machine_config()
                           .pick_place_state_machine_config().placing_acc_threshold(); 
      if (normal_acc_ > threshold) {
        logic_state_machine.process_event(Completed());
        return false;
      }
    } else {
      double threshold = logic_state_machine.base_state_machine_config_.visual_servoing_state_machine_config()
                           .pick_place_state_machine_config().checking_acc_threshold();
      if (normal_acc_ < threshold) {
        logic_state_machine.process_event(Completed());
        return false;
      }
    }
  }
};

// Exactly the same as the VisualServoingStatus_ functor, but no complete events are sent.
template <class LogicStateMachineT, class AbortEventT>
struct NoCompleteVisualServoingStatus_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    // Check config for which connector to use
    auto connector_type = logic_state_machine.base_state_machine_config_
                              .visual_servoing_state_machine_config()
                              .connector_type();
    bool result = false;
    switch (connector_type) {
    case VisualServoingStateMachineConfig::RPYTPose:
      result = ControllerStatusInternalActionFunctor_<
                   LogicStateMachineT, RPYTRelativePoseVisualServoingConnector,
                   false, AbortEventT>()
                   .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::RPYTRef:
      result =
          ControllerStatusInternalActionFunctor_<
              LogicStateMachineT,
              RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
              false, AbortEventT>()
              .run(robot_system, logic_state_machine);
      result &= ControllerStatusInternalActionFunctor_<
                    LogicStateMachineT,
                    UAVVisionSystem::RPYTVisualServoingReferenceConnectorT,
                    false, AbortEventT>()
                    .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::MPC:
      result =
          ControllerStatusInternalActionFunctor_<LogicStateMachineT,
                                                 MPCControllerQuadConnector,
                                                 false, AbortEventT>()
              .run(robot_system, logic_state_machine);
      result &= ControllerStatusInternalActionFunctor_<
                    LogicStateMachineT,
                    UAVVisionSystem::MPCVisualServoingReferenceConnectorT,
                    false, AbortEventT>()
                    .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::HeadingDepth:
      result = ControllerStatusInternalActionFunctor_<
                   LogicStateMachineT, VisualServoingControllerDroneConnector,
                   false, AbortEventT>()
                   .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::VelPose:
      result = ControllerStatusInternalActionFunctor_<
                   LogicStateMachineT,
                   RelativePoseVisualServoingControllerDroneConnector, true,
                   AbortEventT>()
                   .run(robot_system, logic_state_machine);
      break;
    }
    return result;
  }
};

// Internal action functor to send a reset event if the state has lasted longer than the grip_timeout()
template <class LogicStateMachineT, class StateT> 
struct TimeoutInternalActionFunctor_
    : public StateDependentInternalActionFunctor<
          UAVArmSystem, LogicStateMachineT, StateT> { 
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
           StateT &state) { 
    std::chrono::milliseconds threshold = std::chrono::milliseconds(
                                            logic_state_machine.base_state_machine_config_
                                            .visual_servoing_state_machine_config()
                                            .pick_place_state_machine_config()
                                            .grip_config().grip_timeout());
    if (state.timeInState() > threshold) {
      logic_state_machine.process_event(Reset());
    }
  }
}; 

/**
* @brief Logic for going to the pre-place position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PrePlaceInternalActionFunctor_ = boost::msm::front::ShortingActionSequence_<
    boost::mpl::vector<UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                       ArmStatusInternalActionFunctor_<LogicStateMachineT>,
                       VisualServoingStatus_<LogicStateMachineT, Reset>>>;


/**
* @brief State for going to the placement staging position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PrePlaceState_ = BaseState<UAVArmSystem, LogicStateMachineT, 
                        PrePlaceInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief Logic to check while placing object
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PlaceInternalActionFunctor_ = boost::msm::front::ShortingActionSequence_<
    boost::mpl::vector<UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                       ArmStatusInternalActionFunctor_<LogicStateMachineT>,
                       NoCompleteVisualServoingStatus_<LogicStateMachineT, Abort>,
                       NormalForceThresholdInternalActionFunctor_<LogicStateMachineT, true>,
                       TimeoutInternalActionFunctor_<LogicStateMachineT, PlaceState_<LogicStateMachineT>>>>;
/**
* @brief State that uses visual servoing to place object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PlaceState_ = TimedState<UAVArmSystem, LogicStateMachineT,
                              PlaceInternalActionFunctor_<LogicStateMachineT>>;
/**
* @brief Logic to check if an object has been placed
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using CheckingInternalActionFunctor_ = boost::msm::front::ShortingActionSequence_<
    boost::mpl::vector<UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                       ArmStatusInternalActionFunctor_<LogicStateMachineT>,
                       NoCompleteVisualServoingStatus_<LogicStateMachineT, Abort>,
                       NormalForceThresholdInternalActionFunctor_<LogicStateMachineT, false>,
                       TimeoutInternalActionFunctor_<LogicStateMachineT, CheckingState_<LogicStateMachineT>>>>;
/**
* @brief State that checks that an object has been placed.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using CheckingState_ = TimedState<UAVArmSystem, LogicStateMachineT,
                              CheckingInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief Logic for going to the post-place position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PostPlaceInternalActionFunctor_ = boost::msm::front::ShortingActionSequence_<
    boost::mpl::vector<UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                       ArmStatusInternalActionFunctor_<LogicStateMachineT>,
                       VisualServoingStatus_<LogicStateMachineT, Reset>>>;


/**
* @brief State for going to the placement staging position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PostPlaceState_ = BaseState<UAVArmSystem, LogicStateMachineT, 
                        PostPlaceInternalActionFunctor_<LogicStateMachineT>>;
