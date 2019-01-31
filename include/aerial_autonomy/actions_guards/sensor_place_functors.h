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
#include <chrono>
#include <glog/logging.h>
#include <tf_conversions/tf_eigen.h>
#include <thread>

// Forward Declarations
template <class LogicStateMachineT> class SensorPlaceState_;
template <class LogicStateMachineT> class SensorCheckingState_;

// New Functors
// Set thust mixing gain to 0
template <class LogicStateMachineT>
struct ZeroThrustMixingGain_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system_) {
    LOG(INFO) << "Setting thrust mixing gain to zero";
    robot_system_.setThrustMixingGain(0);
  }
};

/**
* @brief Reset the acceleration bias estimator
*/
template <class LogicStateMachineT>
struct ResetAccelerationBiasEstimator_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system_) {
    LOG(INFO) << "Resetting acceleration bias estimator";
    robot_system_.resetAccelerationBiasEstimator();
  }
};

// Check the normal acceleration bias in the horizontal plane
// Send complete event if theshold is reached.
template <class LogicStateMachineT, bool placingFlag>
struct NormalForceThresholdInternalActionFunctor_
    : InternalActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool run(UAVVisionSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    // Get the acceleration bias (in body frame)
    Eigen::Vector3d bias_acc = robot_system.getAccelerationBias();
    tf::Vector3 bias_acc_tf =
        tf::Vector3(bias_acc(0), bias_acc(1), bias_acc(2));
    // Transform into the yaw-compensated gravity-aligned frame (denoted as
    // local frame)
    Eigen::Vector3d curr_rpy =
        conversions::transformTfToRPY(robot_system.getPose());
    tf::Transform rotation_from_body_to_local;
    conversions::transformRPYToTf(curr_rpy(0), curr_rpy(1), 0,
                                  rotation_from_body_to_local);
    tf::Vector3 bias_acc_local_tf = rotation_from_body_to_local * bias_acc_tf;
    // Take the x value of the vector, which, if the robot is oriented normal to
    // the
    // wall, is the normal direction.  If the local x is not aligned with the
    // wall
    // normal vector, the true result would be a combination of the x and y
    // components.
    double normal_acc_ = bias_acc_local_tf.x();
    if (placingFlag) {
      double threshold = logic_state_machine.base_state_machine_config_
                             .visual_servoing_state_machine_config()
                             .sensor_place_state_machine_config()
                             .placing_acc_threshold();
      if (normal_acc_ < threshold) {
        VLOG(1) << "Placing Threshold Exceeded";
        logic_state_machine.process_event(Completed());
        return false;
      }
    } else {
      double threshold = logic_state_machine.base_state_machine_config_
                             .visual_servoing_state_machine_config()
                             .sensor_place_state_machine_config()
                             .checking_acc_threshold();
      if (normal_acc_ > threshold) {
        VLOG(1) << "Checking Threshold Exceeded";
        logic_state_machine.process_event(Completed());
        return false;
      }
    }
    return true;
  }
};

// Internal action functor to send a reset event if the state has lasted longer
// than the grip_timeout()
template <class LogicStateMachineT, class StateT>
struct TimeoutInternalActionFunctor_
    : public StateDependentInternalActionFunctor<UAVArmSystem,
                                                 LogicStateMachineT, StateT> {
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
           StateT &state) {
    std::chrono::milliseconds threshold =
        std::chrono::milliseconds(logic_state_machine.base_state_machine_config_
                                      .visual_servoing_state_machine_config()
                                      .sensor_place_state_machine_config()
                                      .grip_timeout());
    if (state.timeInState() > threshold) {
      logic_state_machine.process_event(Reset());
      VLOG(1) << "Timout Reached: Resetting";
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
using PreSensorPlaceInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, be::Abort>>>;

/**
* @brief State for going to the placement staging position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PreSensorPlaceState_
    : public BaseState<
          UAVArmSystem, LogicStateMachineT,
          PreSensorPlaceInternalActionFunctor_<LogicStateMachineT>> {};

/**
* @brief Logic to check while placing object
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using SensorPlaceInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, be::Abort, false>,
        NormalForceThresholdInternalActionFunctor_<LogicStateMachineT, true>,
        TimeoutInternalActionFunctor_<LogicStateMachineT,
                                      SensorPlaceState_<LogicStateMachineT>>>>;
/**
* @brief State that uses visual servoing to place object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class SensorPlaceState_
    : public TimedState<UAVArmSystem, LogicStateMachineT,
                        SensorPlaceInternalActionFunctor_<LogicStateMachineT>> {
};
/**
* @brief Logic to check if an object has been placed
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using SensorCheckingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, be::Abort, false>,
        NormalForceThresholdInternalActionFunctor_<LogicStateMachineT, false>,
        TimeoutInternalActionFunctor_<
            LogicStateMachineT, SensorCheckingState_<LogicStateMachineT>>>>;
/**
* @brief State that checks that an object has been placed.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class SensorCheckingState_
    : public TimedState<
          UAVArmSystem, LogicStateMachineT,
          SensorCheckingInternalActionFunctor_<LogicStateMachineT>> {};

/**
* @brief Logic for going to the post-place position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PostPlaceInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, be::Abort>>>;

/**
* @brief State for going to the placement staging position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PostPlaceState_
    : public BaseState<UAVArmSystem, LogicStateMachineT,
                       PostPlaceInternalActionFunctor_<LogicStateMachineT>> {};
