#pragma once
#include "grip_config.pb.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/actions_guards/arm_functors.h>
#include <aerial_autonomy/actions_guards/position_control_functors.h>
#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/logic_states/timed_state.h>
#include <aerial_autonomy/orange_tracking_events.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
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
template <class LogicStateMachineT> class ResetOrangeTrackingState_;
template <class LogicStateMachineT> class ResetTrialState_;
template <class LogicStateMachineT> class OrangeTrackingFinalRiseState_;
template <class LogicStateMachineT> class OrangeGrippingState_;


/**
* @brief Transition action to perform when going into position control mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT, int GoalIndex>
struct RelativePositionWaypointTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    tf::StampedTransform start_pose = robot_system.getPose();
    auto state_machine_config = this->state_machine_config_.visual_servoing_state_machine_config();
    auto &reference_config =
        this->state_machine_config_.poly_reference_config();
    PositionYaw start_position_yaw;
    conversions::tfToPositionYaw(start_position_yaw, start_pose);
    if (GoalIndex >= state_machine_config.relative_pose_goals().size()) {
            LOG(ERROR) << "GoalIndex: " << GoalIndex << " Relative Pose goals size: "
                             << state_machine_config.relative_pose_goals().size();
    }
    auto goal = state_machine_config.relative_pose_goals().Get(GoalIndex);
    PositionYaw pyGoal = conversions::protoPositionYawToPositionYaw(goal);
    tf::Transform tfGoal;
    conversions::positionYawToTf(pyGoal, tfGoal);
    tf::Transform goal_pose_world = start_pose * tfGoal;
    PositionYaw goal_world;
    conversions::tfToPositionYaw(goal_world,goal_pose_world);
    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> reference(
        new PolynomialReferenceTrajectory(goal_world, start_position_yaw,
                                          reference_config));
    robot_system
        .setGoal<RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>(
            reference);
  }
};
// Internal action functor to send a reset event if the state has lasted longer
// than the pick_timeout()
template <class LogicStateMachineT, class StateT>
struct TimeoutInternalActionFunctor_
    : public StateDependentInternalActionFunctor<UAVArmSystem,
                                                 LogicStateMachineT, StateT> {
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
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

/**
* @brief Checks whether grip command has completed or timed out
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam StateT State which stores gripper timer state
*/
template <class LogicStateMachineT, bool flag>
struct JawGripInternalActionFunctor_
    : public InternalActionFunctor<UAVArmSystem, LogicStateMachineT> {
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    bool status = robot_system.getCommandStatus();
    if (status) {
      bool has_grip = robot_system.gripStatus();
      if (has_grip == flag) {
        VLOG(1) << "Done Gripping!";
        logic_state_machine.process_event(Completed());
        return false;
      } else {
        VLOG(1) << "Grip Failed!";
        logic_state_machine.process_event(Reset());
      }
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
* @brief Action to ungrip if config says to
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct TrialGripTransitionActionFunctor_
    : EventAgnosticActionFunctor<ArmSystem, LogicStateMachineT> {
  void run(ArmSystem &robot_system) {
    if (this->state_machine_config_
             .visual_servoing_state_machine_config()
	     .orange_tracking_state_machine_config()
	     .ungrip_flag()) {
      VLOG(1) << "UnGripping!";
      while(!robot_system.grip(false)){
        VLOG(1) << "Ungrip Failed.... ";
        usleep(500);
        VLOG(1) << "Retrying";
      }
    }
  }
};





/**
* @brief Action to reach a pre designated point
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ResetTrialTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    PositionYaw home_location = robot_system.getHomeLocation();
    auto &window_config = this->state_machine_config_
                              .visual_servoing_state_machine_config()
                              .orange_tracking_state_machine_config()
                              .window();
    PositionYaw window = conversions::protoPositionYawToPositionYaw(window_config);
    PositionYaw offset = PositionYaw(((2*(((float) std::rand())/RAND_MAX))-1)*window.x,
                                     ((2*(((float) std::rand())/RAND_MAX))-1)*window.y,
                                     ((2*(((float) std::rand())/RAND_MAX))-1)*window.z,
                                     ((2*(((float) std::rand())/RAND_MAX))-1)*window.yaw);
    PositionYaw goal_location = home_location + offset;
    VLOG(1) << "Resetting Trial";
    tf::StampedTransform start_pose = robot_system.getPose();
    auto &reference_config =
        this->state_machine_config_.poly_reference_config();
    PositionYaw start_position_yaw;
    conversions::tfToPositionYaw(start_position_yaw, start_pose);
    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> reference(
        new PolynomialReferenceTrajectory(goal_location, start_position_yaw,
                                          reference_config));
    robot_system
        .setGoal<RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>(
            reference);
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
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        FlyawayCheckFunctor_<LogicStateMachineT>,
        //SuccessSensorInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, Reset>>>;

/**
* @brief State for going to the picking staging position
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PreOrangeTrackingState_
    : public BaseState<
          UAVArmSystem, LogicStateMachineT,
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
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        FlyawayCheckFunctor_<LogicStateMachineT>,
        //SuccessSensorInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, Reset, true>,
        TimeoutInternalActionFunctor_<LogicStateMachineT,OrangeTrackingState_<LogicStateMachineT>>>>;
/**
* @brief State that uses visual servoing to pick object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class OrangeTrackingState_
    : public TimedState<UAVArmSystem, LogicStateMachineT,
                        OrangeTrackingInternalActionFunctor_<LogicStateMachineT>> {};

/**
* @brief Logic to check while picking the orange
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using OrangeTrackingFinalRiseInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        FlyawayCheckFunctor_<LogicStateMachineT>,
        //JawGripInternalActionFunctor_<LogicStateMachineT,true>,
        //SuccessSensorInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<LogicStateMachineT,RPYTBasedReferenceConnector<Eigen::VectorXd,Eigen::VectorXd>, true>,
        TimeoutInternalActionFunctor_<LogicStateMachineT,OrangeTrackingFinalRiseState_<LogicStateMachineT>>>>;

/**
* @brief State that uses visual servoing to pick object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class OrangeTrackingFinalRiseState_
    : public TimedState<UAVArmSystem, LogicStateMachineT,
                        OrangeTrackingFinalRiseInternalActionFunctor_<LogicStateMachineT>> {};
/**
* @brief State that resets to last staging location
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class ResetOrangeTrackingState_
    : public BaseState<UAVArmSystem, LogicStateMachineT,
                        PositionControlInternalActionFunctor_<LogicStateMachineT>> {};
/**
* @brief State that resets to last set home location
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class ResetTrialState_
    : public BaseState<UAVArmSystem, LogicStateMachineT,
                        PositionControlInternalActionFunctor_<LogicStateMachineT>> {};
/**
* @brief Logic to check while picking the orange
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using OrangeGrippingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT,Reset>,
        FlyawayCheckFunctor_<LogicStateMachineT>,
        JawGripInternalActionFunctor_<LogicStateMachineT,true>,
        ControllerStatusInternalActionFunctor_<LogicStateMachineT,RPYTBasedReferenceConnector<Eigen::VectorXd,Eigen::VectorXd>, false>,
        TimeoutInternalActionFunctor_<LogicStateMachineT,OrangeGrippingState_<LogicStateMachineT>>>>;

/**
* @brief State that uses visual servoing to pick object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class OrangeGrippingState_
    : public TimedState<UAVArmSystem, LogicStateMachineT,
                        OrangeGrippingInternalActionFunctor_<LogicStateMachineT>> {};
