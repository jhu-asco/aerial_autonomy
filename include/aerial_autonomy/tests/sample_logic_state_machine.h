#pragma once
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/robot_systems/uav_vision_system.h>
#include <aerial_autonomy/state_machines/base_state_machine.h>
#include <aerial_autonomy/types/position_yaw.h>
#include <aerial_autonomy/types/velocity_yaw.h>
#include <type_traits>

/**
* @brief Robot system that does not perform any actions
*/
struct EmptyRobotSystem {};

/**
* @brief Example Logic state machine that stores triggered event
*
* @tparam RobotSystemT The robot system that is used by states to perform
* actions
*/
template <class RobotSystemT>
class SampleLogicStateMachine_ : public BaseStateMachine<RobotSystemT> {
  /**
  * @brief type index of the event triggered by LSM
  */
  std::type_index type_index_event_ = typeid(NULL);
  /**
  * @brief Positionyaw message thats triggered
  */
  PositionYaw pose_event_;

  /**
   * @brief Velocityyaw message that was triggered
   */
  VelocityYaw velocity_event_;

public:
  /**
   * @brief Constructor that takes robot system
   *
   * @param robot_system provides actions to select controllers, get/set Goals.
   */
  SampleLogicStateMachine_(
      RobotSystemT &robot_system,
      const BaseStateMachineConfig &base_state_machine_config)
      : BaseStateMachine<RobotSystemT>(robot_system,
                                       base_state_machine_config) {}

  /**
   * @brief Constructor that uses a default state machine config
   *
   * @param robot_system provides actions to select controllers, get/set Goals.
   */
  SampleLogicStateMachine_(RobotSystemT &robot_system)
      : SampleLogicStateMachine_(robot_system, BaseStateMachineConfig()) {}

  /**
  * @brief Generic process event that stores the event type
  *
  * @tparam Event Event type
  * @param event Event instance sent in to process
  */
  template <class Event> void process_event(const Event &event) {
    type_index_event_ = typeid(event);
  }

  /**
  * @brief process pose event message
  *
  * @param event message that is stored
  */
  void process_event(const PositionYaw &event) {
    type_index_event_ = typeid(event);
    pose_event_ = event;
  }

  /**
   * @brief special implementation for processing velocity
   * yaw message
   *
   * @param event velocity yaw command sent to state machine
   */
  void process_event(const VelocityYaw &event) {
    type_index_event_ = typeid(event);
    velocity_event_ = event;
  }

  /**
  * @brief retrieve the event type_index of last processed event
  *
  * @return  type_index of the last processed event
  */
  std::type_index getProcessEventTypeId() { return type_index_event_; }
  /**
  * @brief retrieve poseyaw event message
  *
  * @return poseyaw event message
  */
  PositionYaw getPoseEvent() { return pose_event_; }

  /**
   * @brief get the received velocity yaw event
   *
   * @return velocityyaw event message
   */
  VelocityYaw getVelocityEvent() { return velocity_event_; }
};

/**
* @brief Sample logic state machine templated on empty robot system
*/
using SampleLogicStateMachine = SampleLogicStateMachine_<EmptyRobotSystem>;
/**
* @brief Sample logic state machine templated on UAV system
*/
using UAVLogicStateMachine = SampleLogicStateMachine_<UAVSystem>;
/**
* @brief Sample logic state machine templated on UAV system
*/
using UAVVisionLogicStateMachine = SampleLogicStateMachine_<UAVVisionSystem>;
