#pragma once

#include <Eigen/Dense>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/joystick_control_events.h>
#include <gcop/qrotorsystemid.h>
#include <gcop/so3.h>

namespace jce = joystick_control_events;
/**
* @brief Action while transitioning to system id state
*
* @tparam Logic State Machine to process events
*/
template <class LogicStateMachineT>
struct SystemIdStateTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    VLOG(1) << "Entering system id mode\n";
    robot_system.setGoal<ManualRPYTControllerDroneConnector>(EmptyGoal());
    parsernode::common::quaddata data = robot_system.getUAVData();
    robot_system.setLastCommandedYaw(data.rpydata.z, true);
    VLOG(1) << "Set Yaw to " << data.rpydata.z << "\n";
  }
};
/**
* @brief Guard to check if Sensor status is valid before
* transitioning
*/
template <class LogicStateMachineT>
struct SystemIdStateTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system) {
    if (!bool(robot_system.getVelocityPoseSensorStatus())) {
      LOG(WARNING) << "Sensor Status INVALID";
    }
    return bool(robot_system.getVelocityPoseSensorStatus());
  }
};
/**
* @brief Internal action functor for system Id state
* stores the rpyt controls and trajectory for system ID
*
* @tparam LogicStateMachineT Logic State Machine used to process events
*/
template <class LogicStateMachineT, class AbortT = ManualControlEvent>
struct SystemIdStateInternalActionFunctor_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief Stores the rpyt and position measurements
  *
  * @param robot_system to get sensor data
  * @param logic_state_machine LSM used to process events
  */
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();

    // TODO soham make seperate class for estimators and add this there
    gcop::QRotorSystemIDMeasurement measurement;
    ManualRPYTControllerConfig rpyt_config =
        robot_system.getConfiguration().manual_rpyt_controller_config();

    std::tuple<VelocityYaw, Position> sensor_data =
        robot_system.getVelocityPoseSensorData();
    Position position_data = std::get<1>(sensor_data);
    measurement.t = data.timestamp;
    measurement.position << position_data.x, position_data.y, position_data.z;
    measurement.rpy << data.rpydata.x, data.rpydata.y, data.rpydata.z;

    double roll = math::map(data.servo_in[0], -rpyt_config.max_channel1(),
                            rpyt_config.max_channel1(), -rpyt_config.max_roll(),
                            rpyt_config.max_roll());

    double pitch = math::map(data.servo_in[1], -rpyt_config.max_channel2(),
                             rpyt_config.max_channel2(),
                             -rpyt_config.max_pitch(), rpyt_config.max_pitch());

    double thrust =
        math::map(data.servo_in[2], -rpyt_config.max_channel3(),
                  rpyt_config.max_channel3(), rpyt_config.min_thrust(),
                  rpyt_config.max_thrust());

    double yawdot =
        math::map(-data.servo_in[3], -rpyt_config.max_channel4(),
                  rpyt_config.max_channel4(), -rpyt_config.max_yaw_rate(),
                  rpyt_config.max_yaw_rate());

    measurement.control << thrust, roll, pitch, yawdot;
    robot_system.addMeasurement(measurement);
    return true;
  }
};

/**
* @brief Internal action while in systemid state.
*
* @tparam LogicStateMachineT Logic State Machine used to process events
*/
template <class LogicStateMachineT>
using SystemIdStateCompleteActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        SystemIdStateInternalActionFunctor_<LogicStateMachineT>>>;
/**
* @brief System Id state
*
* @tparam LogicStateMachineT Logic State Machine to process events
*/
template <class LogicStateMachineT>
struct SystemIdState_
    : public BaseState<
          UAVSystem, LogicStateMachineT,
          SystemIdStateInternalActionFunctor_<LogicStateMachineT>> {
  /**
  * @brief Function to run system id when exiting from this state
  */
  template <class Event, class FSM>
  void on_exit(Event const &, FSM &logic_state_machine) {
    VLOG(1) << "Exiting system id state. Estimating Parameters";
    UAVSystem &robot_system = this->getRobotSystem(logic_state_machine);
    robot_system.runSystemId();
  }
};
