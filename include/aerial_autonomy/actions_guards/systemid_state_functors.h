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
    std::cout << "Entering system id mode\n";
    robot_system.setGoal<ManualRPYTControllerDroneConnector>(EmptyGoal());
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

    gcop::QRotorSystemIDMeasurement measurement;
    measurement.position << data.localpos.x, data.localpos.y, data.localpos.z;
    measurement.rpy << data.rpydata.x, data.rpydata.y, data.rpydata.z;
    measurement.control << math::map(data.servo_in[0], -10000, 10000, -M_PI / 6,
                                     M_PI / 6),
        math::map(data.servo_in[1], -10000, 10000, -M_PI / 6, M_PI / 6),
        math::map(data.servo_in[2], -10000, 10000, 10, 1000),
        math::angleWrap(
            data.rpydata.z -
            math::map(data.servo_in[0], -10000, 10000, -M_PI / 6, M_PI / 6) *
                0.02);

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
using SystemIdState_ =
    BaseState<UAVSystem, LogicStateMachineT,
              SystemIdStateCompleteActionFunctor_<LogicStateMachineT>>;
