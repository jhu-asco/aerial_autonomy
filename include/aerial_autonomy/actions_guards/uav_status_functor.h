#pragma once

#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/uav_basic_events.h>

/**
* @brief Functor which internally checks for UAV status
* before calling status independent run function
*
* @tparam RobotSystemT Robot System used by status independent run
* @tparam LogicStateMachineT State Machine used by status independent run
*/
template <class RobotSystemT, class LogicStateMachineT>
struct UAVStatusActionFunctor
    : EventAgnosticActionFunctor<RobotSystemT, LogicStateMachineT> {
  /**
  * @brief Check the status using UAV data. The status is basically checking for
  * low voltage and if manual mode is on
  *
  * @param robot_system UAV system to get sensor data
  * @param logic_state_machine Backend to process events based on data
  *
  * @return true if the checks on sensor data pass
  */
  bool checkStatus(UAVSystem &robot_system,
                   LogicStateMachineT &logic_state_machine) {
    bool status = true;
    parsernode::common::quaddata data = robot_system.getUAVData();
    const auto &robot_config = robot_system.getConfiguration();
    if (!data.rc_sdk_control_switch) {
      LOG(WARNING) << "Aborting Controller due to sdk being closed";
      logic_state_machine.process_event(be::Abort());
      status = false;
    } else if (data.batterypercent < robot_config.minimum_battery_percent()) {
      LOG(WARNING) << "Battery too low " << data.batterypercent
                   << "\% Landing!";
      logic_state_machine.process_event(be::Land());
      status = false;
    }
    return status;
  }

  /**
  * @brief Run function that is independent of uav status. Since uav status is
  * already checked by the run function in this class
  *
  * @param robot_system Provides sensor data and allows for controlling
  * hardware. Should be convertible to UAVSystem.
  * @param logic_state_machine Backend of logic State Machine. can send events
  * using this.
  */
  virtual void
  statusIndependentRun(RobotSystemT &robot_system,
                       LogicStateMachineT &logic_state_machine) = 0;

  void run(RobotSystemT &robot_system,
           LogicStateMachineT &logic_state_machine) final {
    if (checkStatus(robot_system, logic_state_machine)) {
      statusIndependentRun(robot_system, logic_state_machine);
    }
  }
};
