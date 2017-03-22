#pragma once

#include <aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h>
// Store type_map
#include <aerial_autonomy/common/type_map.h>
// Iterable Enum
#include <aerial_autonomy/common/iterable_enum.h>
// Boost thread stuff
#include <boost/thread/mutex.hpp>
// Unique ptr
#include <memory>

/**
 * @brief Provides functions to switch between active controllers and get goals
*/
class BaseRobotSystem {

protected:
  /**
   * @brief Container to store and retrieve controller-hardware-connectors
   */
  TypeMap<AbstractControllerHardwareConnector>
      controller_hardware_connector_container_;

private:
  /**
  * @brief Map to store active controller based on hardware type
  */
  std::map<HardwareType, AbstractControllerHardwareConnector *>
      active_controllers_;
  /**
  * @brief Map to lock and swap the active controller for a given
  * hardware type
  */
  std::map<HardwareType, std::unique_ptr<boost::mutex>> thread_mutexes_;

public:
  /**
  * @brief Constructor to initialize active controllers and mutexes to NULL
  */
  BaseRobotSystem() {
    // Initialize active controller map
    for (auto hardware_type : IterableEnum<HardwareType>()) {
      active_controllers_[hardware_type] = nullptr;
      thread_mutexes_[hardware_type] =
          std::unique_ptr<boost::mutex>(new boost::mutex);
    }
  }

  /**
  * @brief sets goal to the connector and swaps the active
  * connector with the specified connector type.
  *
  * @tparam ControllerHardwareConnectorT type of connector to use
  * @tparam GoalT Type of Goal to set to connector
  * @param goal Goal to set to connector
  */
  template <class ControllerHardwareConnectorT, class GoalT>
  void setGoal(GoalT goal) {
    ControllerHardwareConnectorT *controller_hardware_connector =
        controller_hardware_connector_container_
            .getObject<ControllerHardwareConnectorT>();
    controller_hardware_connector->setGoal(goal);
    HardwareType hardware_type =
        controller_hardware_connector->getHardwareType();
    if (active_controllers_[hardware_type] != controller_hardware_connector) {
      boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
      active_controllers_[hardware_type] = controller_hardware_connector;
    }
  }
  /**
  * @brief Get the goal from connector.
  *
  * @tparam ControllerHardwareConnectorT Type of connector to use
  * @tparam GoalT Type of Goal to get
  *
  * @return goal of GoalT type
  */
  template <class ControllerHardwareConnectorT, class GoalT>
  GoalT getGoal() const {
    const ControllerHardwareConnectorT *controller_hardware_connector =
        controller_hardware_connector_container_
            .getObject<ControllerHardwareConnectorT>();
    return controller_hardware_connector->getGoal();
  }

  /**
  * @brief Remove active controller for given hardware type
  *
  * @param hardware_type Type of hardware for which active controller is
  * switched off
  */
  void abortController(HardwareType hardware_type) {
    boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
    active_controllers_[hardware_type] = nullptr;
  }

  /**
  * @brief Run active controller stored for a given hardware type
  *
  * @param hardware_type Type of hardware for which active controller is run
  */
  void runActiveController(HardwareType hardware_type) {
    // lock to ensure active_control fcn is not changed
    AbstractControllerHardwareConnector *active_controller =
        active_controllers_[hardware_type];
    if (active_controller != nullptr) {
      boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
      active_controller->run();
    }
  }

  /**
  * @brief Provide the current state of robot system
  *
  * @return string representation of the robot system state
  */
  virtual std::string getSystemStatus() const = 0;
};
