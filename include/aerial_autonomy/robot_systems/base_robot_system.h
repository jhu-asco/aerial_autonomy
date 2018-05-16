#pragma once

#include <aerial_autonomy/controller_connectors/base_controller_connector.h>
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
   * @brief Container to store and retrieve controller-connectors
   */
  TypeMap<AbstractControllerConnector> controller_connector_container_;

private:
  /**
  * @brief Map to store active controller based on controller group
  */
  std::map<ControllerGroup, AbstractControllerConnector *> active_controllers_;
  /**
  * @brief Map to lock and swap the active controller for a given
  * controller group
  */
  std::map<ControllerGroup, std::unique_ptr<boost::mutex>> thread_mutexes_;

public:
  /**
  * @brief Constructor to initialize active controllers and mutexes to NULL
  */
  BaseRobotSystem() {
    // Initialize active controller map
    for (auto controller_group : IterableEnum<ControllerGroup>()) {
      active_controllers_.emplace(controller_group, nullptr);
      thread_mutexes_[controller_group] =
          std::unique_ptr<boost::mutex>(new boost::mutex);
    }
  }

  void activateControllerConnector(
      AbstractControllerConnector *controller_connector) {
    ControllerGroup controller_group =
        controller_connector->getControllerGroup();
    if (active_controllers_[controller_group] != controller_connector) {
      boost::mutex::scoped_lock lock(*thread_mutexes_[controller_group]);
      active_controllers_[controller_group] = controller_connector;
    }
    AbstractControllerConnector *dependent_connector =
        controller_connector->getDependentConnector();
    if (dependent_connector != nullptr) {
      activateControllerConnector(dependent_connector);
    }
  }

  /**
  * @brief sets goal to the connector and swaps the active
  * connector with the specified connector type.
  *
  * @tparam ControllerConnectorT type of connector to use
  * @tparam GoalT Type of Goal to set to connector
  * @param goal Goal to set to connector
  */
  template <class ControllerConnectorT, class GoalT> void setGoal(GoalT goal) {
    ControllerConnectorT *controller_connector =
        controller_connector_container_.getObject<ControllerConnectorT>();
    controller_connector->setGoal(goal);
    activateControllerConnector(controller_connector);
    runActiveController(controller_connector->getControllerGroup());
  }
  /**
  * @brief Get the goal from connector.
  *
  * @tparam ControllerConnectorT Type of connector to use
  * @tparam GoalT Type of Goal to get
  *
  * @return goal of GoalT type
  */
  template <class ControllerConnectorT, class GoalT> GoalT getGoal() const {
    const ControllerConnectorT *controller_connector =
        controller_connector_container_.getObject<ControllerConnectorT>();
    return controller_connector->getGoal();
  }

  /**
  * @brief Get the status of a controller
  *
  * @tparam ControllerConnectorT Type of connector
  *
  * @return Status is active/completed/critical
  */
  template <class ControllerConnectorT> ControllerStatus getStatus() const {
    const ControllerConnectorT *controller_connector =
        controller_connector_container_.getObject<ControllerConnectorT>();
    auto active_controller =
        active_controllers_.find(controller_connector->getControllerGroup());
    if (active_controller == active_controllers_.end() ||
        controller_connector != active_controller->second) {
      return ControllerStatus::NotEngaged;
    }
    return controller_connector->getStatus();
  }

  /**
  * @brief Get the status of the active controller
  *
  * @param controller_group  controller group to get controller for
  *
  * @return status of the active controller.
  * If no active controller returns status as not engaged
  */
  ControllerStatus
  getActiveControllerStatus(ControllerGroup controller_group) const {
    auto active_controller = active_controllers_.find(controller_group);
    if (active_controller != active_controllers_.end() &&
        active_controller->second != nullptr) {
      return active_controller->second->getStatus();
    } else {
      return ControllerStatus(ControllerStatus::NotEngaged);
    }
  }

  /**
  * @brief Remove active controller for given controller group
  *
  * @param controller_group group for which active controller is
  * switched off
  */
  void abortController(ControllerGroup controller_group) {
    boost::mutex::scoped_lock lock(*thread_mutexes_[controller_group]);
    active_controllers_[controller_group] = nullptr;
  }

  /**
  * @brief Run active controller stored for a given controller group
  *
  * @param controller_group group for which active controller is run
  */
  void runActiveController(ControllerGroup controller_group) {
    // lock to ensure active_control fcn is not changed
    AbstractControllerConnector *const active_controller =
        active_controllers_[controller_group];
    if (active_controller != nullptr) {
      boost::mutex::scoped_lock lock(*thread_mutexes_[controller_group]);
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
