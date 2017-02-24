#pragma once

#include <aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h>
// Controllers
#include <aerial_autonomy/controllers/basic_controllers.h>
// Specific ControllerConnectors
#include <aerial_autonomy/controller_hardware_connectors/basic_controller_hardware_connectors.h>
// Store type_map
#include <aerial_autonomy/common/type_map.h>
// Boost thread stuff
#include <boost/thread/mutex.hpp>
// Unique ptr
#include <memory>

/**
 * Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin position, velocity, and rpy controllers for controlling UAV
*/
class UAVSystem {

private:
  // Hardware:
  parsernode::Parser &drone_hardware_;
  // Controllers
  BuiltInController<PositionYaw> builtin_position_controller_;
  BuiltInController<VelocityYaw> builtin_velocity_controller_;
  ManualRPYTController manual_rpyt_controller_;
  // Controller connectors
  PositionControllerDroneConnector position_controller_drone_connector_;
  BuiltInVelocityControllerDroneConnector velocity_controller_drone_connector_;
  ManualRPYTControllerDroneConnector rpyt_controller_drone_connector_;
  // Container to store and retrieve controller-hardware-connectors
  TypeMap<AbstractControllerHardwareConnector>
      controller_hardware_connector_container_;
  // Maps to store and swap active controller
  std::map<HardwareType, AbstractControllerHardwareConnector *>
      active_controllers_;
  std::map<HardwareType, std::unique_ptr<boost::mutex>> thread_mutexes_;

public:
  UAVSystem(parsernode::Parser &drone_hardware)
      : drone_hardware_(drone_hardware),
        position_controller_drone_connector_(drone_hardware,
                                             builtin_position_controller_),
        velocity_controller_drone_connector_(drone_hardware,
                                             builtin_velocity_controller_),
        rpyt_controller_drone_connector_(drone_hardware,
                                         manual_rpyt_controller_) {
    // Add control hardware connector containers
    controller_hardware_connector_container_.addObject(
        position_controller_drone_connector_);
    controller_hardware_connector_container_.addObject(
        velocity_controller_drone_connector_);
    controller_hardware_connector_container_.addObject(
        rpyt_controller_drone_connector_);

    // Initialize active controller map
    active_controllers_[HardwareType::Arm] = nullptr;
    thread_mutexes_[HardwareType::Arm] =
        std::unique_ptr<boost::mutex>(new boost::mutex);

    active_controllers_[HardwareType::UAV] = nullptr;
    thread_mutexes_[HardwareType::UAV] =
        std::unique_ptr<boost::mutex>(new boost::mutex);
  }
  // Get sensor data from UAV:
  parsernode::common::quaddata getUAVData() {
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);
    return data;
  }

  // Calls to set goals to controllers and send commands to
  // hardwareAbstractControllerHardwareConnector
  void takeOff() { drone_hardware_.takeoff(); }

  void land() { drone_hardware_.land(); }

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
  template <class ControllerHardwareConnectorT, class GoalT> GoalT getGoal() {
    ControllerHardwareConnectorT *controller_hardware_connector =
        controller_hardware_connector_container_
            .getObject<ControllerHardwareConnectorT>();
    return controller_hardware_connector->getGoal();
  }

  void abortController(HardwareType hardware_type) {
    boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
    active_controllers_[hardware_type] = nullptr;
  }

  void runActiveController(HardwareType hardware_type) {
    // lock to ensure active_control fcn is not changed
    AbstractControllerHardwareConnector *active_controller =
        active_controllers_[hardware_type];
    if (active_controller != nullptr) {
      boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
      active_controller->run();
    }
  }
};
