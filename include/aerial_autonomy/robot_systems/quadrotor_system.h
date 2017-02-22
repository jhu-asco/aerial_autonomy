#pragma once

#include <aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h>
// Position Control Stuff
#include <aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h>
#include <aerial_autonomy/controllers/builtin_controller.h>
// store controller hardware connectors
#include <unordered_map>
// get type info for each controller hardware connector class
#include <typeindex>
// Boost thread stuff
#include <boost/thread/mutex.hpp>
// Unique ptr
#include <memory>

/**
 * Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin position, velocity, and rpy controllers for controlling quadrotor
*/
class QuadRotorSystem {
  // May be use shared pointers
  class ControllerHardwareConnectorContainer {
    // Map to store the controller to hardware connectors
    std::unordered_map<std::type_index, AbstractControllerHardwareConnector* > controller_hardware_connector_map_;
  public:
    // Set
    template<class ControllerHardwareConnectorT>
    void addControllerHardwareConnector(ControllerHardwareConnectorT &controller_hardware_connector) {
      controller_hardware_connector_map_[typeid(controller_hardware_connector)] = &controller_hardware_connector;
    }
    // Get
    template<class ControllerHardwareConnectorT>
    ControllerHardwareConnectorT* getControllerHardwareConnector() {
      return static_cast<ControllerHardwareConnectorT>(controller_hardware_connector_map_[typeid(ControllerHardwareConnectorT)]);
    }
  };

private:
  // Hardware:
  parsernode::Parser &drone_hardware_;
  // Controllers
  BuiltInController<PositionYaw> builtin_position_controller_;
  // Controller connectors
  PositionControllerDroneConnector position_controller_drone_connector_;
  // Container to store and retrieve controller-hardware-connectors
  ControllerHardwareConnectorContainer controller_hardware_connector_container_;
  // Maps to store and swap active controller
  std::map<HardwareType, AbstractControllerHardwareConnector*> active_controllers_;
  std::map<HardwareType, std::unique_ptr<boost::mutex> > thread_mutexes_;
public:
  QuadRotorSystem(parsernode::Parser &drone_hardware):
    drone_hardware_(drone_hardware),
    position_controller_drone_connector_(drone_hardware, builtin_position_controller_)
  {
    // Add control hardware connector containers
    controller_hardware_connector_container_.addControllerHardwareConnector(position_controller_drone_connector_);
    // Initialize active controller map
    active_controllers_[HardwareType::Arm] = nullptr;
    thread_mutexes_[HardwareType::Arm] = std::unique_ptr<boost::mutex>(new boost::mutex);

    active_controllers_[HardwareType::Quadrotor] = nullptr;
    thread_mutexes_[HardwareType::Quadrotor] = std::unique_ptr<boost::mutex>(new boost::mutex);
  }
  // Get sensor data from quadrotor:
  parsernode::common::quaddata getQuadData() {
      parsernode::common::quaddata data;
      drone_hardware_.getquaddata(data);
      return data;
  }

  // Calls to set goals to controllers and send commands to hardwareAbstractControllerHardwareConnector
  void takeOff() {
    drone_hardware_.takeoff();
  }

  void land() {
    drone_hardware_.land();
  }

  template <class ControllerHardwareContainerT, class GoalT>
  void setGoal(GoalT goal) {
    ControllerHardwareContainerT *controller_hardware_connector = controller_hardware_connector_container_.getControllerHardwareConnector<ControllerHardwareContainerT>();
    controller_hardware_connector->setGoal(goal);
    HardwareType hardware_type = controller_hardware_connector->getHardwareType();
    if(active_controllers_[hardware_type] != controller_hardware_connector) {
      boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
      active_controllers_[hardware_type] = controller_hardware_connector;
    }
  }
  template <class ControllerHardwareContainerT, class GoalT>
  GoalT getGoal() {
    ControllerHardwareContainerT *controller_hardware_connector = controller_hardware_connector_container_.getControllerHardwareConnector<ControllerHardwareContainerT>();
    return controller_hardware_connector->getGoal();
  }

  void abortController(HardwareType hardware_type) {
    boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
    active_controllers_[hardware_type] = nullptr;
  }

  void activeControllerRun(HardwareType hardware_type) {
    // lock to ensure active_control fcn is not changed
    AbstractControllerHardwareConnector *active_controller = active_controllers_[hardware_type];
    if(active_controller != nullptr) {
      boost::mutex::scoped_lock lock(*thread_mutexes_[hardware_type]);
      active_controller->run();
    }
  }
};
