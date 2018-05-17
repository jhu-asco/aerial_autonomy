#pragma once
#include "aerial_autonomy/robot_systems/base_robot_system.h"

/**
* @brief A helper robot system to test controller connectors
*/
struct SampleRobotSystem : public BaseRobotSystem {
  /**
  * @brief Helper function to add external controller connectors
  * to internal typemap
  *
  * @tparam ControllerConnectorT The type of controller connector to add
  * @param connector The connector object to add
  */
  template <class ControllerConnectorT>
  void addControllerConnector(ControllerConnectorT &connector) {
    controller_connector_container_.setObject(connector);
  }

  /**
  * @brief The system status
  *
  * @return Empty string
  */
  std::string getSystemStatus() const { return ""; }
};
