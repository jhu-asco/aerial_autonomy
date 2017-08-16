#pragma once
// Html utils
#include <aerial_autonomy/common/html_utils.h>
// Ostream
#include <iostream>

/**
* @brief Status of the controller
*/
class ControllerStatus {
public:
  enum Status {
    Active,    ///< Controller active
    Completed, ///< Controller completed
    Critical,  ///< Controller is critical and unable to continue
    NotEngaged ///< This status is used when no controller is engaged
  };

private:
  Status status_;           ///< Current status
  std::string description_; ///< Any description such as error metrics
                            /// associated with controller status

  friend bool operator==(const ControllerStatus &controller_status,
                         const Status &status_enum);
  friend bool operator==(const ControllerStatus &controller_status,
                         const ControllerStatus &controller_status_2);

public:
  ControllerStatus(Status status = Status::NotEngaged,
                   std::string description = "");

  std::string getStatus();
};
