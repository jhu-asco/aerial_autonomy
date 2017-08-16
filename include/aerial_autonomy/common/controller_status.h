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
  /**
   * @brief Types of status available for a controller
   */
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

  /**
   * @brief An overload for comparing against an enum
   *
   * @param controller_status The LHS which is a controller status
   * @param status_enum  The RHS which is an enum
   *
   * @return true if the controller status has the same enum as RHS
   */
  friend bool operator==(const ControllerStatus &controller_status,
                         const Status &status_enum);
  /**
   * @brief An overload function to compare two controller status
   * classes
   *
   * @param lhs_controller_status first controller status
   * @param rhs_controller_status second controller status
   *
   * @return true if both controller status have the same status enum stored
   */
  friend bool operator==(const ControllerStatus &lhs_controller_status,
                         const ControllerStatus &rhs_controller_status);

public:
  /**
   * @brief Constructor
   *
   * @param status      store the status enum
   * @param description text message describing the status
   */
  ControllerStatus(Status status = Status::NotEngaged,
                   std::string description = "");

  /**
   * @brief Get a Html text describing the controller status
   *
   * @return html string containing controller status and description
   */
  std::string getStatus();
};
