#pragma once
// Define strings
#include <string>
// Vector for debug info
#include <vector>
// Html utils
#include <aerial_autonomy/common/html_utils.h>
#include <tuple>

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
  /**
  * @brief Tuple of controller status, description, debug header, debug info
  */
  using DebugInfo =
      std::tuple<Status, std::string, std::string, std::vector<double>>;

private:
  Status status_;                  ///< Current status
  std::string status_description_; ///< Description for status if any
  std::string debug_header_;       ///< Header for debug info

  /**
  * @brief Debug data associated with current status
  */
  std::vector<double> debug_info_;
  /**
   * @brief Debug information from other statuses that gets added when combining
   * multiple status together. The tuple contains, status, status description,
   * debug header and debug info in the specified order.
   */
  std::vector<DebugInfo> additional_debug_info_;

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

  /**
   * @brief An overload function to compare two controller status
   * classes
   * @param lhs_controller_status first controller status
   * @param rhs_controller_status second controller status
   *
   * @return true if both controller status do not have the same status enum
   * stored
   */
  friend bool operator!=(const ControllerStatus &lhs_controller_status,
                         const ControllerStatus &rhs_controller_status);
  /**
   * @brief Add data to controller status
   *
   * @param cs ControllerStatus instance
   * @param data data to be input to controller status
   *
   * @return ControllerStatus instance
   */
  friend ControllerStatus &operator<<(ControllerStatus &cs, const double &data);

  /**
   * @brief Add header for debug info
   *
   * @param cs Controller status instance
   * @param data header name
   *
   * @return ControllerStatus instance
   */
  friend ControllerStatus &operator<<(ControllerStatus &cs,
                                      const std::string &data);

  /**
  * @brief Helper function to add status, status description, debug header and
  * debug data to html table
  *
  * @param debug_tuple        status, status description, debug header and debug
  * data
  * @param html_table_writer  Table to add status and debug data
  */
  void addDebugInfo(const DebugInfo &debug_tuple,
                    HtmlTableWriter &html_table_writer);

public:
  /**
   * @brief Constructor with default status as not engaged and short message
   * describing the status
   *
   * @param status Type of status
   * @param status_description short message describing the status
   */
  ControllerStatus(
      ControllerStatus::Status status = ControllerStatus::NotEngaged,
      std::string status_description = "");
  /**
   * @brief Get a Html text describing the controller status
   *
   * @return html string containing controller status and description
   */
  std::string getHtmlStatusString();

  /**
   * @brief Set the internal status of controller status
   *
   * @param status Enum describing the current controller status
   * @param status_description Description about the status
   */
  void setStatus(ControllerStatus::Status status,
                 std::string status_description = "") {
    status_ = status;
    status_description_ = status_description;
  }

  /**
   * @brief Get the internal status of controller status
   * @return The status
   */
  Status status() { return status_; }

  /**
  * @brief convert enum to human readable value
  *
  * @return text describing the enum value
  */
  std::string statusAsText();

  /**
   * @brief Allows for explicit conversion of the class to a boolean variable
   * This is pretty useful to check controller status implies controller
   * convergence or not. For example:
   * if(controller_status) { do something} can be used to perform
   * actions if controller has converged.
   *
   * @return true if controller status is completed. False otherwise
   */
  explicit operator bool() const {
    return status_ == ControllerStatus::Completed;
  }
  /**
  * @brief Merge two status together. Add the debug info, header into
  * additional_debug_info tuple
  *
  * @param rhs_status merge this status into the current status
  *
  * @return current status with updated debug info and data
  */
  ControllerStatus &operator+=(const ControllerStatus &rhs_status);
};
