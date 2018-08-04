#include <aerial_autonomy/common/controller_status.h>
// Ostream
#include <iostream>

#include <tuple>

ControllerStatus::ControllerStatus(ControllerStatus::Status status,
                                   std::string status_description)
    : status_(status), status_description_(status_description),
      debug_header_("") {}

std::string ControllerStatus::getHtmlStatusString() {
  HtmlTableWriter controller_status_table(90); // Shorter width
  addDebugInfo(
      std::make_tuple(status_, status_description_, debug_header_, debug_info_),
      controller_status_table);
  for (const auto &debug_tuple : additional_debug_info_) {
    addDebugInfo(debug_tuple, controller_status_table);
  }
  return controller_status_table.getTableString();
}

std::string ControllerStatus::statusAsText() {
  std::string result = "unknown";
  switch (status_) {
  case ControllerStatus::Active:
    result = "Active";
    break;
  case ControllerStatus::Completed:
    result = "Completed";
    break;
  case ControllerStatus::Critical:
    result = "Critical";
    break;
  case ControllerStatus::NotEngaged:
    result = "NotEngaged";
    break;
  }
  return result;
}

void ControllerStatus::addDebugInfo(
    const ControllerStatus::DebugInfo &debug_tuple,
    HtmlTableWriter &html_table_writer) {
  const auto &status = std::get<0>(debug_tuple);
  const auto &status_description = std::get<1>(debug_tuple);
  const auto &header = std::get<2>(debug_tuple);
  const auto &data_vector = std::get<3>(debug_tuple);
  html_table_writer.beginRow();
  switch (status) {
  case ControllerStatus::Active:
    html_table_writer.addCell("Active", "Status", Colors::green);
    break;
  case ControllerStatus::Completed:
    html_table_writer.addCell("Completed", "Status", Colors::green);
    break;
  case ControllerStatus::Critical:
    html_table_writer.addCell("Critical", "Status", Colors::red);
    break;
  case ControllerStatus::NotEngaged:
    html_table_writer.addCell("NotEngaged", "Status", Colors::yellow);
    break;
  default:
    html_table_writer.addCell("Unknown controller status!", "Status",
                              Colors::red, 2);
    break;
  }
  html_table_writer.addCell(status_description);
  html_table_writer.beginRow();
  if (!header.empty()) {
    html_table_writer.addCell(header);
  }
  for (const double &data : data_vector) {
    html_table_writer.addCell(data);
  }
}

ControllerStatus &ControllerStatus::
operator+=(const ControllerStatus &rhs_status) {
  // - If rhs status is critical, result status is critical.
  // - If lhs status is critical, result status  is critical.
  // - If rhs status is not complete and lhs status is not
  //   critical, result status is active
  // - If both rhs status and lhs status are complete, resulting
  //   status is complete too.
  if (rhs_status.status_ == ControllerStatus::Critical ||
      status_ == ControllerStatus::Critical) {
    status_ = ControllerStatus::Critical;
  } else if (rhs_status.status_ == ControllerStatus::Completed &&
             status_ == ControllerStatus::Completed) {
    status_ = ControllerStatus::Completed;
  } else {
    status_ = ControllerStatus::Active;
  }
  additional_debug_info_.push_back(
      std::make_tuple(rhs_status.status_, rhs_status.status_description_,
                      rhs_status.debug_header_, rhs_status.debug_info_));
  additional_debug_info_.insert(additional_debug_info_.end(),
                                rhs_status.additional_debug_info_.begin(),
                                rhs_status.additional_debug_info_.end());
  return *this;
}

bool operator==(const ControllerStatus &controller_status,
                const ControllerStatus::Status &status_enum) {
  return controller_status.status_ == status_enum;
}

bool operator==(const ControllerStatus &lhs_controller_status,
                const ControllerStatus &rhs_controller_status) {
  return lhs_controller_status.status_ == rhs_controller_status.status_;
}

bool operator!=(const ControllerStatus &lhs_controller_status,
                const ControllerStatus &rhs_controller_status) {
  return lhs_controller_status.status_ != rhs_controller_status.status_;
}

ControllerStatus &operator<<(ControllerStatus &cs, const double &data) {
  cs.debug_info_.push_back(data);
  return cs;
}

ControllerStatus &operator<<(ControllerStatus &cs, const std::string &data) {
  cs.debug_header_ = std::string(data);
  return cs;
}
