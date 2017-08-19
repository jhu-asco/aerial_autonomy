#include <aerial_autonomy/common/controller_status.h>
// Html utils
#include <aerial_autonomy/common/html_utils.h>
// Ostream
#include <iostream>

ControllerStatus::ControllerStatus(ControllerStatus::Status status,
                                   std::string status_description)
    : status_(status), status_description_(status_description),
      debug_header("") {}

std::string ControllerStatus::getHtmlStatusString() {
  HtmlTableWriter controller_status_table(90); // Shorter width
  controller_status_table.beginRow();
  switch (status_) {
  case ControllerStatus::Active:
    controller_status_table.addCell("Active", "Status", Colors::green);
    break;
  case ControllerStatus::Completed:
    controller_status_table.addCell("Completed", "Status", Colors::green);
    break;
  case ControllerStatus::Critical:
    controller_status_table.addCell("Critical", "Status", Colors::red);
    break;
  case ControllerStatus::NotEngaged:
    controller_status_table.addCell("NotEngaged", "Status", Colors::yellow);
    break;
  default:
    controller_status_table.addCell("Unknown controller status!", "Status",
                                    Colors::red, 2);
    break;
  }
  controller_status_table.addCell(status_description_);
  controller_status_table.beginRow();
  if (!debug_header.empty()) {
    controller_status_table.addCell(debug_header);
  }
  for (const double &data : debug_info) {
    controller_status_table.addCell(data);
  }
  return controller_status_table.getTableString();
}

bool operator==(const ControllerStatus &controller_status,
                const ControllerStatus::Status &status_enum) {
  return controller_status.status_ == status_enum;
}

bool operator==(const ControllerStatus &lhs_controller_status,
                const ControllerStatus &rhs_controller_status) {
  return lhs_controller_status.status_ == rhs_controller_status.status_;
}

ControllerStatus &operator<<(ControllerStatus &cs, const double &data) {
  cs.debug_info.push_back(data);
  return cs;
}

ControllerStatus &operator<<(ControllerStatus &cs, const std::string &data) {
  cs.debug_header = std::string(data);
  return cs;
}
