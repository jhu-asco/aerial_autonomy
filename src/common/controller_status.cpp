#include <aerial_autonomy/common/controller_status.h>

ControllerStatus::ControllerStatus(ControllerStatus::Status status,
                                   std::string description)
    : status_(status), description_(description) {}

std::string ControllerStatus::getStatus() {
  HtmlTableWriter controller_status_table;
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
  controller_status_table.beginRow();
  controller_status_table.addCell(description_, "", Colors::white, 4);
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
