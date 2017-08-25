#include "aerial_autonomy/common/string_utils.h"

#include <ctime>
#include <stdexcept>

namespace string_utils {
std::string currentDateTimeString() {
  std::time_t t = std::time(nullptr);
  char time_str[100];
  if (!std::strftime(time_str, sizeof(time_str), "_%y_%m_%d_%H_%M_%S",
                     std::localtime(&t))) {
    throw std::runtime_error("Log timestamp exceeds string size");
  }
  return std::string(time_str);
}
}
