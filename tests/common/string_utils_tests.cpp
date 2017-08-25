#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aerial_autonomy/common/string_utils.h"

TEST(StringUtils, CurrentDateTimeString) {
  std::time_t t = std::time(nullptr);
  std::tm *tm = std::localtime(&t);

  std::string dt_str = string_utils::currentDateTimeString();
  ASSERT_THAT(dt_str, ::testing::MatchesRegex("_([0-9][0-9])_([0-9][0-9])_([0-"
                                              "9][0-9])_([0-9][0-9])_([0-9][0-"
                                              "9])_([0-9][0-9])"));
  ASSERT_EQ(std::stoi(dt_str.substr(1, 2)) + 2000, tm->tm_year + 1900);
  ASSERT_EQ(std::stoi(dt_str.substr(4, 2)), tm->tm_mon + 1);
  ASSERT_EQ(std::stoi(dt_str.substr(7, 2)), tm->tm_mday);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
