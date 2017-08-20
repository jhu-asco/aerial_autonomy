#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <aerial_autonomy/common/controller_status.h>
#include <aerial_autonomy/common/html_utils.h>

TEST(ControllerStatusTests, Constructor) {
  ControllerStatus controller_status(ControllerStatus::Active, "active");
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr("active"));
  ASSERT_EQ(controller_status, ControllerStatus::Active);
}

TEST(ControllerStatusTests, SetStatus) {
  ControllerStatus controller_status(ControllerStatus::Active, "active");
  ASSERT_EQ(controller_status, ControllerStatus::Active);
  controller_status.setStatus(ControllerStatus::Completed, "completed");
  ASSERT_EQ(controller_status, ControllerStatus::Completed);
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr("completed"));
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr(Colors::green));
}

TEST(ControllerStatusTests, Critical) {
  ControllerStatus controller_status(ControllerStatus::Critical, "critical");
  ASSERT_EQ(controller_status, ControllerStatus::Critical);
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr("critical"));
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr(Colors::red));
}

TEST(ControllerStatusTests, NotEngaged) {
  ControllerStatus controller_status(ControllerStatus::NotEngaged,
                                     "notengaged");
  ASSERT_EQ(controller_status, ControllerStatus::NotEngaged);
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr("notengaged"));
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr(Colors::yellow));
}

TEST(ControllerStatusTests, InputHeader) {
  ControllerStatus controller_status(ControllerStatus::NotEngaged,
                                     "notengaged");
  controller_status << "DebugInfoHeader";
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr("DebugInfoHeader"));
}

TEST(ControllerStatusTests, InputData) {
  ControllerStatus controller_status(ControllerStatus::NotEngaged,
                                     "notengaged");
  controller_status << 12121.0; // Some random number
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr("12121.0"));
}

TEST(ControllerStatusTests, UnknownControllerStatus) {
  ControllerStatus controller_status(ControllerStatus::Status(5), "notengaged");
  ASSERT_THAT(controller_status.getHtmlStatusString(),
              testing::HasSubstr("Unknown controller status"));
}

TEST(ControllerStatusTests, Compare) {
  ControllerStatus controller_status_1(ControllerStatus::Active);
  ControllerStatus controller_status_2(ControllerStatus::NotEngaged);
  ControllerStatus controller_status_3(ControllerStatus::Active, "Active");
  EXPECT_EQ(controller_status_1, controller_status_3);
  EXPECT_NE(controller_status_1, controller_status_2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
