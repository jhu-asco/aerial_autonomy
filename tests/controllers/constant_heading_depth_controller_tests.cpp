#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"

#include <gtest/gtest.h>

TEST(ConstantHeadingDepthControllerTests, Constructor) {
  ASSERT_NO_THROW(ConstantHeadingDepthController());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
