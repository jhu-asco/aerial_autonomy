#include "aerial_autonomy/controllers/manual_velocity_controller.h"
#include <gtest/gtest.h>

TEST(ManualVelocityControllerTests, Constructor)
{
  ASSERT_NO_THROW(ManualVelocityController());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();  
}