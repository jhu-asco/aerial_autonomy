#include "aerial_autonomy/controllers/manual_velocity_controller.h"
#include <gtest/gtest.h>

TEST(ManualVelocityControllerTests, Constructor)
{
  ASSERT_NO_THROW(ManualVelocityController());
}

TEST(ManualVelocityControllerTests, IsConvergedTest)
{
  ManualVelocityController controller;
  EmptyGoal goal;
  controller.setGoal(goal);

  JoysticksYaw joy_data(0.0, 0.0, 0.0, 0.0, 0.0);
  VelocityYaw vel_data(0, 0 ,0, 0.0);
  std::tuple<JoysticksYaw, VelocityYaw> sensor_data=
   std::make_tuple(joy_data, vel_data);

   RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.y, 0, 1e-4);
  ASSERT_NEAR(controls.p, 0, 1e-4);
  ASSERT_NEAR(controls.t, 0, 1e-4);
  ASSERT_NEAR(controls.r, 0, 1e-4);
  ASSERT_TRUE(result);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();  
}