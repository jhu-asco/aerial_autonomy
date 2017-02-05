#include <aerial_autonomy/logic_states/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
struct RobotSystem {};
struct LSM {};

struct EmptyRunFunctor : BaseRunFunctor<RobotSystem, LSM> {
  virtual void run(RobotSystem const &, LSM &) {}
};

struct EmptyGuardFunctor : BaseGuardFunctor<RobotSystem, LSM> {
  virtual bool guard(RobotSystem const &, LSM &) { return true; }
};
////

/// \brief TEST
/// All the tests are defined here
TEST(BaseStateTests, BaseStateCtor) {
  typedef BaseState<RobotSystem, LSM, EmptyRunFunctor> BaseStateEmpty;
  ASSERT_NO_THROW(BaseStateEmpty());

  typedef BaseState<RobotSystem, LSM, EmptyRunFunctor, EmptyGuardFunctor>
      BaseStateEmptyGuard;
  ASSERT_NO_THROW(BaseStateEmptyGuard());
}

TEST(BaseStateTests, EmptytFctor) {
  EmptyRunFunctor empty_functor;
  RobotSystem rs;
  LSM lsm;
  // Robotsystem, LSM, source, target
  ASSERT_NO_THROW(empty_functor.run(rs, lsm));
}

TEST(BaseStateTests, EmptytGuardFctor) {
  EmptyGuardFunctor empty_functor;
  RobotSystem rs;
  LSM lsm;
  // Robotsystem, LSM, source, target
  ASSERT_TRUE(empty_functor.guard(rs, lsm));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
