#include "position_yaw.pb.h"
#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/actions_guards/orange_picking_states_actions.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <arm_parsers/arm_simulator.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <typeindex>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/**
* @brief Namespace fortest utilities
*/
using namespace test_utils;

/**
* @brief Sample logic state machine templated on UAV arm system
*/
using UAVArmLogicStateMachine = SampleLogicStateMachine_<UAVArmSystem>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using opsa = OrangePickingStatesActions<UAVArmLogicStateMachine>;

// Picking internal actions

using PathFollowInternalAction =
    PathFollowInternalActionFunctor_<UAVArmLogicStateMachine>;

class OrangePickingFunctorTests : public ::testing::Test {
  // There's nothing needed here.
protected:
  OrangePickingFunctorTests() {}
  virtual ~OrangePickingFunctorTests(){};
};
/// \brief Test Visual Servoing
// The PickingInternalAction initializes all the new functors.
// The functionality is tested in the state machine test.
TEST_F(OrangePickingFunctorTests, Constructor) {
  ASSERT_NO_THROW(new PathFollowInternalAction());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
