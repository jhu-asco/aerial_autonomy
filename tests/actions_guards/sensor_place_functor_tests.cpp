#include "position_yaw.pb.h"
#include <aerial_autonomy/actions_guards/sensor_place_states_actions.h>
#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
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
using ssa = SensorPlaceStatesActions<UAVArmLogicStateMachine>;

// Picking internal actions

using PlaceInternalAction =
    PlaceInternalActionFunctor_<UAVArmLogicStateMachine>;

class SensorPlaceFunctorTests : public ::testing::Test {
//There's nothing needed here.
protected:
  SensorPlaceFunctorTests() {}
  virtual ~SensorPlaceFunctorTests(){};
};
/// \brief Test Visual Servoing
//The PlaceInternalAction initializes all the new functors.
//The functionality is tested in the state machine test.
TEST_F(SensorPlaceFunctorTests, Constructor) {
  ASSERT_NO_THROW(new PlaceInternalAction());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
