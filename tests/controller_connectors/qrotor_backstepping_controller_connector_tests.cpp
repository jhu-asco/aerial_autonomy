#include "aerial_autonomy/controller_connectors/qrotor_backstepping_controller_connector.h"
#include "aerial_autonomy/tests/test_utils.h"

#include <quad_simulator_parser/quad_simulator.h>

#include <gtest/gtest.h>

using namespace quad_simulator;

using namespace test_utils;

class QrotorBacksteppingControllerConnectorTests : public ::testing::Test {
public:
  /* Todo Constructor
  */

  /* Todo QrotorBacksteppingControllerConfig config;
  */

  /* Todo reset unique_ptr
  */
  controller_.reset(new QrotorBacksteppingController(config);
  controller_connector_.reset(new QrotorBacksteppingControllerConnector(drone_hardware_, *controller_, thrust_gain_estimator_, config, std::chrono::milliseconds(20)));
  drone_hardware_.usePerfectTime();

  /* Todo static void SetUpTestCase()
  */

  /* Todo void runUntilConvergence()
  */
  
  QuadSimulator drone_hardware_;
  std::unique_ptr<QrotorBacksteppingController> controller_;
  std::unique_ptr<QrotorBacksteppingControllerConnector> controller_connector_;
  ThrustGainEstimator thrust_gain_estimator_;
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
