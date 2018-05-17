#include <aerial_autonomy/controller_connectors/mpc_controller_drone_connector.h>

///\todo Gowtham Replace state estimator with something thats created internally
/// based on proto
MPCControllerDroneConnector::MPCControllerDroneConnector(
    parsernode::Parser &drone_hardware,
    AbstractMPCController<QuadState, RollPitchYawThrust> &controller,
    AbstractConstraintGenerator &constraint_generator,
    AbstractStateEstimator<QuadState, RollPitchYawThrust> &state_estimator)
    : MPCControllerConnector(controller, constraint_generator, state_estimator,
                             ControllerGroup::UAV),
      drone_hardware_(drone_hardware) {}

void MPCControllerDroneConnector::sendCommandsToHardware(
    RollPitchYawThrust control) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = control.r;
  rpyt_msg.y = control.p;
  rpyt_msg.z = control.y;
  rpyt_msg.w = control.t;
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}
