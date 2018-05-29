#include <aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h>

///\todo Gowtham Replace state estimator with something thats created internally
/// based on proto
MPCControllerAirmConnector::MPCControllerAirmConnector(
    parsernode::Parser &drone_hardware, ArmParser &arm_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    AbstractConstraintGenerator &constraint_generator,
    AbstractStateEstimator<StateType, ControlType> &state_estimator)
    : MPCControllerConnector(controller, constraint_generator, state_estimator,
                             ControllerGroup::UAV),
      drone_hardware_(drone_hardware), arm_hardware_(arm_hardware) {}

void MPCControllerAirmConnector::sendCommandsToHardware(ControlType control) {
  geometry_msgs::Quaternion rpyt_msg;
  RollPitchYawThrust quad_control = control.first;
  rpyt_msg.x = quad_control.r;
  rpyt_msg.y = quad_control.p;
  rpyt_msg.z = quad_control.y;
  rpyt_msg.w = quad_control.t;
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
  arm_hardware_.setJointAngles(control.second);
}
