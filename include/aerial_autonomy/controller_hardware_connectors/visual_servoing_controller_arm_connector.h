#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controllers/relative_position_controller.h"
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/types/position.h"

#include <arm_parsers/arm_parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 */
class VisualServoingControllerArmConnector
    : public ControllerHardwareConnector<std::tuple<Position, Position>,
                                         Position, Position> {
public:
  /**
   * @brief Constructor
   */
  VisualServoingControllerArmConnector(BaseTracker &tracker,
                                       ArmParser &arm_hardware,
                                       RelativePositionController &controller,
                                       tf::Transform camera_transform,
                                       tf::Transform arm_transform)
      : ControllerHardwareConnector(controller, HardwareType::Arm),
        arm_hardware_(arm_hardware), tracker_(tracker),
        camera_transform_(camera_transform), arm_transform_(arm_transform) {}
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingControllerArmConnector() {}

  /**
   * @brief Get the tracking vector of the tracker in the arm
   * frame
   * @param tracking_vector Returned tracking vector
   * @return True if successful and false otherwise
   */
  bool getTrackingVectorArmFrame(Position &tracking_vector);

protected:
  /**
   * @brief Extracts pose data from ROI
   *
   * @param sensor_data Position of arm end effector and position of object
   * tracked by ROI in arm frame
   *
   * @return true if able to extract ROI position
   */
  virtual bool extractSensorData(std::tuple<Position, Position> &sensor_data);

  /**
   * @brief  Send position commands to hardware
   *
   * @param controls position command to send to arm
   */
  virtual void sendHardwareCommands(Position controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  ArmParser &arm_hardware_;
  /**
  * @brief Tracks whatever we are servoing to
  */
  BaseTracker &tracker_;
  /**
  * @brief camera transform with respect to body
  */
  tf::Transform camera_transform_;
  /**
  * @brief arm transform with respect to body
  */
  tf::Transform arm_transform_;
};
