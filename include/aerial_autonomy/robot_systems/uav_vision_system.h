#pragma once
#include "aerial_autonomy/common/html_utils.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/robot_systems/uav_system.h"
#include "uav_system_config.pb.h"

#include <tf/tf.h>

/**
* @brief UAV system with a camera and visual sevoing capabilities.
*/
class UAVVisionSystem : public UAVSystem {
public:
  /**
  * @brief Constructor
  * @param tracker Used to track targets for visual servoing
  * @param drone_hardware UAV driver
  * @param config Configuration parameters
  */
  UAVVisionSystem(BaseTracker &tracker, parsernode::Parser &drone_hardware,
                  UAVSystemConfig config)
      : UAVSystem(drone_hardware, config),
        camera_transform_(math::getTransformFromVector(
            config_.uav_vision_system_config().camera_transform())),
        tracker_(tracker), constant_heading_depth_controller_(
                               config_.uav_vision_system_config()
                                   .constant_heading_depth_controller_config()),
        visual_servoing_drone_connector_(tracker, drone_hardware_,
                                         constant_heading_depth_controller_,
                                         camera_transform_) {
    controller_hardware_connector_container_.setObject(
        visual_servoing_drone_connector_);
  }

  /**
  * @brief Get the direction vector of the tracking target in
  * the
  * global frame
  * @param pos Returned position
  * @return True if successful and false otherwise
  */
  bool getTrackingVector(Position &pos) {
    return visual_servoing_drone_connector_.getTrackingVectorGlobalFrame(pos);
  }

  bool initializeTracker() { return tracker_.initialize(); }

  std::string getSystemStatus() const {
    std::stringstream status;
    status << UAVSystem::getSystemStatus() << std::endl;
    HtmlTableWriter table_writer;
    table_writer.beginRow();
    table_writer.addHeader("Tracker Status", Colors::blue);
    table_writer.beginRow();
    std::string tracking_valid =
        (tracker_.trackingIsValid() ? "True" : "False");
    std::string valid_color =
        (tracker_.trackingIsValid() ? Colors::green : Colors::red);
    table_writer.addCell(tracking_valid, "Valid", valid_color);
    table_writer.beginRow();
    table_writer.addCell("Tracking Vectors: ");
    std::unordered_map<uint32_t, Position> tracking_vectors;
    if (tracker_.getTrackingVectors(tracking_vectors)) {
      for (auto tv : tracking_vectors) {
        tf::Vector3 tv_body_frame =
            camera_transform_ *
            tf::Vector3(tv.second.x, tv.second.y, tv.second.z);
        table_writer.beginRow();
        table_writer.addCell(tv.first);
        table_writer.addCell(tv_body_frame.x());
        table_writer.addCell(tv_body_frame.y());
        table_writer.addCell(tv_body_frame.z());
      }
    }
    status << table_writer.getTableString();
    return status.str();
  }

protected:
  /**
  * @brief Camera transform in the frame of the UAV
  */
  tf::Transform camera_transform_;

private:
  BaseTracker &tracker_;
  /**
  * @brief Track the target position given by the tracker
  */
  ConstantHeadingDepthController constant_heading_depth_controller_;
  /**
  * @brief Connector for the constant heading depth controller to
  * UAV
  */
  VisualServoingControllerDroneConnector visual_servoing_drone_connector_;
};
