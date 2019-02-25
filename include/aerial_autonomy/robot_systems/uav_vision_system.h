#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/html_utils.h"
#include "aerial_autonomy/controller_connectors/relative_pose_visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/controller_connectors/rpyt_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/controller_connectors/visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/controller_connectors/visual_servoing_reference_connector.h"
#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/controllers/quad_particle_reference_controller.h"
#include "aerial_autonomy/controllers/quad_polynomial_reference_controller.h"
#include "aerial_autonomy/controllers/quad_polynomial_adaptive_reference_controller.h"
#include "aerial_autonomy/controllers/rpyt_based_relative_pose_controller.h"
#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/estimators/acceleration_bias_estimator.h"
#include "aerial_autonomy/estimators/tracking_vector_estimator.h"
#include "aerial_autonomy/robot_systems/uav_system.h"
#include "aerial_autonomy/trackers/alvar_tracker.h"
#include "aerial_autonomy/trackers/roi_to_position_converter.h"
#include "aerial_autonomy/trackers/simulated_ros_tracker.h"
#include "uav_system_config.pb.h"

#include <tf/tf.h>

/**
* @brief UAV system with a camera and visual sevoing capabilities.
*/
class UAVVisionSystem : public UAVSystem {
protected:
  /**
   * @brief typedef for abstract base tracker
   */
  using BaseTrackerPtr = std::shared_ptr<BaseTracker>;

public:
  /**
   * @brief Shortcut for visual servoing reference connector
   *
   * with dependent mpc connector
   */
  using RPYTAdaptiveReferenceConnectorT =
      VisualServoingReferenceConnector<
          ParticleStateYaw, Snap,
          RPYTRelativePoseAdaptiveEstimateConnector>;

  using RPYTVisualServoingReferenceConnectorT =
      VisualServoingReferenceConnector<
          Eigen::VectorXd, Eigen::VectorXd,
          RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>>;

  using MPCVisualServoingReferenceConnectorT = VisualServoingReferenceConnector<
      Eigen::VectorXd, Eigen::VectorXd,
      MPCControllerConnector<Eigen::VectorXd, Eigen::VectorXd>>;
  /**
  * @brief Constructor
  * @param config Configuration parameters
  * @param tracker Used to track targets for visual servoing. If provided, will
  * overwrite
  * the one in config file
  * @param drone_hardware UAV driver. If provided, will overwrite the one in
  * config file
  */
  UAVVisionSystem(UAVSystemConfig config, BaseTrackerPtr tracker = nullptr,
                  UAVParserPtr drone_hardware = nullptr,
                  SensorPtr<Velocity> velocity_sensor = nullptr)
      : UAVSystem(config, drone_hardware, velocity_sensor),
        camera_transform_(conversions::protoTransformToTf(
            config_.uav_vision_system_config().camera_transform())),
        tracker_(UAVVisionSystem::chooseTracker(tracker, drone_hardware_,
                                                camera_transform_, config)),
        acceleration_bias_estimator_(config_.uav_vision_system_config()
                                         .acceleration_bias_estimator_config()),
        constant_heading_depth_controller_(
            config_.uav_vision_system_config()
                .constant_heading_depth_controller_config()),
        rpyt_based_relative_pose_controller_(
            config_.uav_vision_system_config()
                .rpyt_based_relative_pose_controller_config(),
            std::chrono::milliseconds(config_.uav_controller_timer_duration())),
        velocity_based_relative_pose_controller_(
            config_.uav_vision_system_config()
                .velocity_based_relative_pose_controller_config(),
            std::chrono::milliseconds(config_.uav_controller_timer_duration())),
        quad_reference_generator_(
            config_.uav_vision_system_config().particle_reference_config()),
        quad_poly_reference_generator_(
            config_.uav_vision_system_config().poly_reference_config()),
        quad_poly_adaptive_reference_generator_(
            config_.uav_vision_system_config().poly_reference_config()),
        visual_servoing_drone_connector_(*tracker_, *drone_hardware_,
                                         constant_heading_depth_controller_,
                                         camera_transform_),
        rpyt_relative_pose_visual_servoing_drone_connector_(
            *tracker_, *drone_hardware_, rpyt_based_relative_pose_controller_,
            thrust_gain_estimator_, acceleration_bias_estimator_,
            camera_transform_,
            conversions::protoTransformToTf(config_.uav_vision_system_config()
                                                .tracking_offset_transform())),
        velocity_relative_pose_visual_servoing_drone_connector_(
            *tracker_, *drone_hardware_,
            velocity_based_relative_pose_controller_, camera_transform_,
            conversions::protoTransformToTf(config_.uav_vision_system_config()
                                                .tracking_offset_transform())),
        rpyt_adaptive_reference_connector_(
            *tracker_, *drone_hardware_, quad_poly_adaptive_reference_generator_,
            rpyt_adaptive_estimate_controller_drone_connector_, camera_transform_,
            conversions::protoTransformToTf(
                config_.uav_vision_system_config().tracking_offset_transform()),
            config_.uav_vision_system_config()
                .gain_visual_servoing_tracking_pose(),
            odom_sensor_),
        rpyt_visual_servoing_reference_connector_(
            *tracker_, *drone_hardware_, quad_poly_reference_generator_,
            rpyt_based_reference_connector_, camera_transform_,
            conversions::protoTransformToTf(
                config_.uav_vision_system_config().tracking_offset_transform()),
            config_.uav_vision_system_config()
                .gain_visual_servoing_tracking_pose(),
            odom_sensor_),
        mpc_visual_servoing_reference_connector_(
            *tracker_, *drone_hardware_, quad_poly_reference_generator_,
            quad_mpc_connector_, camera_transform_,
            conversions::protoTransformToTf(
                config_.uav_vision_system_config().tracking_offset_transform()),
            config_.uav_vision_system_config()
                .gain_visual_servoing_tracking_pose(),
            odom_sensor_) {
    controller_connector_container_.setObject(visual_servoing_drone_connector_);
    controller_connector_container_.setObject(
        rpyt_relative_pose_visual_servoing_drone_connector_);
    controller_connector_container_.setObject(
        velocity_relative_pose_visual_servoing_drone_connector_);
    controller_connector_container_.setObject(
        mpc_visual_servoing_reference_connector_);
    controller_connector_container_.setObject(
        rpyt_visual_servoing_reference_connector_);
    controller_connector_container_.setObject(
        rpyt_adaptive_reference_connector_);
    controller_connector_container_.setObject(
        rpyt_adaptive_estimate_controller_drone_connector_);
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

  /**
  * @brief Get the ID of the current tracked target
  * @param id Returned ID
  * @return True if successful, false otherwise
  */
  bool getTrackingVectorId(uint32_t &id) {
    std::tuple<uint32_t, tf::Transform> tracking_vec;
    bool valid = tracker_->getTrackingVector(tracking_vec);
    if (valid) {
      id = std::get<0>(tracking_vec);
    }
    return valid;
  }

  /**
  * @brief Set the tracker's tracking strategy
  * @param strategy Tracking strategy to set
  */
  void
  setTrackingStrategy(std::unique_ptr<TrackingStrategy> &&tracking_strategy) {
    tracker_->setTrackingStrategy(std::move(tracking_strategy));
  }

  /**
   * @brief Initialize the internal tracker.
   * Usually called before tracking begins.
   *
   * @return True if initialization succeeded
   */
  bool initializeTracker() { return tracker_->initialize(); }

  std::string getSystemStatus() const {
    std::stringstream status;
    status << UAVSystem::getSystemStatus() << std::endl;
    HtmlTableWriter table_writer;
    table_writer.beginRow();
    table_writer.addHeader("Tracker Status", Colors::blue);
    table_writer.beginRow();
    std::string tracking_valid =
        (tracker_->trackingIsValid() ? "True" : "False");
    std::string valid_color =
        (tracker_->trackingIsValid() ? Colors::green : Colors::red);
    table_writer.addCell(tracking_valid, "Valid", valid_color);
    table_writer.beginRow();
    table_writer.addCell("Tracking Vectors: ");
    std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
    if (tracker_->getTrackingVectors(tracking_vectors)) {
      for (auto tv : tracking_vectors) {
        tf::Transform tv_body_frame = camera_transform_ * tv.second;
        table_writer.beginRow();
        table_writer.addCell(tv.first);
        table_writer.addCell(tv_body_frame.getOrigin().x());
        table_writer.addCell(tv_body_frame.getOrigin().y());
        table_writer.addCell(tv_body_frame.getOrigin().z());
        double roll, pitch, yaw;
        tv_body_frame.getBasis().getRPY(roll, pitch, yaw);
        table_writer.addCell(roll);
        table_writer.addCell(pitch);
        table_writer.addCell(yaw);
      }
    }
    status << table_writer.getTableString();
    return status.str();
  }

  /**
   * @brief reset the integrator of internal relative pose controller
   */
  void resetRelativePoseController() {
    rpyt_based_relative_pose_controller_.resetIntegrator();
  }

  void setNoisePolyReferenceController(bool flag) {
    quad_poly_reference_generator_.useNoise(flag);
    quad_poly_adaptive_reference_generator_.useNoise(flag);
  }

  /**
  * @brief Get the estimated acceleration bias
  * @return Estimated acceleration bias
  */
  Eigen::Vector3d getAccelerationBias() {
    return acceleration_bias_estimator_.getAccelerationBias();
  }

protected:
  /**
  * @brief Camera transform in the frame of the UAV
  */
  tf::Transform camera_transform_;

  BaseTrackerPtr tracker_; ///< Tracking system

  /**
   * @brief Choose between user provided tracker and the one in the config file
   *
   * @param tracker user provided tracker or null pointer
   * @param config Configuration file containing a tracker type
   *
   * @return the user provided one if it is not nullptr and create the config
   * specified one if user provided nullptr.
   */
  static BaseTrackerPtr chooseTracker(BaseTrackerPtr tracker,
                                      UAVParserPtr drone_hardware,
                                      tf::Transform camera_transform,
                                      UAVSystemConfig &config) {
    BaseTrackerPtr tracker_pointer;
    if (tracker) {
      tracker_pointer = tracker;
    } else {
      std::string tracker_type =
          config.uav_vision_system_config().tracker_type();
      if (tracker_type == "ROI") {
        tracker_pointer = BaseTrackerPtr(new RoiToPositionConverter());
      } else if (tracker_type == "Alvar") {
        tracker_pointer = BaseTrackerPtr(new AlvarTracker());
      } else if (tracker_type == "Simulated") {
        tracker_pointer = BaseTrackerPtr(new SimulatedROSTracker(
            *drone_hardware, camera_transform, "simulated_ros_tracker"));
      } else {
        throw std::runtime_error("Unknown tracker type provided: " +
                                 tracker_type);
      }
    }
    return tracker_pointer;
  }

private:
  /**
  * @brief Estimates acceleration bias
  */
  AccelerationBiasEstimator acceleration_bias_estimator_;
  /**
  * @brief Track the target position given by the tracker
  */
  ConstantHeadingDepthController constant_heading_depth_controller_;
  /**
  * @brief Controller to mantain a relative pose with respect to the object
  */
  RPYTBasedRelativePoseController rpyt_based_relative_pose_controller_;
  /**
  * @brief Controller to mantain a relative pose with respect to the object
  * using velocity commands
  */
  VelocityBasedRelativePoseController velocity_based_relative_pose_controller_;
  /**
   * @brief generates exponential reference trajectory for quad from start to
   * end
   */
  QuadParticleReferenceController quad_reference_generator_;
  /**
   * @brief generates polynomial reference trajectory for quad from start to
   * end
   */
  QuadPolynomialReferenceController quad_poly_reference_generator_;
  QuadPolynomialAdaptiveReferenceController quad_poly_adaptive_reference_generator_;
  /**
  * @brief Connector for the constant heading depth controller to
  * UAV
  */
  VisualServoingControllerDroneConnector visual_servoing_drone_connector_;
  /**
  * @brief Connects relative pose controller ot tracker and UAV
  */
  RPYTRelativePoseVisualServoingConnector
      rpyt_relative_pose_visual_servoing_drone_connector_;
  /**
  * @brief Connects velocity relative pose controller ot tracker and UAV
  */
  RelativePoseVisualServoingControllerDroneConnector
      velocity_relative_pose_visual_servoing_drone_connector_;
  /**
   * @brief High level visual servoing connector
   */
  RPYTAdaptiveReferenceConnectorT
      rpyt_adaptive_reference_connector_;

  RPYTVisualServoingReferenceConnectorT
      rpyt_visual_servoing_reference_connector_;
  /**
   * @brief High level visual servoing connector
   */
  MPCVisualServoingReferenceConnectorT mpc_visual_servoing_reference_connector_;
};
