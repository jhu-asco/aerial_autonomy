#pragma once
#include <aerial_autonomy/common/type_map.h>
#include <aerial_autonomy/trackers/base_tracker.h>

/**
 * @brief Manages vision systems and vision system configurations
 */
class BaseVisionSystem {
protected:
  /**
   * @brief Container to store and retrieve trackers
   */
  TypeMap<BaseTracker> tracker_container_;
  /**
   * @brief Get a reference to a tracker
   * @tparam Type of tracker to get reference to
   * @return The tracker
   */
  template <class TrackerT> TrackerT &tracker() {
    TrackerT *tracker_ptr = tracker_container_.getObject<TrackerT>();
    return *tracker_ptr;
  }

public:
  /**
   * @brief Constructor
   */
  BaseVisionSystem();
  /*
  * How should we instantiate the different trackers?
  * 1) Let BaseVisionSystem instantiate them, own them, and add them to tracker
  * container
  *  - BaseVisionSystem will need to have a ROS dependency so it can pass node
  * handles to
  *    some tracker constructors
  * 2) Let SystemHandler construct them and add them to BaseVisionSystem
  *  + Decouples BaseVisionSystem from ROS
  *  - RobotSystem subclass will need to know about ROS dependencies and
  * specific trackers
  *    to pass specific trackers to each particular ControllerHardwareConnector
  * 3) Let SystemHandler construct them and add them to BaseVisionSystem.  Also,
  * let SystemHandler
  *    construct ControllerHardwareConnectors and add them to system
  *
  */
  /**
   * @brief Set a tracker's tracking strategy
   * @tparam Strategy will be set for this tracker type
   */
  template <class TrackerT>
  void setTrackingStrategy(std::unique_ptr<TrackingStrategy> strategy) {
    TrackerT *tracker_ptr = tracker_container_.getObject<TrackerT>();
    tracker_ptr->setTrackingStrategy(std::move(strategy));
  }
};
