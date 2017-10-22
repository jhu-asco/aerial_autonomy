#pragma once
#include "base_state_machine_config.pb.h"
#include "log_config.pb.h"
#include "uav_system_handler_config.pb.h"
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/log/log.h>
#include <glog/logging.h>
#include <ros/ros.h>

/**
 * Utility functions to create configs from ros params.
 * These functions are used by the system handler nodes
 */

/**
 * @brief create log config file and configure log using the config
 *
 * @param nh NodeHandle to get param for config file name
 */
void createAndConfigureLogConfig(ros::NodeHandle &nh);

/**
 * @brief Load a proto config from a ros param
 *
 * @tparam ConfigT Type of config to load from file
 * @param nh NodeHandle to get param for config file name
 * @param config_param_name ros parameter containing file name for proto config
 *
 * @return Config loaded from the param file
 */
template <class ConfigT>
ConfigT loadConfigFromROSParam(ros::NodeHandle &nh,
                               std::string config_param_name) {
  std::string config_filename;
  if (!nh.getParam(config_param_name, config_filename)) {
    LOG(FATAL) << "ROS param \"" << config_param_name << "\" not found";
  }

  ConfigT config;
  if (!proto_utils::loadProtoText(config_filename, config)) {
    LOG(FATAL) << "Failed to open config file: " << config_filename;
  }
  return config;
}
