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
 * @brief create state machine config from file name specified in ros param
 *
 * @param nh NodeHandle to get param for config file name
 *
 * @return state machine config
 */
BaseStateMachineConfig createStateMachineConfig(ros::NodeHandle &nh);

/**
 * @brief create UAV system handler config from ros param
 *
 * @param nh NodeHandle to get param for config file name
 *
 * @return uav system handler config
 */
UAVSystemHandlerConfig createUAVSystemHandlerConfig(ros::NodeHandle &nh);
