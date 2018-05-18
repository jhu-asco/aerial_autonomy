#pragma once

/**
 * @brief class that stores the package id of interest
 */
struct ObjectId {
  /**
   * @brief Constructor that saves the id with default value of 0
   *
   * @param id_ The id to store
   */
  ObjectId(uint32_t id_ = 0) : id(id_) {}
  /**
   * @brief Internal id being stored for an object
   */
  uint32_t id;
};
