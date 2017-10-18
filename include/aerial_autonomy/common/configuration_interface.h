#pragma once
#include <glog/logging.h>
#include <stdexcept>
#include <tuple>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

/**
 * @brief Dummy base class for all the proto configs.
 */
class ProtoBase {};

/**
 * @brief Wrapper subclass to wrap any class as
 * a subclass of common base class. This lets
 * us store these random objects in a typemap
 *
 * @tparam T type of the object being stored
 */
template <class T> class ProtoBaseWrapper : public ProtoBase {
public:
  /**
   * @brief Constructor to store the input data
   *
   * @param class object to store
   */
  ProtoBaseWrapper(T input) : input_(input) {}
  /**
   * @brief Get a copy of the stored object
   *
   * @return copy of stored object
   */
  T getInput() const { return input_; }
  /**
   * @brief Get a reference to stored object.
   * Can be used to modify the internally stored object
   *
   * @return reference to stored object
   */
  T &getInput() { return input_; }

private:
  T input_; ///< Stored internal class object
};

/**
 * @brief Class that stores different configuration objects in a typemap
 * and provides access to them.
 *
 * This interface should be used with state machine configs to store
 * different configuration objects used by various states in a state machine.
 *
 * The configuration objects are mapped based on the target state type.
 */
class ConfigurationInterface {
  /**
   * Stores a map of the config based on the type index of state. The configs
   * are encoded as a tuple of the type index of proto config and the proto
   * config
   * as a wrapped object with a base class as ProtoBase.
   */
  std::unordered_map<std::type_index,
                     std::tuple<const std::type_info *, ProtoBase *>>
      configurations;

public:
  /**
   * @brief add a configuration object into typemap
   *
   * @tparam TargetState The state of statemachine for which the configuration
   * is intended
   * @tparam ProtoConfig the type of configuration object being stored
   * @param config The configuration object to be stored
   */
  template <class TargetState, class ProtoConfig>
  void addConfig(ProtoConfig config) {
    auto config_wrapper = new ProtoBaseWrapper<ProtoConfig>(config);
    configurations[typeid(TargetState)] =
        std::make_tuple(&typeid(ProtoConfig), config_wrapper);
  }
  /**
   * @brief Get the configuration object stored in typemap
   *
   * @tparam ProtoConfig The type of configuration object to retrieve
   * @tparam TargetState The state for which the configuration is intended
   *
   * @return A copy of the configuration object stored in typemap
   */
  template <class ProtoConfig, class TargetState>
  ProtoConfig getConfig() const {
    auto it = configurations.find(typeid(TargetState));
    if (it == configurations.end())
      throw std::runtime_error("Cannot find the target state key");
    auto config_tuple = it->second;
    if (std::type_index(*std::get<0>(config_tuple)) !=
        std::type_index(typeid(ProtoConfig))) {
      throw std::runtime_error(
          "Proto config does not match with the type of stored");
    }
    auto config_wrapper =
        static_cast<ProtoBaseWrapper<ProtoConfig> *>(std::get<1>(config_tuple));
    return config_wrapper->getInput();
  }
  /**
   * @brief Get a reference to the configuration object stored in typemap
   *
   * @tparam ProtoConfig The type of configuration object to retrieve
   * @tparam TargetState The state for which the configuration is intended
   *
   * @return A reference to the configuration object stored in typemap
   */
  template <class ProtoConfig, class TargetState> ProtoConfig &getConfig() {
    auto it = configurations.find(typeid(TargetState));
    if (it == configurations.end())
      throw std::runtime_error("Cannot find the target state key");
    auto config_tuple = it->second;
    if (std::type_index(*std::get<0>(config_tuple)) !=
        std::type_index(typeid(ProtoConfig))) {
      throw std::runtime_error(
          "Proto config does not match with the type of stored");
    }
    auto config_wrapper =
        static_cast<ProtoBaseWrapper<ProtoConfig> *>(std::get<1>(config_tuple));
    return config_wrapper->getInput();
  }
};
