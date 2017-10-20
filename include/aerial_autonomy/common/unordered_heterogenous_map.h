#pragma once
#include <glog/logging.h>
#include <gtest/gtest_prod.h>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

/**
 * @brief Class that stores different heterogenous objects in a unordered_map
 * and provides access to them.
 *
 * @tparam Key The objects are mapped based on the key type
 */
template <class Key> class UnorderedHeterogenousMap {
  /**
   * @brief Dummy base class for all the value objects
   */
  struct AbstractBase {
    /**
     * @brief Pure virtual destructor
     */
    virtual ~AbstractBase() = 0;
  };

  /**
   * @brief Wrapper subclass to wrap any class as
   * a subclass of common base class. This lets
   * us store these random objects in a typemap
   *
   * @tparam T type of the object being stored
   */
  template <class T> class AbstractBaseWrapper : public AbstractBase {
  public:
    /**
     * @brief Constructor to store the input data
     *
     * @param class object to store
     */
    AbstractBaseWrapper(T input) : input_(input) {}
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
   * Stores a map from Key to heterogenous objects. The heterogenous objects
   * are wrapped inside a container with a common base class. This allows
   * for extracting objects from the map using the key.
   */
  std::unordered_map<
      Key, std::tuple<const std::type_info *, std::unique_ptr<AbstractBase>>>
      configurations;
  // Add friend to access for testing AbstractBaseWrapper
  FRIEND_TEST(WrapperTests, SaveAndRetrieveObject);

public:
  /**
   * @brief Add a key value pair to the map
   *
   * @tparam Value the type of configuration object being stored
   * @param key the value of key used to map the value
   * @param value The configuration object to be stored
   */
  template <class Value> void insert(const Key &key, Value value) {
    std::unique_ptr<AbstractBase> value_wrapper(
        new AbstractBaseWrapper<Value>(value));
    configurations[key] =
        std::make_tuple(&typeid(Value), std::move(value_wrapper));
  }
  /**
   * @brief Get the value for a given key. If the key is not present,
   * throws a runtime_error.
   *
   * @tparam Value The type of configuration object to retrieve
   * @param key object used to map to the object needed
   *
   * @return A copy of the configuration object stored in typemap
   */
  template <class Value> Value find(const Key &key) const {
    auto it = configurations.find(key);
    if (it == configurations.end())
      throw std::runtime_error("Cannot find the target state key");
    auto &value_tuple = it->second;
    if (std::type_index(*std::get<0>(value_tuple)) !=
        std::type_index(typeid(Value))) {
      throw std::runtime_error("Value does not match with the type of stored");
    }
    if (std::get<1>(value_tuple).get() == nullptr)
      throw std::runtime_error("The value stored is a null ptr");
    auto value_wrapper = static_cast<AbstractBaseWrapper<Value> *>(
        std::get<1>(value_tuple).get());
    return value_wrapper->getInput();
  }
};

/**
 * @brief Default implementation for the destructor
 *
 * @tparam Key The key used for parent Heterogenous map
 */
template <class Key>
UnorderedHeterogenousMap<Key>::AbstractBase::~AbstractBase() {}
