#pragma once
// get type info for each storage object
#include <typeindex>
// store objects
#include <unordered_map>

/**
* @brief Store objects with a common base class
*
* @tparam GenericObjectT Common base class for objects stored
*/
template <class GenericObjectT> class TypeMap {
  /**
  * @brief Map to store the objects using the type of object as key
  * \todo Gowtham Maybe use shared/unique pointers
  */
  std::unordered_map<std::type_index, GenericObjectT *> object_storage_map_;

public:
  /**
  * @brief Store the object with base class as GenericObjectT
  *
  * @tparam ObjectT The type of object being stored. ObjectT should
  * be base class of GenericObjectT
  * @param object The address of an instance of ObjectT being stored
  */
  template <class ObjectT> void setObject(ObjectT &object) {
    object_storage_map_[typeid(ObjectT)] = &object;
  }
  /**
  * @brief return the object stored
  *
  * @tparam ObjectT type of object to be retrieved.
  *
  * @return  pointer to the instance of the object.
  */
  template <class ObjectT> ObjectT *getObject() {
    return static_cast<ObjectT *>(object_storage_map_[typeid(ObjectT)]);
  }
};
