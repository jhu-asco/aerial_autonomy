#pragma once
// get type info for each controller hardware connector class
#include <typeindex>
// store objects
#include <unordered_map>

template <class GenericObjectT> class TypeMap {
  // Map to store the controller to hardware connectors
  std::unordered_map<std::type_index, GenericObjectT *> object_storage_map_;

public:
  // Set
  template <class ObjectT> void addObject(ObjectT &object) {
    object_storage_map_[typeid(ObjectT)] = &object;
  }
  // Get
  template <class ObjectT> ObjectT *getObject() {
    return static_cast<ObjectT *>(object_storage_map_[typeid(ObjectT)]);
  }
};
