#pragma once

struct ObjectId {
  ObjectId(uint32_t id_) : id(id_) {}
  ObjectId() : ObjectId(0) {}
  uint32_t id = 0;
};
