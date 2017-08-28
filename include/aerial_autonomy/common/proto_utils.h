#pragma once

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

namespace proto_utils {

/**
* @brief Load a protobuf text file
* @param path Path of the file to load
* @param proto Returned proto object
* @return True if loaded successfully, false otherwise
* @tparam T Class of proto to load
*/
template <class T> bool loadProtoText(std::string path, T &proto) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    return false;
  }
  google::protobuf::io::FileInputStream fstream(fd);
  if (!google::protobuf::TextFormat::Parse(&fstream, &proto)) {
    return false;
  }
  return true;
}
}
