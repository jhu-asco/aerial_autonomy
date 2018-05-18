#pragma once

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/text_format.h>

/**
 * @brief Namespace for functions that operate/manipulate
 * google protobuf files
 */
namespace proto_utils {

/**
* @brief Check if a proto repeated contains a value
* @param list List to check for x
* @param x Item to check for in list
* @return True if list contains x, false otherwise
*/
template <typename T>
bool contains(const google::protobuf::RepeatedField<T> &list, T x) {
  for (auto y : list) {
    if (y == x) {
      return true;
    }
  }
  return false;
}

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
