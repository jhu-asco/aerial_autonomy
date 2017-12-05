#include <gtest/gtest.h>

#include <fcntl.h>
#include <fstream>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "aerial_autonomy/common/proto_utils.h"
#include "position.pb.h"

using namespace proto_utils;

TEST(ProtoUtils, LoadProtoFromTextBadFile) {
  config::Position p;
  ASSERT_FALSE(loadProtoText("/fake", p));
}

TEST(ProtoUtils, LoadProtoFromTextBadProto) {
  config::Position p, p_load;
  p.set_x(1);
  std::string file_name("/tmp/test.pbtxt");
  std::remove(file_name.c_str());
  std::fstream fs(file_name, std::fstream::out);
  ASSERT_TRUE(fs.is_open());
  fs << "garbage";
  fs.close();
  ASSERT_FALSE(loadProtoText(file_name, p_load));
}

TEST(ProtoUtils, LoadProtoFromText) {
  config::Position p, p_load;
  p.set_x(1);

  std::string file_name("/tmp/test2.pbtxt");
  std::remove(file_name.c_str());
  std::fstream fs(file_name, std::fstream::out);
  ASSERT_TRUE(fs.is_open());
  fs << "x: 1";
  fs.close();

  ASSERT_TRUE(loadProtoText(file_name, p_load));
  ASSERT_EQ(p_load.x(), p.x());
}

TEST(Contains, Empty) {
  google::protobuf::RepeatedField<int> list = {};
  ASSERT_FALSE(proto_utils::contains(list, 0));
}

TEST(Contains, SizeOne) {
  google::protobuf::RepeatedField<int> list;
  list.Add(0);
  ASSERT_TRUE(proto_utils::contains(list, 0));
  ASSERT_FALSE(proto_utils::contains(list, 1));
}

TEST(Contains, General) {
  google::protobuf::RepeatedField<int> list;
  list.Add(4);
  list.Add(0);
  list.Add(-1);
  list.Add(3);
  list.Add(5);
  list.Add(5);
  list.Add(8);
  list.Add(10);
  ASSERT_TRUE(proto_utils::contains(list, 4));
  ASSERT_TRUE(proto_utils::contains(list, 0));
  ASSERT_TRUE(proto_utils::contains(list, -1));
  ASSERT_TRUE(proto_utils::contains(list, 3));
  ASSERT_TRUE(proto_utils::contains(list, 5));
  ASSERT_TRUE(proto_utils::contains(list, 8));
  ASSERT_TRUE(proto_utils::contains(list, 10));
  ASSERT_FALSE(proto_utils::contains(list, 1));
  ASSERT_FALSE(proto_utils::contains(list, -2));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
