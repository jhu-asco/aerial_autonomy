#include <aerial_autonomy/common/html_table_writer.h>
#include <gtest/gtest.h>
#include <iostream>

/// \brief TEST
/// Simple Html writer tests
TEST(HtmlTableWriterTests, SimpleTable) {
  HtmlTableWriter table_writer;
  table_writer.beginRow();
  double data = 2.0;
  table_writer.addCell("Hello", data);
  table_writer.endRow();
  table_writer.endTable();
  std::cout << "Table: \n" << table_writer.getTableString() << std::endl;
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
