#include <aerial_autonomy/common/html_utils.h>
#include <gtest/gtest.h>
#include <iostream>

/// \brief TEST
/// Simple Html table tests
TEST(HtmlTableWriterTests, SimpleTable) {
  HtmlTableWriter table_writer;
  table_writer.beginRow();
  table_writer.addHeader("Header");
  table_writer.beginRow();
  double data = 2.0;
  table_writer.addCell(data, "Hello");
  table_writer.addCell("Data");
  table_writer.addCell("Data", "Hello", Colors::white);
  ///\todo Add an automatic way of validating html string
  std::cout << "Table: \n" << table_writer.getTableString() << std::endl;
}
TEST(HtmlTableWriterTests, TableError) {
  HtmlTableWriter table_writer;
  EXPECT_THROW(table_writer.addHeader("Hello"), std::runtime_error);
  EXPECT_THROW(table_writer.addCell("Hello"), std::runtime_error);
}
/// Simple Html division tests
TEST(HtmlTableWriterTests, SimpleDivision) {
  HtmlDivisionWriter division_writer;
  division_writer.addHeader("Hello", 1);
  division_writer.addText("My text");
  std::cout << "Division: \n" << division_writer.getDivisionText() << std::endl;
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
