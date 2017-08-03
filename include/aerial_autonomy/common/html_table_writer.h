#pragma once
#include <iomanip> // precision
#include <sstream>
#include <stdexcept>
#include <string>

class HtmlTableWriter {
public:
  HtmlTableWriter() : table_ended(false), row_ended(true) {
    table_string_stream << std::fixed << std::setprecision(2);
    table_string_stream << "<table border=\"1\" width=\"100\%\">";
  }

  template <typename DataT>
  void addCell(const DataT &data, std::string header = "",
               std::string bg_color = "#FFF") {
    if (row_ended) {
      throw std::runtime_error("Cannot add cell without beginning row");
    }
    table_string_stream << ("<td bgcolor=\"" + bg_color + "\" >");
    if (!header.empty()) {
      table_string_stream << header << ": ";
    }
    table_string_stream << data;
    table_string_stream << "</td>";
  }

  void beginRow() {
    if (!row_ended) {
      endRow();
    }
    table_string_stream << "<tr>";
    row_ended = false;
  }

  void addHeader(std::string header) {
    if (row_ended) {
      throw std::runtime_error("Cannot add cell without beginning row");
    }
    table_string_stream << "<th>";
    table_string_stream << header;
    table_string_stream << "</th>";
  }

  std::string getTableString() {
    if (!row_ended) {
      endRow();
    }
    if (!table_ended) {
      endTable();
    }
    return table_string_stream.str();
  }

private:
  std::stringstream table_string_stream; ///< Html table in string format
  bool table_ended;
  bool row_ended;
  void endTable() {
    if (!row_ended) {
      endRow();
    }
    table_ended = true;
    table_string_stream << "</table>";
  }
  void endRow() {
    row_ended = true;
    table_string_stream << "</tr>";
  }
};
