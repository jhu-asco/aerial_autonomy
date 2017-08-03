#pragma once
#include <iomanip> // precision
#include <sstream>
#include <stdexcept>
#include <string>

/**
 * @brief Helper class to write html tables to text format
 */
class HtmlTableWriter {
public:
  /**
   * @brief constructor initializes the table header
   */
  HtmlTableWriter() : table_ended(false), row_ended(true) {
    table_string_stream << std::fixed << std::setprecision(2);
    table_string_stream << "<table border=\"1\">";
  }

  /**
   * @brief Add a cell to the table. Should be called after
   * beginning a row. Otherwise will throw an exception
   *
   * @tparam DataT Type of data to add to cell
   * @param data value of the data to add
   * @param header If header is provided it is added along with data to same
   * cell
   * @param bg_color Background color of the cell to use (Default white) The
   * color
   *        is provided as a hexcode #RGB where R, G, B go from 0 to F
   */
  template <typename DataT>
  void addCell(const DataT &data, std::string header = "",
               std::string bg_color = "#FFF") {
    if (row_ended) {
      throw std::runtime_error("Cannot add cell without beginning row");
    }
    table_string_stream << ("<td width=\"120\" bgcolor=\"" + bg_color + "\" >");
    if (!header.empty()) {
      table_string_stream << header << ": ";
    }
    table_string_stream << data;
    table_string_stream << "</td>";
  }

  /**
   * @brief Begin a new row in the table
   */
  void beginRow() {
    if (!row_ended) {
      endRow();
    }
    table_string_stream << "<tr>";
    row_ended = false;
  }

  /**
   * @brief Add a table header. Should be called after beginning a row
   *
   * This is similar to a addCell but is bold and is placed at the top of the
   * table
   *
   * @param header Header value to add.
   */
  void addHeader(std::string header) {
    if (row_ended) {
      throw std::runtime_error("Cannot add cell without beginning row");
    }
    table_string_stream << "<th>";
    table_string_stream << header;
    table_string_stream << "</th>";
  }

  /**
   * @brief Get the html table in string format
   *
   * @return string of html table
   */
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
  bool table_ended; ///< Flag to check if table has been completed
  bool row_ended;   ///< Flag to check if row has ended

  /**
   * @brief Add the table footer
   */
  void endTable() {
    if (!row_ended) {
      endRow();
    }
    table_ended = true;
    table_string_stream << "</table>";
  }
  /**
   * @brief Add row footer
   */
  void endRow() {
    row_ended = true;
    table_string_stream << "</tr>";
  }
};
