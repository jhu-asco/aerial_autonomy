#pragma once
#include <iomanip> // precision
#include <sstream>
#include <stdexcept>
#include <string>

/**
 * @brief Helper class that defines colors in hex format
 */
struct Colors {
  /**
   * @brief Red color hex code
   */
  static constexpr const char *red = "\"#DC143C\"";
  /**
   * @brief Green color hex code
   */
  static constexpr const char *green = "\"#7FFF00\"";
  /**
   * @brief Blue color hex code
   */
  static constexpr const char *blue = "\"#4169E1\"";
  /**
   * @brief Grey color hex code
   */
  static constexpr const char *grey = "\"#D3D3D3\"";
  /**
   * @brief Yellow color hex code
   */
  static constexpr const char *yellow = "\"#FFFF94\"";
  /**
   * @brief White color hex code
   */
  static constexpr const char *white = "\"#FFF\"";
  /**
   * @brief Black color hex code
   */
  static constexpr const char *black = "\"#000\"";
};

/**
 * @brief Add a division to html
 */
class HtmlDivisionWriter {
public:
  /**
   * @brief Constructor
   */
  HtmlDivisionWriter() {
    division_string_stream << std::fixed << std::setprecision(2);
    division_string_stream << "<div>";
  }
  /**
   * @brief Add a header inside division
   *
   * @param header Text to put in header
   * @param level  The integer level can be 1 to 5 corresponding
   *               to h1 to h5
   */
  void addHeader(std::string header, int level = 1) {
    std::string h_name = "h" + std::to_string(level);
    division_string_stream << "<" << h_name << " align=\"center\" >" << header
                           << "</" << h_name << ">";
  }

  /**
   * @brief Add text to the division
   *
   * @param text string to add
   */
  void addText(std::string text) {
    division_string_stream << text << std::endl;
  }

  /**
   * @brief Create a divsion html in string format
   *
   * @return  return string of html division
   */
  std::string getDivisionText() {
    return division_string_stream.str() + "</div>";
  }

private:
  std::stringstream division_string_stream; ///< Html table in string format
};

/**
 * @brief Helper class to write html tables to text format
 */
class HtmlTableWriter {
public:
  /**
   * @brief constructor initializes the table header
   * @param width The width of each column in the table in px
   */
  HtmlTableWriter(int width = 120) : width_(width), row_ended(true) {
    table_string_stream << std::fixed << std::setprecision(3);
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
   * @param colspan  The number of columns to use for the cell
   */
  template <typename DataT>
  void addCell(const DataT &data, std::string header = "",
               std::string bg_color = Colors::white, int colspan = 1) {
    if (row_ended) {
      throw std::runtime_error("Cannot add cell without beginning row");
    }
    std::string width_text = "\"" + std::to_string(colspan * width_) + "\"";
    table_string_stream << "<td width=" << width_text << " colspan=\""
                        << std::to_string(colspan) << "\" bgcolor=" << bg_color
                        << " >";
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
   * @param text_color The color of the text in header. Default is black
   */
  void addHeader(std::string header, std::string text_color = Colors::black,
                 int colspan = 1) {
    if (row_ended) {
      throw std::runtime_error("Cannot add cell without beginning row");
    }
    std::string width_text = "\"" + std::to_string(colspan * width_) + "\"";
    table_string_stream << "<th width=" << width_text << " colspan=\""
                        << std::to_string(colspan) << "\" >";
    table_string_stream << "<font color=" << text_color << ">" << header
                        << "</font>";
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
    return table_string_stream.str() + "</table>";
  }

private:
  std::stringstream table_string_stream; ///< Html table in string format
  int width_;                            ///< Text to input for width of cell
  bool row_ended;                        ///< Flag to check if row has ended

  /**
   * @brief Add row footer
   */
  void endRow() {
    row_ended = true;
    table_string_stream << "</tr>";
  }
};
