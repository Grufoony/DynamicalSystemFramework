/// @file       /src/dsf/utility/csv_writer.hpp
/// @brief      Helpers to write dsf's delimiter-separated output files.
///
/// @details    This file wraps the csv-parser writer with the conventions dsf uses for
///             its CSV artifacts, so that every component writing one agrees on the
///             separator and on how the output stream is opened.

#pragma once

#include <csv.hpp>

#include <format>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <string>

namespace dsf::utility {
  /// @brief The writer used for dsf's CSV artifacts.
  /// @details dsf CSV files are semicolon-separated. csv::DelimWriter takes the
  /// delimiter as a template parameter and only holds a reference to the output stream,
  /// so the caller owns the std::ofstream it is built on and must keep it alive for at
  /// least as long as the writer.
  using CSVRowWriter = csv::DelimWriter<std::ofstream, ';', '"'>;

  /// @brief Open a CSV file for writing
  /// @param fileName The path of the file to open
  /// @param mode The open mode. Defaults to appending, so that rows flushed over
  /// several calls accumulate in the same file
  /// @return std::ofstream The output stream, which must outlive any writer built on it
  /// @throws std::runtime_error If the file cannot be opened
  inline std::ofstream openCSVFile(std::string const& fileName,
                                   std::ios::openmode const mode = std::ios::app) {
    std::ofstream stream{fileName, mode};
    if (!stream.is_open()) {
      throw std::runtime_error(
          std::format("Failed to open CSV file for writing: {}", fileName));
    }
    return stream;
  }
}  // namespace dsf::utility
