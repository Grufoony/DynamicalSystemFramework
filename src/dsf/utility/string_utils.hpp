#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace dsf {
  /// @brief Convert a string to lowercase, in place
  /// @param str The string to convert
  inline void to_lower(std::string& str) {
    std::ranges::transform(
        str, str.begin(), [](unsigned char c) { return std::tolower(c); });
  }
}  // namespace dsf
