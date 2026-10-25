#pragma once

#include <format>
#include <string>
#include <string_view>
#include <vector>

#include "Point.hpp"

namespace dsf::geometry {
  /// @brief A polyline represented as a vector of Points
  class PolyLine : public std::vector<Point> {
  public:
    using std::vector<Point>::vector;  // Inherit constructors
    /// @brief Construct a PolyLine from a string representation.
    /// @param strLine The string representation of the polyline.
    /// @param format The format of the string representation. Default is "WKT".
    /// @throws std::invalid_argument if the format is not supported or the string is invalid.
    PolyLine(std::string const& strLine, std::string_view const format = "WKT");
  };
}  // namespace dsf::geometry

// Specialization of std::formatter for dsf::geometry::PolyLine
template <>
struct std::formatter<dsf::geometry::PolyLine> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(dsf::geometry::PolyLine const& polyline, FormatContext&& ctx) const {
    auto out = std::format_to(ctx.out(), "LINESTRING (");
    for (std::size_t i = 0; i < polyline.size(); ++i) {
      if (i > 0) {
        out = std::format_to(out, ", ");
      }
      out = std::format_to(out, "{} {}", polyline[i].x(), polyline[i].y());
    }
    return std::format_to(out, ")");
  }
};
