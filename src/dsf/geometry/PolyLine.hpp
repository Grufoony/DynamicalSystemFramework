#pragma once

#include <boost/geometry/geometries/linestring.hpp>

#include <format>
#include <string>

#include "Point.hpp"

namespace dsf::geometry {
  /// @brief A polyline represented as a sequence of Points.
  /// @details This is an alias for a Boost.Geometry linestring model, which derives
  ///          publicly from std::vector<Point>, so the usual container interface
  ///          (size, empty, front, back, operator[], reserve, push_back,
  ///          emplace_back, iteration) and brace initialisation are all available.
  using PolyLine = boost::geometry::model::linestring<Point>;

  /// @brief Construct a PolyLine from its WKT representation, e.g.
  ///        "LINESTRING (1 2, 3 4)".
  /// @param wkt The WKT string. The LINESTRING keyword is required and trailing
  ///        tokens are rejected. "LINESTRING ()" yields an empty PolyLine.
  /// @return The parsed PolyLine.
  /// @throws std::invalid_argument if the string is not a valid WKT LINESTRING.
  PolyLine polyLineFromWkt(std::string const& wkt);
}  // namespace dsf::geometry

// Specialization of std::formatter for dsf::geometry::PolyLine
template <>
struct std::formatter<dsf::geometry::PolyLine> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  // N.B. deliberately not boost::geometry::wkt(); see the note on
  // std::formatter<dsf::geometry::Point> for why.
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
