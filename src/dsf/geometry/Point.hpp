#pragma once

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <cstddef>
#include <format>
#include <string>

namespace dsf::geometry {
  /// @brief A 2D geographic point, in degrees, with x = longitude and y = latitude.
  /// @details This is an alias for a Boost.Geometry point model, so every
  ///          Boost.Geometry algorithm (distance, within, simplify, index::rtree, ...)
  ///          accepts it directly.
  ///
  ///          The coordinate system is spherical-equatorial rather than geographic
  ///          because distances in this library are great-circle (haversine) on a
  ///          sphere of radius 6371 km; see haversine_km. Switching to
  ///          cs::geographic would make the default strategy ellipsoidal and return
  ///          metres, which would silently change clustering results in dsf::mdt.
  ///
  ///          Note that, unlike a hand-written type, the default constructor leaves
  ///          both coordinates uninitialised.
  using Point = boost::geometry::model::d2::
      point_xy<double, boost::geometry::cs::spherical_equatorial<boost::geometry::degree>>;

  namespace detail {
    /// @brief Count the numeric tokens between the outermost parentheses of a WKT
    ///        string.
    /// @param wkt The WKT string.
    /// @return The number of coordinates found, or 0 if there are no parentheses.
    /// @details boost::geometry::read_wkt silently substitutes zero for coordinates
    ///          it does not find, so "POINT (1)" reads as (1, 0) and
    ///          "LINESTRING (1,2)" as the two points (1, 0) and (2, 0). Comparing
    ///          this count against the expected two-per-point rejects those inputs
    ///          instead of importing corrupt geometry.
    std::size_t countWktCoordinates(std::string const& wkt);
  }  // namespace detail

  /// @brief Construct a Point from its WKT representation, e.g. "POINT (1 2)".
  /// @param wkt The WKT string. The POINT keyword is required, and both trailing
  ///        tokens and missing coordinates are rejected.
  /// @return The parsed Point.
  /// @throws std::invalid_argument if the string is not a valid WKT POINT.
  Point pointFromWkt(std::string const& wkt);

  /// @brief Compute the Haversine distance between two geographic points.
  /// @param p1 The first point (longitude, latitude)
  /// @param p2 The second point (longitude, latitude)
  /// @return The distance in kilometers.
  double haversine_km(Point const& p1, Point const& p2) noexcept;
}  // namespace dsf::geometry

// Specialization of std::formatter for dsf::geometry::Point
template <>
struct std::formatter<dsf::geometry::Point> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  // N.B. deliberately not boost::geometry::wkt(): that writes through operator<<
  // without setting the stream precision, so it truncates coordinates to the
  // default 6 significant digits (~10 m of error), and it omits the space after
  // the keyword. std::format emits the shortest round-trippable representation of
  // a double, which is what pointFromWkt and every CSV fixture expect.
  template <typename FormatContext>
  auto format(dsf::geometry::Point const& point, FormatContext& ctx) const {
    return std::format_to(ctx.out(), "POINT ({} {})", point.x(), point.y());
  }
};
