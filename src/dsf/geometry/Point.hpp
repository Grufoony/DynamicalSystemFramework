#pragma once

#include <cmath>
#include <format>
#include <limits>
#include <string>

namespace dsf::geometry {
  class Point {
  private:
    double m_x;
    double m_y;

  public:
    /// @brief Construct a Point with given x and y coordinates.
    /// @param x The x coordinate
    /// @param y The y coordinate
    Point(double x, double y) : m_x(x), m_y(y) {}
    /// @brief Construct a Point from a string representation.
    /// @param strPoint The string representation of the point.
    /// @param format The format of the string representation. Default is "WKT".
    /// @throws std::invalid_argument if the format is not supported or the string is invalid.
    Point(std::string const& strPoint, std::string const& format = "WKT");
    /// @brief Equality operator for Point
    inline bool operator==(const Point& other) const {
      return std::abs(m_x - other.m_x) < std::numeric_limits<double>::epsilon() &&
             std::abs(m_y - other.m_y) < std::numeric_limits<double>::epsilon();
    }
    /// @brief Support for structured bindings, e.g., auto const& [x, y] = point;
    template <std::size_t Index>
    inline double const& get() const {
      if constexpr (Index == 0)
        return m_x;
      else if constexpr (Index == 1)
        return m_y;
    }

    inline double const& x() const { return m_x; }
    inline double const& y() const { return m_y; }
  };

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

  template <typename FormatContext>
  auto format(dsf::geometry::Point const& point, FormatContext& ctx) const {
    return std::format_to(ctx.out(), "POINT ({} {})", point.x(), point.y());
  }
};

// Structured binding support for dsf::geometry::Point
namespace std {
  template <>
  struct tuple_size<dsf::geometry::Point> : std::integral_constant<std::size_t, 2> {};

  template <>
  struct tuple_element<0, dsf::geometry::Point> {
    using type = double;
  };

  template <>
  struct tuple_element<1, dsf::geometry::Point> {
    using type = double;
  };
}  // namespace std

// ADL-based get for structured bindings
template <std::size_t I>
inline double const& get(dsf::geometry::Point const& point) {
  return point.get<I>();
}