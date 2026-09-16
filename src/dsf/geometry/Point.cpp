#include "Point.hpp"

#include <boost/geometry/io/wkt/read.hpp>
#include <boost/geometry/strategies/spherical/distance_haversine.hpp>

#include <cstdlib>
#include <format>
#include <stdexcept>

namespace dsf::geometry {
  namespace detail {
    std::size_t countWktCoordinates(std::string const& wkt) {
      auto const open = wkt.find('(');
      auto const close = wkt.rfind(')');
      if (open == std::string::npos || close == std::string::npos || close <= open) {
        return 0u;
      }
      std::size_t count{0u};
      char const* cursor = wkt.c_str() + open + 1;
      char const* const bodyEnd = wkt.c_str() + close;
      while (cursor < bodyEnd) {
        char* next{};
        std::strtod(cursor, &next);
        if (next == cursor) {
          // Not the start of a number: skip this separator character.
          ++cursor;
          continue;
        }
        ++count;
        cursor = next;
      }
      return count;
    }
  }  // namespace detail

  Point pointFromWkt(std::string const& wkt) {
    Point point;
    try {
      boost::geometry::read_wkt(wkt, point);
    } catch (boost::geometry::exception const& e) {
      // read_wkt throws read_wkt_exception, which derives from std::exception but
      // not from std::invalid_argument. Translate it to keep the exception contract
      // this function has always documented.
      throw std::invalid_argument(
          std::format("Invalid WKT POINT '{}': {}", wkt, e.what()));
    }
    if (auto const nCoordinates = detail::countWktCoordinates(wkt); nCoordinates != 2u) {
      throw std::invalid_argument(std::format(
          "Invalid WKT POINT '{}': expected 2 coordinates, found {}", wkt, nCoordinates));
    }
    return point;
  }

  double haversine_km(Point const& p1, Point const& p2) noexcept {
    constexpr double EARTH_RADIUS_KM = 6371.0;
    // Invoke the strategy directly rather than going through boost::geometry::distance:
    // that skips the umbrella-strategy dispatch, and the result comes back in whatever
    // units the radius was given in, i.e. kilometres.
    return boost::geometry::strategy::distance::haversine<double>(EARTH_RADIUS_KM)
        .apply(p1, p2);
  }
}  // namespace dsf::geometry
