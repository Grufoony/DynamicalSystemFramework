#include "Point.hpp"

#include <cmath>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace dsf::geometry {
  Point::Point(const std::string& strPoint, std::string_view const format) {
    if (format == "WKT") {
      auto start = strPoint.find('(');
      auto end = strPoint.find(')');
      if (start == std::string::npos || end == std::string::npos || end <= start) {
        throw std::invalid_argument("Invalid WKT POINT format: " + strPoint);
      }
      std::string coordStr = strPoint.substr(start + 1, end - start - 1);
      std::istringstream coordStream(coordStr);
      double x, y;
      if (!(coordStream >> x >> y)) {
        throw std::invalid_argument("Malformed WKT POINT coordinates: " + strPoint);
      }
      m_x = x;
      m_y = y;
    } else {
      throw std::invalid_argument(std::format("Unsupported format: {}", format));
    }
  }

  double haversine_km(dsf::geometry::Point const& p1,
                      dsf::geometry::Point const& p2) noexcept {
    constexpr double EARTH_RADIUS_KM = 6371.0;  // Earth radius in kilometers
    constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;

    double const lat1 = p1.y() * DEG_TO_RAD;
    double const lat2 = p2.y() * DEG_TO_RAD;
    double const dLat = lat2 - lat1;
    double const dLon = (p2.x() - p1.x()) * DEG_TO_RAD;

    double const a =
        std::sin(dLat * 0.5) * std::sin(dLat * 0.5) +
        std::cos(lat1) * std::cos(lat2) * std::sin(dLon * 0.5) * std::sin(dLon * 0.5);
    double const c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return EARTH_RADIUS_KM * c;
  }
}  // namespace dsf::geometry
