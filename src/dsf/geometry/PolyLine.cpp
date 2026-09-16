#include "PolyLine.hpp"

#include <boost/geometry/io/wkt/read.hpp>

#include <format>
#include <stdexcept>

namespace dsf::geometry {
  PolyLine polyLineFromWkt(std::string const& wkt) {
    PolyLine polyline;
    try {
      boost::geometry::read_wkt(wkt, polyline);
    } catch (boost::geometry::exception const& e) {
      // See pointFromWkt: translate read_wkt_exception to preserve the documented
      // std::invalid_argument contract.
      throw std::invalid_argument(
          std::format("Invalid WKT LINESTRING '{}': {}", wkt, e.what()));
    }
    if (auto const nCoordinates = detail::countWktCoordinates(wkt);
        nCoordinates != 2u * polyline.size()) {
      throw std::invalid_argument(
          std::format("Invalid WKT LINESTRING '{}': expected {} coordinates for {} "
                      "points, found {}",
                      wkt,
                      2u * polyline.size(),
                      polyline.size(),
                      nCoordinates));
    }
    return polyline;
  }
}  // namespace dsf::geometry
