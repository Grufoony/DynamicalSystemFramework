#include "PolyLine.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace dsf::geometry {
  PolyLine::PolyLine(const std::string& strLine, std::string_view const format) {
    if (format == "WKT") {
      auto start = strLine.find('(');
      auto end = strLine.find(')');
      if (start == std::string::npos || end == std::string::npos || end <= start) {
        throw std::invalid_argument("Invalid WKT LINESTRING format: " + strLine);
      }
      std::string coordsStr = strLine.substr(start + 1, end - start - 1);
      std::istringstream coordsStream(coordsStr);
      // Count the number of ',' to estimate points
      std::size_t nPoints = std::ranges::count(coordsStr, ',') + 1;
      this->reserve(nPoints);
      std::string pointStr;
      while (std::getline(coordsStream, pointStr, ',')) {
        std::istringstream pointStream(pointStr);
        double x, y;
        std::string extra;
        if (!(pointStream >> x >> y)) {
          throw std::invalid_argument("Malformed WKT LINESTRING point: " + pointStr);
        }
        // Should not be any extra tokens after two numbers
        if (pointStream >> extra) {
          throw std::invalid_argument("Too many values in WKT LINESTRING point: " +
                                      pointStr);
        }
        this->push_back(Point{x, y});
      }
    } else {
      throw std::invalid_argument(std::format("Unsupported PolyLine format: {}", format));
    }
  }
}  // namespace dsf::geometry
