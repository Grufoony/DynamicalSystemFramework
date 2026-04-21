#include "Edge.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>

namespace dsf {
  Edge::Edge(Id id, std::pair<Id, Id> nodePair, geometry::PolyLine geometry)
      : m_geometry{std::move(geometry)}, m_id(id), m_nodePair(nodePair) {
    if (m_geometry.size() > 1) {
      m_setAngle(m_geometry[m_geometry.size() - 2], m_geometry.back());
    } else {
      m_angle = 0.;
    }
  }

  void Edge::m_setAngle(geometry::Point srcNodeCoordinates,
                        geometry::Point dstNodeCoordinates) {
    // N.B.: lat, lon <==> y, x
    double const dy{dstNodeCoordinates.y() - srcNodeCoordinates.y()};
    double const dx{dstNodeCoordinates.x() - srcNodeCoordinates.x()};
    m_angle = std::atan2(dy, dx);
    if (m_angle < 0.) {
      m_angle += 2 * std::numbers::pi;
    }
    assert(!(std::abs(m_angle) > 2 * std::numbers::pi));
  }

  void Edge::setGeometry(geometry::PolyLine geometry) {
    m_geometry = std::move(geometry);
    if (m_geometry.size() > 1) {
      m_setAngle(m_geometry[m_geometry.size() - 2], m_geometry.back());
    } else {
      m_angle = 0.;
    }
  }
  void Edge::setAttribute(std::string const& name, std::any const& value) {
    m_attributes[name] = value;
  }

  double Edge::deltaAngle(double const previousEdgeAngle) const {
    double deltaAngle{m_angle - previousEdgeAngle};
    if (deltaAngle > std::numbers::pi) {
      deltaAngle -= 2 * std::numbers::pi;
    } else if (deltaAngle < -std::numbers::pi) {
      deltaAngle += 2 * std::numbers::pi;
    }
    return deltaAngle;
  }
};  // namespace dsf