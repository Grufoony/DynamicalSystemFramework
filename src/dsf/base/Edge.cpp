#include "Edge.hpp"
#include "Node.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>

namespace dsf {
  Edge::Edge(Id id, std::pair<Node*, Node*> nodePair, geometry::PolyLine geometry)
      : m_geometry{std::move(geometry)}, m_id(id), m_nodePair(nodePair) {
    if (m_nodePair.first == nullptr || m_nodePair.second == nullptr) {
      throw std::invalid_argument(std::format(
          "Edge {} must be built from non-null source and target nodes.", id));
    }
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
  void Edge::setAttribute(
      std::string const& name,
      std::variant<std::monostate, bool, std::int64_t, double, std::string> const& value) {
    m_attributes[name] = value;
  }
  void Edge::setSource(Node* pNode) {
    if (pNode == nullptr || pNode->id() != source()->id()) {
      throw std::invalid_argument(
          std::format("Invalid source node for edge {}: expected node with id {}.",
                      m_id,
                      source()->id()));
    }
    m_nodePair.first = pNode;
  }
  void Edge::setTarget(Node* pNode) {
    if (pNode == nullptr || pNode->id() != target()->id()) {
      throw std::invalid_argument(
          std::format("Invalid target node for edge {}: expected node with id {}.",
                      m_id,
                      target()->id()));
    }
    m_nodePair.second = pNode;
  }
  void Edge::setReverse(Edge* pEdge) {
    if (pEdge != nullptr && (pEdge->source()->id() != target()->id() ||
                             pEdge->target()->id() != source()->id())) {
      throw std::invalid_argument(
          std::format("Edge {} ({} -> {}) is not the reverse of edge {} ({} -> {}).",
                      pEdge->id(),
                      pEdge->source()->id(),
                      pEdge->target()->id(),
                      m_id,
                      source()->id(),
                      target()->id()));
    }
    m_pReverse = pEdge;
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