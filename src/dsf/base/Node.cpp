#include "Node.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>

namespace dsf {
  void Node::addIngoingEdge(Edge* pEdge) {
    if (pEdge == nullptr || pEdge->target() != this) {
      throw std::invalid_argument(
          std::format("Invalid ingoing edge for node with id {}.", m_id));
    }
    if (std::ranges::find(m_ingoingEdges, pEdge->id(), &Edge::id) !=
        m_ingoingEdges.cend()) {
      throw std::invalid_argument(std::format(
          "Edge with id {} already exists in the incoming edges of node with id {}.",
          pEdge->id(),
          m_id));
    }
    m_ingoingEdges.push_back(pEdge);
  }
  void Node::addOutgoingEdge(Edge* pEdge) {
    if (pEdge == nullptr || pEdge->source() != this) {
      throw std::invalid_argument(
          std::format("Invalid outgoing edge for node with id {}.", m_id));
    }
    if (std::ranges::find(m_outgoingEdges, pEdge->id(), &Edge::id) !=
        m_outgoingEdges.cend()) {
      throw std::invalid_argument(std::format(
          "Edge with id {} already exists in the outgoing edges of node with id {}.",
          pEdge->id(),
          m_id));
    }
    m_outgoingEdges.push_back(pEdge);
  }
};  // namespace dsf
