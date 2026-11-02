#include "RoadJunction.hpp"

namespace dsf::mobility {
  RoadJunction::RoadJunction(Id id) : Node(id), m_capacity{1}, m_transportCapacity{1.} {}
  RoadJunction::RoadJunction(Id id, geometry::Point point)
      : Node(id, point), m_capacity{1}, m_transportCapacity{1.} {}
  RoadJunction::RoadJunction(RoadJunction const& other)
      : Node(other),
        m_capacity{other.m_capacity},
        m_transportCapacity{other.m_transportCapacity},
        m_flowAccumulator{other.m_flowAccumulator} {}

  void RoadJunction::setCapacity(std::size_t const capacity) { m_capacity = capacity; }
  void RoadJunction::setTransportCapacity(double capacity) {
    assert(capacity > 0.);
    m_transportCapacity = capacity;
  }
}  // namespace dsf::mobility