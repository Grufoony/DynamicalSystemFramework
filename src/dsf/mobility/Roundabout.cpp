#include "Roundabout.hpp"

#include <format>
#include <stdexcept>

namespace dsf::mobility {
  Roundabout::Roundabout(const RoadJunction& node) : RoadJunction{node} {}

  std::unique_ptr<Agent> Roundabout::dequeue() {
    assert(!m_agents.empty());
    return m_agents.extract_front();
  }
}  // namespace dsf::mobility