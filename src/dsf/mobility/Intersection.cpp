#include "Intersection.hpp"

#include <limits>
#include <stdexcept>

namespace dsf::mobility {
  void Intersection::setCapacity(std::size_t const capacity) {
    if (capacity < this->nAgents()) {
      throw std::runtime_error(std::format(
          "Intersection capacity ({}) is smaller than the current queue size ({}).",
          capacity,
          this->nAgents()));
    }
    RoadJunction::setCapacity(capacity);
  }

  void Intersection::addAgent(double angle, std::unique_ptr<Agent> pAgent) {
    if (isFull()) {
      throw std::runtime_error(std::format("{} is full.", *this));
    }
    auto iAngle{static_cast<int16_t>(angle * 100)};
    m_agents.emplace(iAngle, std::move(pAgent));
  }

  void Intersection::addAgent(std::unique_ptr<Agent> pAgent) {
    if (isFull()) {
      throw std::runtime_error(std::format("{} is full.", *this));
    }
    int16_t lastKey{0};
    if (!m_agents.empty()) {
      lastKey = m_agents.rbegin()->first;
      if (lastKey == std::numeric_limits<int16_t>::max()) {
        throw std::runtime_error(std::format(
            "{} cannot order any further agent: the queue key saturated.", *this));
      }
      ++lastKey;
    }
    m_agents.emplace(lastKey, std::move(pAgent));
  }
}  // namespace dsf::mobility