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

  void Intersection::addAgent(double angle,
                              std::unique_ptr<Agent> pAgent,
                              std::optional<std::size_t> lane) {
    if (isFull()) {
      throw std::runtime_error(std::format("{} is full.", *this));
    }
    if (lane.has_value()) {
      if (!pAgent->streetId().has_value()) {
        throw std::invalid_argument(
            std::format("{} is on no street: it cannot come from lane {} into {}.",
                        *pAgent,
                        *lane,
                        *this));
      }
      auto const [_, bInserted] =
          m_laneOccupants.try_emplace({*pAgent->streetId(), *lane}, pAgent->id());
      if (!bInserted) {
        throw std::runtime_error(
            std::format("Lane {} of street {} already has an agent in {}.",
                        *lane,
                        *pAgent->streetId(),
                        *this));
      }
    }
    auto iAngle{static_cast<int16_t>(angle * 100)};
    m_agents.emplace(AgentKey{!m_fromMainRoad(*pAgent), iAngle}, std::move(pAgent));
  }

  void Intersection::addAgent(std::unique_ptr<Agent> pAgent) {
    if (isFull()) {
      throw std::runtime_error(std::format("{} is full.", *this));
    }
    int16_t lastKey{0};
    if (!m_agents.empty()) {
      lastKey = m_agents.rbegin()->first.second;
      if (lastKey == std::numeric_limits<int16_t>::max()) {
        throw std::runtime_error(std::format(
            "{} cannot order any further agent: the queue key saturated.", *this));
      }
      ++lastKey;
    }
    m_agents.emplace(AgentKey{!m_fromMainRoad(*pAgent), lastKey}, std::move(pAgent));
  }

  std::unique_ptr<Agent> Intersection::removeAgent(AgentMap::const_iterator it) {
    auto const agentId{it->second->id()};
    std::erase_if(m_laneOccupants,
                  [agentId](auto const& occupant) { return occupant.second == agentId; });
    auto node{m_agents.extract(it)};
    return std::move(node.mapped());
  }
}  // namespace dsf::mobility
