
#include "Street.hpp"

#include <algorithm>
#include <spdlog/spdlog.h>

namespace dsf::mobility {
  std::function<double(Street const&)> Street::m_estimatedTravelTimeFunction =
      [](Street const& street) { return street.length() / street.maxSpeed(); };

  std::optional<tbb::concurrent_unordered_map<Id, std::vector<AgentStreetRecord>>>
      Street::m_agentData = std::nullopt;

  void Street::setEstimatedTravelTimeFunction(
      std::function<double(Street const&)> estimatedTravelTimeFunction) {
    if (!estimatedTravelTimeFunction) {
      throw std::invalid_argument(
          "Street::setEstimatedTravelTimeFunction: empty callable");
    }
    m_estimatedTravelTimeFunction = std::move(estimatedTravelTimeFunction);
  }

  std::function<double(Street const&)> const&
  Street::estimatedTravelTimeFunction() noexcept {
    return m_estimatedTravelTimeFunction;
  }

  Street::Street(Id id,
                 std::pair<Node*, Node*> nodePair,
                 double length,
                 double maxSpeed,
                 int nLanes,
                 std::string name,
                 geometry::PolyLine geometry,
                 std::optional<int> capacity,
                 double transportCapacity)
      : Road(id,
             std::move(nodePair),
             length,
             maxSpeed,
             nLanes,
             std::move(name),
             std::move(geometry),
             capacity,
             transportCapacity),
        m_lanes(static_cast<std::size_t>(nLanes)),
        m_movingAgents{dsf::priority_queue<std::unique_ptr<Agent>,
                                           std::vector<std::unique_ptr<Agent>>,
                                           AgentComparator>()} {}
  auto Street::operator==(Street const& other) const -> bool {
    bool isEqual{true};
    isEqual &= (this->m_id == other.m_id);
    isEqual &= (Edge::source()->id() == other.Edge::source()->id());
    isEqual &= (Edge::target()->id() == other.Edge::target()->id());
    isEqual &= (this->m_length == other.m_length);
    isEqual &= (this->m_maxSpeed == other.m_maxSpeed);
    isEqual &= (this->m_nLanes == other.m_nLanes);
    isEqual &= (this->m_name == other.m_name);
    isEqual &= (this->m_hasPriority == other.m_hasPriority);
    return isEqual;
  }

  void Street::setLaneMapping(std::vector<Direction> const& laneMapping) {
    if (laneMapping.size() != static_cast<size_t>(m_nLanes)) {
      throw std::invalid_argument(
          std::format("Lane mapping size ({}) does not match number of lanes ({})",
                      laneMapping.size(),
                      m_nLanes));
    }
    for (std::size_t i{0}; i < m_lanes.size(); ++i) {
      m_lanes[i].direction = laneMapping[i];
    }
    spdlog::debug("New lane mapping for street {}", *this);
  }
  void Street::setQueue(dsf::queue<std::unique_ptr<Agent>> queue, size_t index) {
    assert(index < m_lanes.size());
    m_lanes[index].queue = std::move(queue);
  }
  void Street::changeNLanes(int const nLanes, std::optional<double> const speedFactor) {
    if (this->nExitingAgents() > 0) {
      spdlog::warn("Changing number of lanes for {} which has {} exiting agents",
                   *this,
                   this->nExitingAgents());
    }
    if (nLanes <= 0) {
      throw std::invalid_argument("Number of lanes must be positive");
    }
    if (nLanes == m_nLanes) {
      return;
    }
    spdlog::debug(
        "Changing number of lanes for {} from {} to {}", *this, m_nLanes, nLanes);
    m_capacity = static_cast<int>(m_capacity * static_cast<double>(nLanes) / m_nLanes);
    m_transportCapacity = m_transportCapacity * nLanes / m_nLanes;
    auto const previousNLanes = m_nLanes;
    m_nLanes = nLanes;
    if (speedFactor.has_value()) {
      m_maxSpeed *= *speedFactor;
      m_bumpRoutingRevision();
    }
    if (nLanes < previousNLanes) {
      // Resizing down would destroy the agents queued in the removed lanes without
      // anyone noticing. Move them to the last surviving lane instead.
      auto& survivingQueue{m_lanes[static_cast<std::size_t>(nLanes) - 1].queue};
      for (auto i{static_cast<std::size_t>(nLanes)}; i < m_lanes.size(); ++i) {
        while (!m_lanes[i].queue.empty()) {
          survivingQueue.push(m_lanes[i].queue.extract_front());
        }
      }
    }
    m_lanes.resize(static_cast<std::size_t>(m_nLanes));
    // The old mapping does not fit the new lanes
    for (auto& lane : m_lanes) {
      lane.direction = Direction::ANY;
    }
  }
  void Street::enableCounter(std::string name, CounterPosition position) {
    if (m_counter.has_value()) {
      throw std::runtime_error(
          std::format("{} already has a counter named {}", *this, m_counter->name()));
    }
    if (name.empty()) {
      name = std::format("Coil_{}", m_id);
    }
    m_counter.emplace();
    m_counter->setName(name);
    m_counterPosition = position;
  }
  void Street::resetCounter() {
    if (!hasCoil()) {
      throw std::runtime_error(
          std::format("Cannot reset counter for {} which does not have a coil.", *this));
    }
    m_counter->reset();
  }

  void Street::addAgent(std::unique_ptr<Agent> pAgent, std::time_t const currentTime) {
    assert(!isFull());
    spdlog::trace("Adding {} on {}", *pAgent, *this);
    m_agentsInsertionTimes[pAgent->id()] = currentTime;
    m_movingAgents.push(std::move(pAgent));
    if (m_counter.has_value() && m_counterPosition == CounterPosition::ENTRY) {
      ++(*m_counter);
    }
  }
  std::unique_ptr<Agent> Street::dequeueMovingAgent() {
    assert(!m_movingAgents.empty());
    return m_movingAgents.extract_top();
  }
  void Street::enqueue(std::size_t const& queueId) {
    assert(!m_movingAgents.empty());
    m_movingAgents.top()->incrementDistance(m_length);
    m_lanes[queueId].queue.push(m_movingAgents.extract_top());
    if (m_counter.has_value() && m_counterPosition == CounterPosition::MIDDLE) {
      ++(*m_counter);
    }
  }
  std::unique_ptr<Agent> Street::dequeue(std::size_t const& index,
                                         std::time_t const currentTime) {
    auto& lane{m_lanes[index]};
    assert(!lane.queue.empty());
    auto pAgent{lane.queue.extract_front()};
    lane.flowAccumulator.consume();
    m_recordExit(*pAgent, currentTime);
    if (m_counter.has_value() && m_counterPosition == CounterPosition::EXIT) {
      ++(*m_counter);
    }
    return pAgent;
  }
  std::unique_ptr<Agent> Street::dequeueArrivedAgent(std::time_t const currentTime) {
    assert(!m_movingAgents.empty());
    auto pAgent{m_movingAgents.extract_top()};
    pAgent->incrementDistance(m_length);
    if (m_counter.has_value() && m_counterPosition == CounterPosition::MIDDLE) {
      ++(*m_counter);
    }
    m_recordExit(*pAgent, currentTime);
    return pAgent;
  }
  void Street::m_recordExit(Agent const& agent, std::time_t const currentTime) {
    // Keep track of average speed
    auto const itInsertion{m_agentsInsertionTimes.find(agent.id())};
    auto const insertionTime{
        itInsertion == m_agentsInsertionTimes.cend() ? currentTime : itInsertion->second};
    // An agent leaving in the same time-step it entered would divide by zero.
    if (currentTime > insertionTime) {
      m_avgSpeeds.push_back(m_length / (currentTime - insertionTime));
    }
    if (itInsertion != m_agentsInsertionTimes.cend()) {
      m_agentsInsertionTimes.erase(itInsertion);
    }
    if (m_agentData.has_value()) {
      m_agentData->operator[](m_id).push_back(AgentStreetRecord{
          .agentId = agent.id(), .timeIn = insertionTime, .timeOut = currentTime});
    }
  }

  std::size_t Street::nAgents() const {
    auto nAgents{m_movingAgents.size()};
    for (auto const& lane : m_lanes) {
      nAgents += lane.queue.size();
    }
    return nAgents;
  }

  std::vector<Direction> Street::laneMapping() const {
    std::vector<Direction> laneMapping;
    laneMapping.reserve(m_lanes.size());
    for (auto const& lane : m_lanes) {
      laneMapping.push_back(lane.direction);
    }
    return laneMapping;
  }

  std::size_t Street::nMovingAgents() const { return m_movingAgents.size(); }
  double Street::nExitingAgents(Direction direction, bool normalizeOnNLanes) const {
    double nAgents{0.};
    std::size_t n{0};
    for (auto const& lane : m_lanes) {
      auto const laneDirection{lane.direction};
      if (direction == Direction::ANY || laneDirection == direction ||
          (laneDirection == Direction::RIGHTANDSTRAIGHT &&
           (direction == Direction::RIGHT || direction == Direction::STRAIGHT)) ||
          (laneDirection == Direction::LEFTANDSTRAIGHT &&
           (direction == Direction::LEFT || direction == Direction::STRAIGHT)) ||
          (direction == Direction::RIGHTANDSTRAIGHT &&
           (laneDirection == Direction::RIGHT || laneDirection == Direction::STRAIGHT)) ||
          (direction == Direction::LEFTANDSTRAIGHT &&
           (laneDirection == Direction::LEFT || laneDirection == Direction::STRAIGHT))) {
        nAgents += lane.queue.size();
        ++n;
      }
    }
    if (normalizeOnNLanes && n > 1) {
      nAgents /= n;
    }
    return nAgents;
  }

};  // namespace dsf::mobility
