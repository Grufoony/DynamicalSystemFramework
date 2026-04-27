
#include "Street.hpp"

#include <algorithm>
#include <spdlog/spdlog.h>

namespace dsf::mobility {
  std::function<double(Street const&)> Street::m_estimatedTravelTimeFunction =
      [](Street const& street) { return street.length() / street.maxSpeed(); };

  std::optional<
      tbb::concurrent_unordered_map<Id,
                                    std::vector<std::tuple<Id, std::time_t, std::time_t>>>>
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

  void Street::m_updateLaneMapping(int const nLanes) {
    m_laneMapping.clear();
    switch (nLanes) {
      case 1:
        m_laneMapping.emplace_back(Direction::ANY);
        break;
      case 2:
        m_laneMapping.emplace_back(Direction::RIGHTANDSTRAIGHT);
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
      case 3:
        m_laneMapping.emplace_back(Direction::RIGHTANDSTRAIGHT);
        m_laneMapping.emplace_back(Direction::STRAIGHT);
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
      default:
        m_laneMapping.emplace_back(Direction::RIGHT);
        for (auto i{1}; i < nLanes - 1; ++i) {
          m_laneMapping.emplace_back(Direction::STRAIGHT);
        }
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
    }
  }
  Street::Street(Id id,
                 std::pair<Id, Id> nodePair,
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
        m_exitQueues{std::vector<dsf::queue<std::unique_ptr<Agent>>>(nLanes)},
        m_movingAgents{dsf::priority_queue<std::unique_ptr<Agent>,
                                           std::vector<std::unique_ptr<Agent>>,
                                           AgentComparator>()} {
    m_updateLaneMapping(nLanes);
  }
  auto Street::operator==(Street const& other) const -> bool {
    bool isEqual{true};
    isEqual &= (this->m_id == other.m_id);
    isEqual &= (this->m_nodePair == other.m_nodePair);
    isEqual &= (this->m_length == other.m_length);
    isEqual &= (this->m_maxSpeed == other.m_maxSpeed);
    isEqual &= (this->m_nLanes == other.m_nLanes);
    isEqual &= (this->m_name == other.m_name);
    isEqual &= (this->m_hasPriority == other.m_hasPriority);
    return isEqual;
  }

  void Street::setLaneMapping(std::vector<Direction> const& laneMapping) {
    assert(laneMapping.size() == static_cast<size_t>(m_nLanes));
    m_laneMapping = laneMapping;
    std::string strLaneMapping;
    std::for_each(
        laneMapping.cbegin(), laneMapping.cend(), [&strLaneMapping](auto const item) {
          strLaneMapping +=
              std::format("{} - ", directionToString[static_cast<size_t>(item)]);
        });
    spdlog::debug("New lane mapping for street {} -> {} is: {}",
                  m_nodePair.first,
                  m_nodePair.second,
                  strLaneMapping);
  }
  void Street::setQueue(dsf::queue<std::unique_ptr<Agent>> queue, size_t index) {
    assert(index < m_exitQueues.size());
    m_exitQueues[index] = std::move(queue);
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
    spdlog::info(
        "Changing number of lanes for {} from {} to {}", *this, m_nLanes, nLanes);
    m_capacity = static_cast<int>(m_capacity * static_cast<double>(nLanes) / m_nLanes);
    m_transportCapacity = m_transportCapacity * nLanes / m_nLanes;
    m_nLanes = nLanes;
    if (speedFactor.has_value()) {
      m_maxSpeed = m_maxSpeed * speedFactor.value();
    }
    m_exitQueues.resize(m_nLanes);
    m_updateLaneMapping(m_nLanes);
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
    m_exitQueues[queueId].push(m_movingAgents.extract_top());
    if (m_counter.has_value() && m_counterPosition == CounterPosition::MIDDLE) {
      ++(*m_counter);
    }
  }
  std::unique_ptr<Agent> Street::dequeue(std::size_t const& index,
                                         std::time_t const currentTime) {
    assert(!m_exitQueues[index].empty());
    auto pAgent{m_exitQueues[index].extract_front()};
    // Keep track of average speed
    auto const insertionTime{m_agentsInsertionTimes[pAgent->id()]};
    m_avgSpeeds.push_back(m_length / (currentTime - insertionTime));
    m_agentsInsertionTimes.erase(pAgent->id());
    if (m_agentData.has_value()) {
      m_agentData->operator[](m_id).emplace_back(
          pAgent->id(), insertionTime, currentTime);
    }

    if (m_counter.has_value() && m_counterPosition == CounterPosition::EXIT) {
      ++(*m_counter);
    }
    return pAgent;
  }

  std::size_t Street::nAgents() const {
    auto nAgents{m_movingAgents.size()};
    for (const auto& queue : m_exitQueues) {
      nAgents += queue.size();
    }
    return nAgents;
  }

  std::size_t Street::nMovingAgents() const { return m_movingAgents.size(); }
  double Street::nExitingAgents(Direction direction, bool normalizeOnNLanes) const {
    double nAgents{0.};
    std::size_t n{0};
    for (auto i{0}; i < m_nLanes; ++i) {
      if (direction == Direction::ANY) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (m_laneMapping[i] == direction) {
        nAgents += m_exitQueues[i].size();
      } else if (m_laneMapping[i] == Direction::RIGHTANDSTRAIGHT &&
                 (direction == Direction::RIGHT || direction == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (m_laneMapping[i] == Direction::LEFTANDSTRAIGHT &&
                 (direction == Direction::LEFT || direction == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (direction == Direction::RIGHTANDSTRAIGHT &&
                 (m_laneMapping[i] == Direction::RIGHT ||
                  m_laneMapping[i] == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (direction == Direction::LEFTANDSTRAIGHT &&
                 (m_laneMapping[i] == Direction::LEFT ||
                  m_laneMapping[i] == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      }
    }
    if (normalizeOnNLanes) {
      n > 1 ? nAgents /= n : nAgents;
    }
    return nAgents;
  }
  Measurement<double> Street::meanSpeed(bool const bReset) {
    auto const avgSpeed{Measurement<double>(m_avgSpeeds)};
    if (bReset) {
      m_avgSpeeds.clear();
    }
    return avgSpeed;
  }

};  // namespace dsf::mobility
