#include "Agent.hpp"

#include <utility>

namespace dsf::mobility {
  Agent::Agent(Id const id,
               std::time_t const& spawnTime,
               std::shared_ptr<Itinerary> itinerary,
               std::optional<Id> srcNodeId)
      : m_id{id},
        m_spawnTime{spawnTime},
        m_freeTime{0},
        m_trip{itinerary != nullptr ? std::vector<std::shared_ptr<Itinerary>>{itinerary}
                                    : std::vector<std::shared_ptr<Itinerary>>{}},
        m_srcNodeId{srcNodeId},
        m_srcStreetId{std::nullopt},
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.},
        m_bIsIntelligent{false} {}
  Agent::Agent(Id const id,
               std::time_t const& spawnTime,
               std::vector<std::shared_ptr<Itinerary>> const& trip,
               std::optional<Id> srcNodeId)
      : m_id{id},
        m_spawnTime{spawnTime},
        m_freeTime{spawnTime},
        m_trip{trip},
        m_srcNodeId{srcNodeId},
        m_srcStreetId{std::nullopt},
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.},
        m_bIsIntelligent{false} {}

  void Agent::setSrcNodeId(Id srcNodeId) { m_srcNodeId = srcNodeId; }
  void Agent::setSpawnTime(std::time_t const& spawnTime) { m_spawnTime = spawnTime; }
  void Agent::setStreetId(std::optional<Id> streetId) {
    if (!streetId.has_value()) {
      if (!m_nextStreetId.has_value()) {
        throw std::logic_error(std::format(
            "Agent {} has no next street id to set the current street id to", m_id));
      }
      m_streetId = std::exchange(m_nextStreetId, std::nullopt);
      return;
    }
    if (m_nextStreetId.has_value()) {
      throw std::logic_error(std::format(
          "Agent {} has a next street id already set, cannot set street id directly",
          m_id));
    }
    m_streetId = streetId;
    m_nextStreetId = std::nullopt;
  }
  void Agent::setSpeed(double speed) {
    if (speed < 0.) {
      throw std::invalid_argument(
          std::format("Speed ({}) of agent {} must be positive", speed, m_id));
    }
    m_speed = speed;
  }
  void Agent::setFreeTime(std::time_t const& freeTime) { m_freeTime = freeTime; }

  void Agent::incrementDistance(double distance) {
    if (distance < 0) {
      throw std::invalid_argument(std::format(
          "Distance travelled ({}) by agent {} must be positive", distance, m_id));
    }
    m_distance += distance;
  }
  void Agent::setPath(std::vector<Id> path) noexcept {
    m_path = std::move(path);
    m_pathIndex = 0;
  }
  void Agent::clearPath() noexcept {
    m_path.clear();
    m_pathIndex = 0;
  }
  void Agent::advancePath() {
    if (m_pathIndex + 1 >= m_path.size()) {
      throw std::logic_error(
          std::format("Agent {} is at the end of its planned route", m_id));
    }
    ++m_pathIndex;
  }
  std::optional<Id> Agent::nextPlannedStreetId() const noexcept {
    if (m_pathIndex + 1 >= m_path.size()) {
      return std::nullopt;
    }
    return m_path[m_pathIndex + 1];
  }
  bool Agent::isOnPath(Id const streetId) const noexcept {
    return m_pathIndex < m_path.size() && m_path[m_pathIndex] == streetId;
  }

  void Agent::updateItinerary() {
    if (!m_trip.empty() && m_itineraryIdx < m_trip.size() - 1) {
      ++m_itineraryIdx;
      clearPath();
    }
  }
  void Agent::reset(std::time_t const& spawnTime) {
    m_spawnTime = spawnTime;
    m_freeTime = 0;
    m_streetId = std::nullopt;
    m_nextStreetId = m_srcStreetId;
    m_speed = 0.;
    m_distance = 0.;
    m_itineraryIdx = 0;
    clearPath();
  }

  std::shared_ptr<Itinerary> const& Agent::itinerary() const {
    if (isRandom()) {
      throw std::logic_error(
          std::format("Agent {} is a random agent and has no itinerary", m_id));
    }
    return m_trip[m_itineraryIdx];
  }
}  // namespace dsf::mobility