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
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.} {}
  Agent::Agent(Id const id,
               std::time_t const& spawnTime,
               std::vector<std::shared_ptr<Itinerary>> const& trip,
               std::optional<Id> srcNodeId)
      : m_id{id},
        m_spawnTime{spawnTime},
        m_freeTime{spawnTime},
        m_trip{trip},
        m_srcNodeId{srcNodeId},
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.} {}

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
  void Agent::updateItinerary() {
    if (m_itineraryIdx < m_trip.size() - 1) {
      ++m_itineraryIdx;
    }
  }
  void Agent::reset(std::time_t const& spawnTime) {
    m_spawnTime = spawnTime;
    m_freeTime = 0;
    m_streetId = std::nullopt;
    m_speed = 0.;
    m_distance = 0.;
    m_itineraryIdx = 0;
  }

  std::shared_ptr<Itinerary> const& Agent::itinerary() const {
    if (isRandom()) {
      throw std::logic_error(
          std::format("Agent {} is a random agent and has no itinerary", m_id));
    }
    return m_trip[m_itineraryIdx];
  }
}  // namespace dsf::mobility