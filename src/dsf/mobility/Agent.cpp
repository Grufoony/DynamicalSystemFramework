#include "Agent.hpp"

#include <utility>

namespace dsf::mobility {
  Agent::Agent(Id const id,
               std::time_t const& spawnTime,
               std::optional<Id> destination,
               std::optional<Id> srcNodeId)
      : m_id{id},
        m_spawnTime{spawnTime},
        m_freeTime{0},
        m_srcNodeId{srcNodeId},
        m_srcStreetId{std::nullopt},
        m_nextStreetId{std::nullopt},
        m_speed{0.},
        m_distance{0.},
        m_bIsIntelligent{false} {
    if (destination.has_value()) {
      m_route.push_back(*destination);
    }
  }

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
  void Agent::setPath(std::span<Id const> streets) {
    if (isRandom()) {
      throw std::logic_error(
          std::format("Agent {} is a random agent and cannot plan a route", m_id));
    }
    dropPath();
    m_route.insert(m_route.begin(), streets.begin(), streets.end());
  }
  void Agent::dropPath() noexcept {
    if (m_route.size() > 1) {
      m_route.erase(m_route.begin(), m_route.end() - 1);
    }
  }
  void Agent::advancePath() {
    if (m_route.size() < 2) {
      throw std::logic_error(std::format("Agent {} has no planned street to take", m_id));
    }
    m_route.pop_front();
  }

  void Agent::reset(std::time_t const& spawnTime) {
    m_spawnTime = spawnTime;
    m_freeTime = 0;
    m_streetId = std::nullopt;
    m_nextStreetId = m_srcStreetId;
    m_speed = 0.;
    m_distance = 0.;
    dropPath();
  }

}  // namespace dsf::mobility