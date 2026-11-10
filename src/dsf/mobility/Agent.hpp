/// @file       /src/dsf/headers/Agent.hpp
/// @brief      Defines the Agent class.
///
/// @details    This file contains the definition of the Agent class.
///             The Agent class represents an agent in the network. It is templated by the type
///             of the agent's id and the size of agents, which must both be unsigned integrals.
///				      It is also templated by the delay_t type, which must be a numeric (see utility/TypeTraits/is_numeric.hpp)
///				      and represents the spatial or temporal (depending on the type of the template) distance
///				      between the agent and the one in front of it.

#pragma once

#include "../utility/Typedef.hpp"

#include <cassert>
#include <concepts>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace dsf::mobility {
  /// @brief The Agent class represents an agent in the network.
  class Agent {
  private:
    Id m_id;
    std::time_t m_spawnTime, m_freeTime;
    std::optional<Id> m_streetId;
    std::optional<Id> m_srcNodeId;
    std::optional<Id> m_srcStreetId;  // The first street of the agent's trip
    std::optional<Id> m_nextStreetId;
    double m_speed;
    double m_distance;                     // Travelled distance
    std::optional<double> m_maxDistance;   // Maximum distance for stochastic agents
    std::optional<std::time_t> m_maxTime;  // Maximum time for stochastic agents
    // The route, reversed: the destination street first, then the planned streets still to
    // take after the current one, the next one last, so that advancing is a pop_back.
    std::vector<Id> m_route;
    bool m_bIsIntelligent;  // If true, the agent plans its routes on the current traffic,
                            // otherwise on free-flow travel times (default is false)

  public:
    /// @brief Construct a new Agent object
    /// @param id The agent's id
    /// @param spawnTime The agent's spawn time
    /// @param destination Optional, the id of the agent's destination street: the agent
    ///   travels it, and arrives at its end. If not provided, the agent is a random agent
    /// @param srcNodeId Optional, The id of the source node of the agent
    Agent(Id const id,
          std::time_t const& spawnTime,
          std::optional<Id> destination = std::nullopt,
          std::optional<Id> srcNodeId = std::nullopt);

    /// @brief Set the id of the source node of the agent
    /// @param srcNodeId The id of the source node
    void setSrcNodeId(Id srcNodeId);
    /// @brief Set the agent's spawn time
    /// @param spawnTime The time-step at which the agent enters the simulation
    /// @details The spawn time is stamped when the agent is created, but an agent may
    /// sit in the dynamics' pending list for an arbitrary number of time-steps before a
    /// free slot on its source node/street becomes available. Re-stamping it on actual
    /// insertion keeps the measured travel time the time spent *in* the network.
    void setSpawnTime(std::time_t const& spawnTime);
    /// @brief Set the street occupied by the agent
    /// @param streetId The id of the street currently occupied by the agent
    void setStreetId(std::optional<Id> streetId = std::nullopt);
    /// @brief Set the id of the first street of the agent's trip
    /// @param srcStreetId The id of the first street, which must be an outgoing street of
    /// the agent's source node
    inline auto setSrcStreetId(Id srcStreetId) { m_srcStreetId = srcStreetId; }
    /// @brief Set the id of the next street
    /// @param nextStreetId The id of the next street
    inline auto setNextStreetId(Id nextStreetId) { m_nextStreetId = nextStreetId; }
    /// @brief Set the agent's speed
    /// @param speed, The agent's speed
    /// @throw std::invalid_argument, if speed is negative
    void setSpeed(double speed);
    /// @brief Set the agent's free time
    /// @param freeTime The agent's free time
    void setFreeTime(std::time_t const& freeTime);
    /// @brief Increment the agent's distance by a given value
    /// @param distance The value to increment the agent's distance byù
    /// @throw std::invalid_argument, if distance is negative
    void incrementDistance(double distance);
    /// @brief Set the agent's maximum distance
    /// @param maxDistance The agent's maximum distance
    /// @throw std::invalid_argument, if maxDistance is not positive
    inline void setMaxDistance(double const maxDistance) {
      maxDistance > 0. ? m_maxDistance = maxDistance
                       : throw std::invalid_argument(
                             "Agent::setMaxDistance: maxDistance must be positive");
    };
    /// @brief Set the agent's maximum time
    /// @param maxTime The agent's maximum time
    inline void setMaxTime(std::time_t const maxTime) { m_maxTime = maxTime; }
    /// @brief Set whether the agent is intelligent
    /// @param isIntelligent If true, the agent plans its routes on the current traffic,
    ///   otherwise on free-flow travel times
    inline void setIntelligent(bool const isIntelligent) noexcept {
      m_bIsIntelligent = isIntelligent;
    }
    /// @brief Set the streets the agent plans to take after the one it is on (or, before its
    ///   departure, after its first street)
    /// @param streets The ids of the streets, in travel order
    /// @throw std::logic_error if the agent is a random agent
    void setPath(std::span<Id const> streets);
    /// @brief Drop the agent's planned streets, so that its route is planned again when
    ///   needed; the destination is kept
    void dropPath() noexcept;
    /// @brief Remove the next planned street, once the agent takes it
    /// @throw std::logic_error if there is no planned street
    void advancePath();
    /// @brief Reset the agent
    /// @details Reset the following values:
    /// - street id = std::nullopt
    /// - delay = 0
    /// - speed = 0
    /// - distance = 0
    /// - time = 0
    /// - planned streets = none (the destination is kept)
    void reset(std::time_t const& spawnTime);

    /// @brief Get the agent's spawn time
    /// @return The agent's spawn time
    inline std::time_t const& spawnTime() const noexcept { return m_spawnTime; };
    /// @brief Get the agent's free time
    /// @return The agent's free time
    inline std::time_t const& freeTime() const noexcept { return m_freeTime; };
    /// @brief Get the agent's id
    /// @return The agent's id
    inline Id id() const noexcept { return m_id; };
    /// @brief Get the agent's destination street
    /// @return std::optional<Id> The id of the destination street, std::nullopt for a random
    ///   agent
    inline std::optional<Id> destination() const noexcept {
      return m_route.empty() ? std::nullopt : std::make_optional(m_route.front());
    }
    /// @brief Get the agent's maximum distance
    /// @return The agent's maximum distance, or throw std::logic_error if not set
    inline auto maxDistance() const {
      return m_maxDistance.has_value()
                 ? m_maxDistance.value()
                 : throw std::logic_error("Agent::maxDistance: maxDistance is not set");
    };
    /// @brief Get the agent's maximum time
    /// @return The agent's maximum time, or throw std::logic_error if not set
    inline auto maxTime() const {
      return m_maxTime.has_value()
                 ? m_maxTime.value()
                 : throw std::logic_error("Agent::maxTime: maxTime is not set");
    };
    /// @brief Get the id of the street currently occupied by the agent
    /// @return The id of the street currently occupied by the agent
    inline std::optional<Id> streetId() const noexcept { return m_streetId; };
    /// @brief Get the id of the source node of the agent
    /// @return The id of the source node of the agent
    inline std::optional<Id> srcNodeId() const noexcept { return m_srcNodeId; };
    /// @brief Get the id of the first street of the agent's trip
    /// @return The id of the first street of the agent's trip
    inline std::optional<Id> srcStreetId() const noexcept { return m_srcStreetId; };
    /// @brief Get the id of the next street
    /// @return The id of the next street
    inline std::optional<Id> nextStreetId() const noexcept { return m_nextStreetId; };
    /// @brief Get the agent's speed
    /// @return The agent's speed
    inline double speed() const noexcept { return m_speed; };
    /// @brief Get the agent's travelled distance
    /// @return The agent's travelled distance
    inline double distance() const noexcept { return m_distance; };
    /// @brief Return true if the agent is a random agent
    /// @return True if the agent is a random agent, false otherwise
    inline bool isRandom() const noexcept { return m_route.empty(); };
    /// @brief Return true if the agent is intelligent
    /// @return True if the agent plans its routes on the current traffic, false if it plans
    ///   them on free-flow travel times
    inline bool isIntelligent() const noexcept { return m_bIsIntelligent; };
    /// @brief Get the next planned street
    /// @return std::optional<Id> The street id, or std::nullopt if no street is planned
    inline std::optional<Id> nextPlannedStreetId() const noexcept {
      return m_route.size() > 1 ? std::make_optional(m_route.back()) : std::nullopt;
    }
    /// @brief Get the number of planned streets
    /// @return std::size_t The number of streets the agent still plans to take
    inline std::size_t nPlannedStreets() const noexcept {
      return m_route.empty() ? 0 : m_route.size() - 1;
    }
    /// @brief Check if a random agent has arrived at its destination
    /// @param currentTime The current simulation time
    /// @return True if the agent has arrived (exceeded max distance or time), false otherwise
    inline bool hasArrived(std::optional<std::time_t> const& currentTime) const noexcept {
      if (!isRandom()) {
        return false;
      }
      if (currentTime.has_value() && m_maxTime.has_value()) {
        return (currentTime.value() - m_spawnTime) >= m_maxTime.value();
      }
      if (m_maxDistance.has_value()) {
        return m_distance >= m_maxDistance.value();
      }
      return false;
    };
  };
}  // namespace dsf::mobility

// Specialization of std::formatter for dsf::mobility::Agent
template <>
struct std::formatter<dsf::mobility::Agent> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::mobility::Agent& agent, FormatContext&& ctx) const {
    return std::format_to(
        ctx.out(),
        "Agent(id: {}, streetId: {}, srcNodeId: {}, nextStreetId: {}, "
        "destination: {}, speed: {:.2f} m/s, distance: {:.2f} m, "
        "spawnTime: {}, freeTime: {})",
        agent.id(),
        agent.streetId().has_value() ? std::to_string(agent.streetId().value()) : "N/A",
        agent.srcNodeId().has_value() ? std::to_string(agent.srcNodeId().value()) : "N/A",
        agent.nextStreetId().has_value() ? std::to_string(agent.nextStreetId().value())
                                         : "N/A",
        agent.isRandom() ? std::string("RANDOM") : std::to_string(*agent.destination()),
        agent.speed(),
        agent.distance(),
        agent.spawnTime(),
        agent.freeTime());
  }
};
