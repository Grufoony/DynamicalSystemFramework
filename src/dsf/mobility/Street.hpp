/// @file       /src/dsf/headers/Street.hpp
/// @brief      Defines the Street class.
///
/// @details    This file contains the definition of the Street class.
///             The Street class represents a street in the network. It is templated by the
///             type of the street's id and the type of the street's capacity.
///             The street's id and capacity must be unsigned integral types.

#pragma once

#include "Agent.hpp"
#include "Road.hpp"
#include "Sensors.hpp"
#include "../utility/TypeTraits/is_numeric.hpp"
#include "../utility/Measurement.hpp"
#include "../utility/queue.hpp"
#include "../utility/Typedef.hpp"

#include <optional>
#include <queue>
#include <type_traits>
#include <utility>
#include <stdexcept>
#include <cmath>
#include <numbers>
#include <format>
#include <cassert>
#include <string>
#include <unordered_map>
#include <vector>

#include <tbb/concurrent_unordered_map.h>

namespace dsf::mobility {

  class AgentComparator {
  public:
    template <typename T>
    bool operator()(T const& lhs, T const& rhs) const {
      return lhs->freeTime() > rhs->freeTime();
    }
  };

  class Agent;

  /// @brief The position of the counter on the street
  enum class CounterPosition : uint8_t { ENTRY = 0, MIDDLE = 1, EXIT = 2 };

  /// @brief The Street class represents a street in the network.
  class Street : public Road {
  private:
    std::vector<dsf::queue<std::unique_ptr<Agent>>> m_exitQueues;
    dsf::priority_queue<std::unique_ptr<Agent>,
                        std::vector<std::unique_ptr<Agent>>,
                        AgentComparator>
        m_movingAgents;
    std::unordered_map<Id, std::time_t> m_agentsInsertionTimes;
    std::vector<double> m_avgSpeeds;
    std::vector<Direction> m_laneMapping;
    std::optional<Counter> m_counter;
    CounterPosition m_counterPosition{CounterPosition::EXIT};
    double m_stationaryWeight{1.0};
    static std::optional<tbb::concurrent_unordered_map<
        Id,
        std::vector<std::tuple<Id, std::time_t, std::time_t>>>>
        m_agentData;

    /// @brief Update the street's lane mapping
    /// @param nLanes The street's number of lanes
    void m_updateLaneMapping(int const nLanes);

  public:
    /// @brief Construct a new Street object
    /// @param id The street's id
    /// @param nodePair The street's node pair
    /// @param length The street's length, in meters (default is the mean vehicle length)
    /// @param nLanes The street's number of lanes (default is 1)
    /// @param maxSpeed The street's speed limit, in m/s (default is 50 km/h)
    /// @param name The street's name (default is an empty string)
    /// @param geometry The street's geometry
    /// @param capacity The street's capacity (default is the maximum number of vehicles that can fit in the street)
    /// @param transportCapacity The street's transport capacity (default is 1)
    Street(Id id,
           std::pair<Id, Id> nodePair,
           double length = Road::meanVehicleLength(),
           double maxSpeed = 13.8888888889,
           int nLanes = 1,
           std::string name = std::string(),
           geometry::PolyLine geometry = {},
           std::optional<int> capacity = std::nullopt,
           double transportCapacity = 1.);
    Street(Street&&) = default;
    Street(Street const&) = delete;
    bool operator==(Street const& other) const;

    static inline void acquireAgentData() {
      m_agentData = tbb::concurrent_unordered_map<
          Id,
          std::vector<std::tuple<Id, std::time_t, std::time_t>>>();
    };
    /// @brief Set the street's lane mapping
    /// @param laneMapping The street's lane mapping
    void setLaneMapping(std::vector<Direction> const& laneMapping);
    /// @brief Set the street's queue
    /// @param queue The street's queue
    /// @param index The index of the queue
    void setQueue(dsf::queue<std::unique_ptr<Agent>> queue, size_t index);
    /// @brief Set the mean vehicle length
    /// @param meanVehicleLength The mean vehicle length
    /// @throw std::invalid_argument If the mean vehicle length is negative
    static void setMeanVehicleLength(double meanVehicleLength);
    /// @brief Set the street's stationary weight
    /// @param weight The street's stationary weight
    inline void setStationaryWeight(double const weight) {
      weight > 0. ? m_stationaryWeight = weight
                  : throw std::invalid_argument("Stationary weight must be positive");
    }
    /// @brief Change the number of lanes of the street. Usually if there is a construction site, you may want to
    /// reduce the number of lanes and possibly the max speed.
    /// @param nLanes The new number of lanes
    /// @param speedFactor Optional, The factor to multiply the max speed of the street
    void changeNLanes(int const nLanes,
                      std::optional<double> const speedFactor = std::nullopt);
    /// @brief Enable a coil (dsf::Counter sensor) on the street
    /// @param name The name of the counter (default is "Coil_<street_id>")
    /// @param position The position of the counter on the street (default is EXIT)
    void enableCounter(std::string name = std::string(),
                       CounterPosition position = CounterPosition::EXIT);
    /// @brief Reset the counter of the street
    /// @throw std::runtime_error If the street does not have a coil
    void resetCounter();

    /// @brief Get the street's queue
    /// @return const dsf::queue<std::unique_ptr<Agent>>&, The street's queue
    const dsf::queue<std::unique_ptr<Agent>>& queue(size_t const& index) const {
      return m_exitQueues[index];
    }
    /// @brief Get the street's queues
    /// @return std::vector<dsf::queue<std::unique_ptr<Agent>>> The street's queues
    std::vector<dsf::queue<std::unique_ptr<Agent>>> const& exitQueues() const {
      return m_exitQueues;
    }
    /// @brief  Get the number of agents on the street
    /// @return std::size_t, The number of agents on the street
    std::size_t nAgents() const final;
    /// @brief Get the street's stationary weight
    /// @return double The street's stationary weight
    inline auto stationaryWeight() const noexcept { return m_stationaryWeight; }
    /// @brief Get the name of the counter
    /// @return std::string The name of the counter
    inline auto counterName() const {
      return hasCoil() ? m_counter->name() : std::string("N/A");
    }
    /// @brief Get the counts of the counter
    /// @return std::size_t The counts of the counter
    inline auto counts() const {
      return hasCoil() ? m_counter->value() : static_cast<std::size_t>(0);
    }
    /// @brief Get the street's moving agents priority queue
    /// @return dsf::priority_queue<std::unique_ptr<Agent>, std::vector<std::unique_ptr<Agent>>,
    /// AgentComparator>& The street's moving agents priority queue
    inline dsf::priority_queue<std::unique_ptr<Agent>,
                               std::vector<std::unique_ptr<Agent>>,
                               AgentComparator>&
    movingAgents() {
      return m_movingAgents;
    }
    inline const auto& movingAgents() const { return m_movingAgents; }
    static inline auto agentData() {
      if (!m_agentData.has_value()) {
        throw std::runtime_error(
            "agentData not initialized. Please call acquireAgentData() first.");
      }
      return std::move(*m_agentData);
    }
    /// @brief Get the number of of moving agents, i.e. agents not yet enqueued
    /// @return std::size_t The number of moving agents
    std::size_t nMovingAgents() const;
    /// @brief Get the number of agents on all queues for a given direction
    /// @param direction The direction of the agents (default is ANY)
    /// @param normalizeOnNLanes If true, the number of agents is normalized by the number of lanes
    /// @return double The number of agents on all queues for a given direction
    double nExitingAgents(Direction direction = Direction::ANY,
                          bool normalizeOnNLanes = false) const;
    /// @brief Get the mean speed of the agents that have passed through the street
    /// @param bReset If true, the average speed data is reset after the computation
    /// @return Measurement<double> The (mean, std) speed of the agents that have passed through the street
    Measurement<double> meanSpeed(bool const bReset = true);

    /// @brief Get the street's lane mapping
    /// @return std::vector<Direction> The street's lane mapping
    inline auto const& laneMapping() const { return m_laneMapping; }
    /// @brief Add an agent to the street
    /// @param pAgent The agent to add to the street
    /// @param currentTime The current simulation time
    void addAgent(std::unique_ptr<Agent> pAgent, std::time_t const currentTime);
    /// @brief Remove the top agent from the street's moving agents priority queue
    /// @return std::unique_ptr<Agent> The agent removed from the street's moving agents priority queue
    std::unique_ptr<Agent> dequeueMovingAgent();
    /// @brief Add an agent to the street's queue
    /// @param queueId The id of the queue
    /// @throw std::runtime_error If the street's queue is full
    void enqueue(std::size_t const& queueId);
    /// @brief Remove an agent from the street's queue
    /// @param index The index of the queue
    /// @param currentTime The current simulation time
    /// @return Id The id of the agent removed from the street's queue
    std::unique_ptr<Agent> dequeue(std::size_t const& index,
                                   std::time_t const currentTime);
    /// @brief Check if the street has a coil (dsf::Counter sensor) on it
    /// @return bool True if the street has a coil, false otherwise
    constexpr bool hasCoil() const { return m_counter.has_value(); };
  };
}  // namespace dsf::mobility

// Specialization of std::formatter for dsf::Street
template <>
struct std::formatter<dsf::mobility::Street> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::mobility::Street& street, FormatContext&& ctx) const {
    auto const& name =
        street.name().empty() ? std::string() : std::format(" \"{}\"", street.name());
    return std::format_to(ctx.out(),
                          "Street(id: {}{}, from {} to {}, length: {} m, max speed: "
                          "{:.2f} m/s, lanes: {}, agents: {}, n enqueued: {})",
                          street.id(),
                          name,
                          street.nodePair().first,
                          street.nodePair().second,
                          street.length(),
                          street.maxSpeed(),
                          street.nLanes(),
                          street.nAgents(),
                          street.nExitingAgents());
  }
};