#pragma once

#include "../base/Edge.hpp"

#include <format>
#include <memory>
#include <optional>
#include <set>
#include <string>

namespace dsf::mobility {
  enum class RoadType : std::uint8_t {
    HIGHWAY = 0,
    PRIMARY = 1,
    SECONDARY = 2,
    TERTIARY = 3,
    RESIDENTIAL = 4,
    UNKNOWN = 255,
  };
  enum class RoadStatus : std::uint8_t {
    OPEN = 0,
    CLOSED = 1,
  };

  /// @brief The Road class represents a road in the network.
  class Road : public Edge {
  protected:
    static double m_meanVehicleLength;
    double m_length;
    double m_maxSpeed;
    int m_nLanes;
    std::size_t m_capacity;
    double m_transportCapacity;
    std::string m_name;
    bool m_hasPriority = false;
    std::set<Id> m_forbiddenTurns;  // Stores the forbidden turns (road ids)
    RoadType m_roadType = RoadType::UNKNOWN;
    RoadStatus m_roadStatus = RoadStatus::OPEN;

  public:
    /// @brief Construct a new Road object
    /// @param id The road's id
    /// @param nodePair The road's node pair
    /// @param length The road's length, in meters (default is the mean vehicle length)
    /// @param nLanes The road's number of lanes (default is 1)
    /// @param maxSpeed The road's speed limit, in m/s (default is 50 km/h)
    /// @param name The road's name (default is an empty string)
    /// @param geometry The road's geometry (default is empty)
    /// @param capacity The road's capacity (default is the maximum number of vehicles that can fit in the road)
    /// @param transportCapacity The road's transport capacity (default is 1)
    Road(Id id,
         std::pair<Id, Id> nodePair,
         double length = m_meanVehicleLength,
         double maxSpeed = 13.8888888889,
         int nLanes = 1,
         std::string name = std::string(),
         geometry::PolyLine geometry = {},
         std::optional<int> capacity = std::nullopt,
         double transportCapacity = 1.);
    /// @brief Set the mean vehicle length, in meters (default is 5)
    /// @param meanVehicleLength The mean vehicle length
    /// @throws std::invalid_argument If the mean vehicle length is less or equal to 0
    static void setMeanVehicleLength(double meanVehicleLength);
    /// @brief Get the mean vehicle length
    /// @return double The mean vehicle length
    static double meanVehicleLength();

    /// @brief Set the maximum speed, in meters per second (default is 50 km/h)
    /// @param speed The maximum speed
    /// @throws std::invalid_argument If the speed is less or equal to 0
    void setMaxSpeed(double speed);
    /// @brief Set the capacity, in number of agents
    /// @param capacity The capacity
    /// @throws std::invalid_argument If the capacity is less or equal to 0
    void setCapacity(int capacity);
    /// @brief Set the transport capacity, in number of agents
    /// @param transportCapacity The transport capacity
    /// @throws std::invalid_argument If the transport capacity is less or equal to 0
    void setTransportCapacity(double transportCapacity);
    /// @brief Set the road's priority to true
    inline void setPriority() { m_hasPriority = true; }
    /// @brief Add a road id to the forbidden turns
    /// @param roadId The road id to add
    void addForbiddenTurn(Id roadId);
    /// @brief Replace the road's forbidden turns with the given set
    /// @param forbiddenTurns The set of forbidden turns
    void setForbiddenTurns(std::set<Id> const& forbiddenTurns);
    /// @brief Set the road type
    /// @param roadType The road type
    inline void setRoadType(RoadType const roadType) { m_roadType = roadType; }
    /// @brief Set the road status
    /// @param status The road status
    inline void setStatus(RoadStatus const status) { m_roadStatus = status; }

    /// @brief Get the length, in meters
    /// @return double The length, in meters
    inline auto length() const noexcept { return m_length; }
    /// @brief Get the maximum speed, in meters per second
    /// @return double The maximum speed, in meters per second
    inline auto maxSpeed() const noexcept { return m_maxSpeed; }
    /// @brief Get the number of lanes
    /// @return int The number of lanes
    inline auto nLanes() const noexcept { return m_nLanes; }
    /// @brief Get the road's capacity, in number of agents
    /// @return int The road's capacity, in number of agents
    inline auto capacity() const noexcept { return m_capacity; }
    /// @brief Get the road's density in \f$m^{-1}\f$ or in \f$a.u.\f$, if normalized
    /// @param normalized If true, the road's density is normalized by the road's capacity
    /// @return double, The road's density
    double density(bool normalized = false) const noexcept;
    /// @brief Check if the road is full
    /// @return bool, True if the road is full, false otherwise
    inline bool isFull() const final { return this->nAgents() == this->capacity(); }
    /// @brief Get the road's transport capacity, in number of agents
    /// @return double The road's transport capacity, in number of agents
    inline auto transportCapacity() const noexcept { return m_transportCapacity; }
    /// @brief Get the name
    /// @return std::string The name
    inline auto const& name() const noexcept { return m_name; }
    /// @brief Get the priority
    /// @return bool Whether the road has priority
    inline auto hasPriority() const noexcept { return m_hasPriority; }
    /// @brief Get the road's forbidden turns
    /// @return std::set<Id> The road's forbidden turns
    /// @details The forbidden turns are the road ids that are not allowed to be used by the agents
    ///          when they are on the road.
    inline auto const& forbiddenTurns() const noexcept { return m_forbiddenTurns; }
    /// @brief Get the road type
    /// @return RoadType The road type
    inline auto roadType() const noexcept { return m_roadType; }
    /// @brief Get the string representation of the road type
    /// @return std::string The string representation of the road type
    constexpr std::string strRoadType() const {
      switch (m_roadType) {
        case RoadType::HIGHWAY:
          return "highway";
        case RoadType::PRIMARY:
          return "primary";
        case RoadType::SECONDARY:
          return "secondary";
        case RoadType::TERTIARY:
          return "tertiary";
        case RoadType::RESIDENTIAL:
          return "residential";
        default:
          return "unknown";
      }
    };
    /// @brief Get the road status
    /// @return RoadStatus The road status
    inline auto roadStatus() const noexcept { return m_roadStatus; }
    /// @brief Get the road's turn direction given the previous road angle
    /// @param previousStreetAngle The angle of the previous road
    /// @return Direction The turn direction
    /// @details The turn direction is the direction that the agent must take when it is on the road.
    ///          The possible values are:
    ///          - UTURN (abs of delta is greater than pi)
    ///          - STRAIGHT (abs of delta is less than pi /8)
    ///          - RIGHT (delta is negative and not covered by the above conditions)
    ///          - LEFT (delta is positive and not covered by the above conditions)
    Direction turnDirection(double const& previousStreetAngle) const;

    virtual std::size_t nAgents() const = 0;
  };
}  // namespace dsf::mobility

template <>
struct std::formatter<dsf::mobility::RoadStatus> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(dsf::mobility::RoadStatus const& status, FormatContext&& ctx) const {
    std::string_view name;
    switch (status) {
      case dsf::mobility::RoadStatus::OPEN:
        name = "OPEN";
        break;
      case dsf::mobility::RoadStatus::CLOSED:
        name = "CLOSED";
        break;
      default:
        name = "UNKNOWN";
        break;
    }
    return std::format_to(ctx.out(), "{}", name);
  }
};