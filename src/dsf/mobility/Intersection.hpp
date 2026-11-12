/// @file src/dsf/headers/Intersection.hpp
/// @brief Header file for the Intersection class

/// @details This file contains the definition of the Intersection class. The Intersection class
///          represents a node in the road network. It is derived from the Node class and has a
///          multimap of agents waiting to pass through the intersection. Agents are ordered by
///          their angle difference, emulating real-world precedence. Each lane of the ingoing
///          streets can have at most one agent in the intersection.

#pragma once

#include "Agent.hpp"
#include "RoadJunction.hpp"

#include <algorithm>
#include <format>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <utility>

namespace dsf::mobility {
  /// @brief The Intersection class represents a node in the network.
  /// @tparam Id The type of the node's id. It must be an unsigned integral type.
  class Intersection : public RoadJunction {
  public:
    using AgentMap = std::multimap<int16_t, std::unique_ptr<Agent>>;

  protected:
    AgentMap m_agents;
    std::set<Id>
        m_streetPriorities;  // A set containing the street ids that have priority - like main roads
    std::map<std::pair<Id, std::size_t>, Id>
        m_laneOccupants;  // (street id, lane) -> id of the agent coming from that lane
    bool m_priorityFromStreets{
        false};  // Whether flagged ingoing streets have right of way

  public:
    /// @brief Construct a new Intersection object
    /// @param id The node's id
    explicit Intersection(Id id) : RoadJunction{id} {};
    /// @brief Construct a new Intersection object
    /// @param id The node's id
    /// @param coords A dsf::geometry::Point containing the node's coordinates
    Intersection(Id id, geometry::Point coords) : RoadJunction{id, coords} {};

    Intersection(RoadJunction const& node) : RoadJunction{node} {};

    Intersection(Intersection const&) = delete;

    virtual ~Intersection() = default;

    /// @brief Set the node's capacity
    /// @param capacity The node's capacity
    /// @throws std::runtime_error if the capacity is smaller than the current queue size
    void setCapacity(std::size_t const capacity) override;

    /// @brief Put an agent in the node
    /// @param angle The agent's angle difference
    /// @param pAgent The agent
    /// @param lane The lane of the agent's current street the agent comes from, if any
    /// @details The agent's angle difference is used to order the agents in the node.
    ///          The agent with the smallest angle difference is the first one to be
    ///          removed from the node. If the lane is given, it stays occupied until the
    ///          agent is removed.
    /// @throws std::runtime_error if the node is full or the lane is already occupied
    /// @throws std::invalid_argument if the lane is given but the agent is on no street
    void addAgent(double angle,
                  std::unique_ptr<Agent> pAgent,
                  std::optional<std::size_t> lane = std::nullopt);
    /// @brief Put an agent in the node
    /// @param agentId The agent's id
    /// @details The agent's angle difference is used to order the agents in the node.
    ///          The agent with the smallest angle difference is the first one to be
    ///          removed from the node.
    /// @throws std::runtime_error if the node is full
    void addAgent(std::unique_ptr<Agent> pAgent);
    /// @brief Remove an agent from the node, freeing the lane it came from
    /// @param it An iterator to the agent in agents()
    /// @return std::unique_ptr<Agent> The removed agent
    std::unique_ptr<Agent> removeAgent(AgentMap::const_iterator it);
    /// @brief Check whether a lane of an ingoing street has an agent in the node
    /// @param streetId The street's id
    /// @param lane The lane's index
    /// @return bool True if an agent coming from that lane is in the node
    inline bool isLaneOccupied(Id streetId, std::size_t lane) const {
      return m_laneOccupants.contains({streetId, lane});
    }
    /// @brief Set the node streets with priority
    /// @param streetPriorities A std::set containing the node's street priorities
    void setStreetPriorities(std::set<Id> streetPriorities) {
      m_streetPriorities = std::move(streetPriorities);
    }
    /// @brief Add a street to the node street priorities
    /// @param streetId The street's id
    void addStreetPriority(Id streetId) {
      if (std::ranges::find(m_ingoingEdges, streetId, &Edge::id) ==
          m_ingoingEdges.cend()) {
        throw std::invalid_argument(std::format(
            "Street with id {} is not ingoing edge of intersection with id {}.",
            streetId,
            m_id));
      }
      m_streetPriorities.emplace(streetId);
    }
    /// @brief Returns the number of agents currently in the node
    /// @return std::size_t The number of agents currently in the node
    inline auto nAgents() const noexcept { return m_agents.size(); }
    /// @brief Returns the node's density
    /// @return double The node's density
    inline double density() const override {
      return static_cast<double>(this->nAgents()) / this->capacity();
    }
    /// @brief Returns true if the node is full
    /// @return bool True if the node is full
    inline bool isFull() const override { return this->nAgents() >= this->capacity(); }

    /// @brief Get the node's street priorities
    /// @details This function returns a std::set containing the node's street priorities.
    ///        If a street has priority, it means that the agents that are on that street
    ///        have priority over the agents that are on the other streets.
    /// @return std::set<Id> A std::set containing the node's street priorities
    virtual const std::set<Id>& streetPriorities() const { return m_streetPriorities; };
    /// @brief Set whether the ingoing streets flagged as priority (see Street::hasPriority)
    ///   have right of way at this node, when it has no street priorities of its own
    /// @param bPriorityFromStreets True to give way to the flagged streets
    void setPriorityFromStreets(bool const bPriorityFromStreets) noexcept {
      m_priorityFromStreets = bPriorityFromStreets;
    }
    /// @brief Whether the ingoing streets flagged as priority have right of way at this node
    bool priorityFromStreets() const noexcept { return m_priorityFromStreets; }
    /// @brief Whether an ingoing street has right of way at this node
    /// @param street An ingoing street of the node
    /// @return True if the street is in the node's street priorities or, when the node has
    ///         none, if it is flagged as priority (e.g. from the "priority" column) and the
    ///         node takes its priorities from the streets (see setPriorityFromStreets)
    bool hasRightOfWay(Street const& street) const {
      if (!m_streetPriorities.empty()) {
        return m_streetPriorities.contains(street.id());
      }
      return m_priorityFromStreets && street.hasPriority();
    }
    /// @brief Whether any ingoing street has right of way at this node
    bool hasPriorityStreets() const {
      return std::ranges::any_of(ingoingEdges(), [this](Street const* pStreet) {
        return hasRightOfWay(*pStreet);
      });
    }
    /// @brief Get the node's agents
    /// @return AgentMap const& The node's agents, in the order they leave the node
    AgentMap const& agents() const noexcept { return m_agents; };

    constexpr bool isIntersection() const noexcept final { return true; }
  };
}  // namespace dsf::mobility

template <>
struct std::formatter<dsf::mobility::Intersection> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(dsf::mobility::Intersection const& intersection,
              FormatContext&& ctx) const {
    std::string strIngoingEdges, strOutgoingEdges;
    for (auto const* pStreet : intersection.ingoingEdges()) {
      strIngoingEdges += std::format("{} ", pStreet->id());
    }
    for (auto const* pStreet : intersection.outgoingEdges()) {
      strOutgoingEdges += std::format("{} ", pStreet->id());
    }
    auto out = std::format_to(
        ctx.out(),
        "Intersection(id: {}, name: {}, capacity: {}, transportCapacity: {}, "
        "nAgents: {}, ingoing edges: {},  outgoing edges: {} coords: ",
        intersection.id(),
        intersection.name(),
        intersection.capacity(),
        intersection.transportCapacity(),
        intersection.nAgents(),
        strIngoingEdges,
        strOutgoingEdges);
    if (intersection.geometry().has_value()) {
      return std::format_to(out, "{})", *intersection.geometry());
    } else {
      return std::format_to(out, "N/A)");
    }
  }
};