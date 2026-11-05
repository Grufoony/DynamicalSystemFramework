/// @file       /src/dsf/mobility/Router.hpp
/// @brief      Defines the Router class, which plans routes on a road network with A*.

#pragma once

#include "../utility/Typedef.hpp"

#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

#include <tbb/enumerable_thread_specific.h>

namespace dsf::mobility {
  class RoadNetwork;
  class Street;

  /// @brief The street costs a route is planned on
  enum class RouteWeights : std::uint8_t {
    /// @brief The free-flow travel time of each street, i.e. length / maxSpeed
    FREE_FLOW = 0,
    /// @brief The network's weight function (see RoadNetwork::setEdgeWeight), as of the last
    ///   snapshot (see Router::updateCurrentWeights)
    CURRENT = 1,
  };

  /// @brief Plans routes on a road network with the A* algorithm
  /// @details The search runs on the streets (the dual graph), so that forbidden turns and the
  ///   U-turn rule of RoadNetwork::isTurnAllowed are honoured. A route ends on the first
  ///   street whose target is the goal node.
  ///   The router holds a compact copy of the network's topology and of the street costs: it
  ///   must be rebuilt (see build) whenever streets are added, opened or closed, or their turns
  ///   or speeds change. Route queries are const and thread-safe, as each thread searches with
  ///   its own buffers; building and taking snapshots are not.
  ///
  ///   The heuristic is the straight-line distance to the goal node, scaled by the lowest cost
  ///   per unit of distance over all streets, which makes it admissible and consistent for any
  ///   non-negative costs and any coordinate system. Without coordinates on every node touching
  ///   a street the heuristic is zero, i.e. the search is Dijkstra's.
  ///
  ///   If the network's weight threshold t (see RoadNetwork::setEdgeWeight) is positive, every
  ///   street cost is multiplied by a factor in [1, 1 + t), drawn from a hash of the query seed
  ///   and the street id: the cost of the returned route is then at most (1 + t) times the
  ///   optimal one, and different seeds spread over near-optimal routes.
  class Router {
  public:
    /// @brief Build the routing graph of a network
    /// @param network The road network
    /// @details Also takes a snapshot of the current weights (see updateCurrentWeights)
    void build(RoadNetwork const& network);
    /// @brief Take a snapshot of the current street weights, i.e. RoadNetwork::edgeWeight
    /// @param network The road network the router was built from
    /// @details The streets are evaluated in parallel: no street may change meanwhile.
    void updateCurrentWeights(RoadNetwork const& network);

    /// @brief Plan a route from a street to a node
    /// @param fromStreetId The street the route starts with: its own cost is not counted
    /// @param goalNodeId The node to reach
    /// @param weights The street costs to plan on
    /// @param seed The seed of the cost perturbation, used only with a positive threshold
    /// @return std::vector<Id> The ids of the streets to travel, from fromStreetId to the first
    ///   street whose target is goalNodeId, or an empty vector if the node cannot be reached
    /// @throws std::out_of_range if the street or the node is not in the routing graph
    std::vector<Id> route(Id fromStreetId,
                          Id goalNodeId,
                          RouteWeights weights,
                          std::uint64_t seed = 0) const;
    /// @brief Plan a route from a node to a node
    /// @param fromNodeId The node the route starts from: the cost of the first street counts
    /// @param goalNodeId The node to reach
    /// @param weights The street costs to plan on
    /// @param seed The seed of the cost perturbation, used only with a positive threshold
    /// @return std::vector<Id> The ids of the streets to travel, from an open outgoing street of
    ///   fromNodeId to the first street whose target is goalNodeId, or an empty vector if the
    ///   node cannot be reached
    /// @throws std::out_of_range if a node is not in the routing graph
    std::vector<Id> routeFromNode(Id fromNodeId,
                                  Id goalNodeId,
                                  RouteWeights weights,
                                  std::uint64_t seed = 0) const;
    /// @brief Get the cost of a route, without perturbation
    /// @param route The ids of the streets of the route
    /// @param weights The street costs to use
    /// @return double The sum of the costs of the streets of the route, but the first
    /// @throws std::out_of_range if a street is not in the routing graph
    double routeCost(std::span<Id const> route, RouteWeights weights) const;
    /// @brief Check whether the routing graph allows a turn
    /// @param fromStreetId The street the agent is on
    /// @param toStreetId The street the agent would take next
    /// @return true if toStreetId is open and the turn is allowed (see
    ///   RoadNetwork::isTurnAllowed), false otherwise or if a street is unknown
    bool isTurnAllowed(Id fromStreetId, Id toStreetId) const;

    /// @brief Mix a seed with a key, e.g. the id of an agent, into a new seed
    /// @param seed The seed
    /// @param key The key
    /// @return std::uint64_t The mixed seed (SplitMix64)
    static constexpr std::uint64_t mixSeed(std::uint64_t seed,
                                           std::uint64_t key) noexcept {
      auto z{seed + 0x9E3779B97F4A7C15ULL * (key + 1)};
      z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
      z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
      return z ^ (z >> 31);
    }

    /// @brief Check whether the router has been built
    inline bool isBuilt() const noexcept { return !m_turnOffsets.empty(); }
    /// @brief Get the number of streets of the routing graph
    inline std::size_t nEdges() const noexcept { return m_streetIds.size(); }
    /// @brief Check whether the search uses the straight-line distance heuristic
    inline bool hasHeuristic() const noexcept { return m_bHeuristic; }
    /// @brief Get the relative tolerance of the cost perturbation (0 if none)
    inline double threshold() const noexcept { return m_threshold; }

  private:
    struct WeightSet {
      std::vector<double> weights;  // Cost of each street, by index
      double heuristicFactor{0.};   // Lowest cost per unit of straight-line distance
    };
    struct HeapEntry {
      double f;  // Cost so far plus heuristic
      double g;  // Cost so far
      std::uint32_t street;
    };
    struct SearchBuffers {
      std::vector<double> cost;
      std::vector<std::uint32_t> parent;
      // cost and parent are valid only where stamp equals generation, so that a search
      // does not need to reset them
      std::vector<std::uint32_t> stamp;
      std::uint32_t generation{0};
      std::vector<HeapEntry> heap;

      void prepare(std::size_t nStreets);
    };

    // Streets, by index (ascending id)
    std::vector<Id> m_streetIds;
    std::vector<Street const*> m_streets;
    std::vector<std::uint32_t> m_streetSources;  // Node index
    std::vector<std::uint32_t> m_streetTargets;  // Node index
    std::vector<double> m_streetDistances;       // Straight-line, empty without heuristic
    std::unordered_map<Id, std::uint32_t> m_streetIndices;
    // Nodes, by index (ascending id)
    std::unordered_map<Id, std::uint32_t> m_nodeIndices;
    std::vector<double> m_nodeX, m_nodeY;  // Planar coordinates, empty without heuristic
    // Allowed turns from each street, and open outgoing streets of each node (CSR)
    std::vector<std::uint32_t> m_turnOffsets, m_turns;
    std::vector<std::uint32_t> m_nodeOutOffsets, m_nodeOut;
    WeightSet m_freeFlow, m_current;
    double m_threshold{0.};
    bool m_bHeuristic{false};
    mutable tbb::enumerable_thread_specific<SearchBuffers> m_buffers;

    void m_buildHeuristic(RoadNetwork const& network, std::span<Id const> nodeIds);
    double m_heuristicFactor(std::span<double const> weights) const;
    std::uint32_t m_streetIndex(Id streetId) const;
    std::uint32_t m_nodeIndex(Id nodeId) const;
    WeightSet const& m_weightSet(RouteWeights weights) const noexcept;
    /// @brief Run A* from the given streets to the goal node
    /// @param starts The indices of the start streets
    /// @param bCountStartCost Whether the cost of the start streets counts
    std::vector<Id> m_search(std::span<std::uint32_t const> starts,
                             bool bCountStartCost,
                             std::uint32_t goalNode,
                             WeightSet const& weightSet,
                             std::uint64_t seed) const;
  };
}  // namespace dsf::mobility
