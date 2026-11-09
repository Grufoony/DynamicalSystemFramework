/// @file       /src/dsf/mobility/Router.hpp
/// @brief      Defines the Router class, which plans routes on a road network with A*.

#pragma once

#include "../utility/Typedef.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string_view>
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

  /// @brief The search algorithm of the routes
  enum class RouteAlgorithm : std::uint8_t {
    /// @brief A*, with the straight-line distance heuristic when coordinates allow it
    ASTAR = 0,
    /// @brief Dijkstra's algorithm
    DIJKSTRA = 1,
  };

  /// @brief Plans routes on a road network with the A* algorithm
  /// @details The search runs on the streets (the dual graph), so that forbidden turns and the
  ///   U-turn rule of RoadNetwork::isTurnAllowed are honoured. A route ends with the goal
  ///   street, entered by an allowed turn: it reaches the end of it.
  ///   The router holds a compact copy of the network's topology and of the street costs: it
  ///   must be rebuilt (see build) whenever streets are added, opened or closed, or their turns
  ///   or speeds change. Route queries are const and thread-safe, as each thread searches with
  ///   its own buffers; building and taking snapshots are not.
  ///
  ///   The heuristic is the straight-line distance to the goal street's target node, scaled by
  ///   the lowest cost per unit of distance over all streets, which makes it admissible and consistent for any
  ///   non-negative costs and any coordinate system. Without coordinates on every node touching
  ///   a street the heuristic is zero, i.e. the search is Dijkstra's.
  ///
  ///   Routes may add a per-agent distance cost, distanceFactor · length, to the street costs
  ///   (see setEdgeWeight with "generalized" and distanceFactor), as in MATSim's generalized
  ///   cost: agents trading travel time for distance differently spread over more routes.
  class Router {
  public:
    /// @brief A street cost function
    using EdgeWeight = std::function<double(Street const&)>;

    /// @brief Get a street cost function by name
    /// @param name One of:
    /// - "traveltime": the street's estimated travel time plus the time needed to clear its
    ///   exit queues, i.e. estimatedTravelTime() + nExitingAgents(ANY, true) /
    ///   min(1, transportCapacity())
    /// - "length": the street's length
    /// - "uniform": 1 for every street
    /// - "generalized": "traveltime" plus distanceCost · length
    /// - any other value is used as the name of a numeric custom attribute of the streets
    /// @param distanceCost The cost of a unit of length of "generalized", in time per unit of
    ///   length (s/m): 0.1 makes a metre cost as much as driving it at 10 m/s
    /// @return EdgeWeight The cost function
    static EdgeWeight edgeWeightFunction(std::string_view name,
                                         double distanceCost = 0.1);
    /// @brief Set the street costs of the routes
    /// @param name The name of the cost function (see edgeWeightFunction), "traveltime" by
    ///   default. It sets the current weights; "generalized" also adds a distance cost to
    ///   every route, free-flow ones included (see distanceFactor)
    /// @param distanceCost The mean cost of a unit of length of "generalized", in s/m
    /// @param sigma Optional, the spread of the per-agent factor of the distance cost of
    ///   "generalized" (see distanceFactor). Without it, every agent has the same cost
    /// @details The current weights change at the next snapshot (see updateCurrentWeights)
    /// @throws std::invalid_argument if distanceCost or sigma is negative
    void setEdgeWeight(std::string_view name,
                       double distanceCost = 0.1,
                       std::optional<double> sigma = std::nullopt);
    /// @brief Get the mean distance cost of the routes, in s/m (0 unless "generalized")
    inline double distanceCost() const noexcept { return m_distanceCost; }
    /// @brief Get the spread of the per-agent factor of the distance cost, if any
    inline std::optional<double> distanceSigma() const noexcept {
      return m_distanceSigma;
    }
    /// @brief Draw the distance cost of an agent
    /// @param seed A seed identifying the agent
    /// @param distanceCost The mean distance cost
    /// @param sigma Optional, the spread
    /// @return double distanceCost · X, where X is drawn from a log-normal distribution of
    ///   mean 1 and shape sigma, i.e. exp(sigma · Z - sigma² / 2) with Z standard normal;
    ///   distanceCost itself without sigma, or if sigma is 0
    static double distanceFactor(std::uint64_t seed,
                                 double distanceCost,
                                 std::optional<double> sigma);
    /// @brief Set the search algorithm
    /// @param algorithm The algorithm (A* by default)
    inline void setAlgorithm(RouteAlgorithm const algorithm) noexcept {
      m_algorithm = algorithm;
    }
    /// @brief Get the search algorithm
    inline auto algorithm() const noexcept { return m_algorithm; }

    /// @brief Build the routing graph of a network
    /// @param network The road network
    /// @details Also takes a snapshot of the current weights (see updateCurrentWeights)
    void build(RoadNetwork const& network);
    /// @brief Take a snapshot of the current street weights (see setEdgeWeight)
    /// @details The streets are evaluated in parallel: no street may change meanwhile.
    void updateCurrentWeights();

    /// @brief Plan a route from a street to the end of a street
    /// @param fromStreetId The street the route starts with: its own cost is not counted
    /// @param goalStreetId The street to reach and travel: its cost is counted
    /// @param weights The street costs to plan on
    /// @param distanceFactor A cost per unit of length added to every street (see
    ///   distanceFactor), 0 by default
    /// @return std::vector<Id> The ids of the streets to travel, from fromStreetId to
    ///   goalStreetId, or an empty vector if goalStreetId cannot be reached. It is fromStreetId
    ///   alone if the two are the same
    /// @throws std::out_of_range if a street is not in the routing graph
    std::vector<Id> route(Id fromStreetId,
                          Id goalStreetId,
                          RouteWeights weights,
                          double distanceFactor = 0.) const;
    /// @brief Plan a route from a node to the end of a street
    /// @param fromNodeId The node the route starts from: the cost of the first street counts
    /// @param goalStreetId The street to reach and travel: its cost is counted
    /// @param weights The street costs to plan on
    /// @param distanceFactor A cost per unit of length added to every street, 0 by default
    /// @return std::vector<Id> The ids of the streets to travel, from an open outgoing street of
    ///   fromNodeId to goalStreetId, or an empty vector if goalStreetId cannot be reached
    /// @throws std::out_of_range if the node or the street is not in the routing graph
    std::vector<Id> routeFromNode(Id fromNodeId,
                                  Id goalStreetId,
                                  RouteWeights weights,
                                  double distanceFactor = 0.) const;
    /// @brief Get the cost of a route
    /// @param route The ids of the streets of the route
    /// @param weights The street costs to use
    /// @param distanceFactor A cost per unit of length added to every street, 0 by default
    /// @return double The sum of the costs of the streets of the route, but the first
    /// @throws std::out_of_range if a street is not in the routing graph
    double routeCost(std::span<Id const> route,
                     RouteWeights weights,
                     double distanceFactor = 0.) const;
    /// @brief Check whether the routing graph allows a turn
    /// @param fromStreetId The street the agent is on
    /// @param toStreetId The street the agent would take next
    /// @return true if toStreetId is open and the turn is allowed (see
    ///   RoadNetwork::isTurnAllowed), false otherwise or if a street is unknown
    bool isTurnAllowed(Id fromStreetId, Id toStreetId) const;

    /// @brief Check whether the router has been built
    inline bool isBuilt() const noexcept { return !m_turnOffsets.empty(); }
    /// @brief Get the number of streets of the routing graph
    inline std::size_t nEdges() const noexcept { return m_streetIds.size(); }
    /// @brief Check whether the coordinates allow the straight-line distance heuristic
    inline bool hasHeuristic() const noexcept { return m_bHeuristic; }

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
    std::vector<double> m_streetLengths;
    double m_lengthHeuristicFactor{
        0.};  // Lowest length per unit of straight-line distance
    std::unordered_map<Id, std::uint32_t> m_streetIndices;
    // Nodes, by index (ascending id)
    std::unordered_map<Id, std::uint32_t> m_nodeIndices;
    std::vector<double> m_nodeX, m_nodeY;  // Planar coordinates, empty without heuristic
    // Allowed turns from each street, and open outgoing streets of each node (CSR)
    std::vector<std::uint32_t> m_turnOffsets, m_turns;
    std::vector<std::uint32_t> m_nodeOutOffsets, m_nodeOut;
    WeightSet m_freeFlow, m_current;
    EdgeWeight m_edgeWeight{edgeWeightFunction("traveltime")};
    RouteAlgorithm m_algorithm{RouteAlgorithm::ASTAR};
    double m_distanceCost{0.};
    std::optional<double> m_distanceSigma;
    bool m_bHeuristic{false};
    mutable tbb::enumerable_thread_specific<SearchBuffers> m_buffers;

    void m_buildHeuristic(RoadNetwork const& network, std::span<Id const> nodeIds);
    double m_heuristicFactor(std::span<double const> weights) const;
    std::uint32_t m_streetIndex(Id streetId) const;
    std::uint32_t m_nodeIndex(Id nodeId) const;
    WeightSet const& m_weightSet(RouteWeights weights) const noexcept;
    /// @brief Run A* from the given streets to the goal
    /// @param starts The indices of the start streets
    /// @param bCountStartCost Whether the cost of the start streets counts
    std::vector<Id> m_search(std::span<std::uint32_t const> starts,
                             bool bCountStartCost,
                             std::uint32_t goalStreet,
                             WeightSet const& weightSet,
                             double distanceFactor) const;
  };
}  // namespace dsf::mobility
