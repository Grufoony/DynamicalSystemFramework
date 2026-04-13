/// @file       /src/dsf/headers/RoadNetwork.hpp
/// @file       /src/dsf/headers/RoadNetwork.hpp
/// @brief      Defines the RoadNetwork class.
///
/// @details    This file contains the definition of the RoadNetwork class.
///             The RoadNetwork class represents a graph in the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include "../base/Network.hpp"
#include "RoadJunction.hpp"
#include "Intersection.hpp"
#include "TrafficLight.hpp"
#include "PathCollection.hpp"
#include "Roundabout.hpp"
#include "Station.hpp"
#include "Street.hpp"
#include "../utility/Typedef.hpp"
#include "../utility/TypeTraits/is_node.hpp"
#include "../utility/TypeTraits/is_street.hpp"

#include <algorithm>
#include <concepts>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <type_traits>
#include <utility>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cassert>
#include <format>

#include <spdlog/spdlog.h>

namespace dsf::mobility {
  /// @brief The RoadNetwork class represents a graph in the network.
  class RoadNetwork : public Network<RoadJunction, Street> {
  private:
    std::size_t m_capacity;

    /// @brief If every node has coordinates, set the street angles
    /// @details The street angles are set using the node's coordinates.
    void m_setStreetAngles();

    void m_updateMaxAgentCapacity();

    void m_csvEdgesImporter(const std::string& fileName, const char separator = ';');
    void m_csvNodePropertiesImporter(const std::string& fileName,
                                     const char separator = ';');

    void m_jsonEdgesImporter(std::ifstream& file);

  public:
    RoadNetwork() = default;
    // Disable copy constructor and copy assignment operator
    RoadNetwork(const RoadNetwork&) = delete;
    RoadNetwork& operator=(const RoadNetwork&) = delete;
    // Enable move constructor and move assignment operator
    RoadNetwork(RoadNetwork&&) = default;
    RoadNetwork& operator=(RoadNetwork&&) = default;

    /// @brief Get the graph's number of coil streets
    /// @return std::size_t The number of coil streets
    std::size_t nCoils() const;

    /// @brief Get the graph's number of intersections
    /// @return std::size_t The number of intersections
    std::size_t nIntersections() const;
    /// @brief Get the graph's number of roundabouts
    /// @return std::size_t The number of roundabouts
    std::size_t nRoundabouts() const;
    /// @brief Get the graph's number of traffic lights
    /// @return std::size_t The number of traffic lights
    std::size_t nTrafficLights() const;

    /// @brief Adjust the nodes' transport capacity
    /// @details The nodes' capacity is adjusted using the graph's streets transport capacity, which may vary basing on the number of lanes. The node capacity will be set to the sum of the incoming streets' transport capacity.
    void adjustNodeCapacities();
    /// @brief Initialize the traffic lights with random parameters
    /// @param mainRoadPercentage The percentage of main roads for the traffic lights cycles (default is 0.6)
    /// @details Traffic Lights with no parameters set are initialized with random parameters.
    /// Street priorities are assigned considering the number of lanes and the speed limit.
    /// Traffic Lights with an input degree lower than 3 are converted to standard intersections.
    void autoInitTrafficLights(double const mainRoadPercentage = 0.6);
    /// @brief Automatically re-maps street lanes basing on network's topology
    /// @details For example, if one street has the right turn forbidden, then the right lane becomes a straight one
    void autoMapStreetLanes();
    /// @brief Automatically assigns road priorities at intersections, basing on road types
    void autoAssignRoadPriorities();

    /// @brief Describe the RoadNetwork
    /// @param os The output stream to write the description to (default is std::cout)
    void describe(std::ostream& os = std::cout) const;

    /// @brief Import the graph's streets from a file
    /// @param fileName The name of the file to import the streets from.
    /// @details Supports csv, json and geojson file formats.
    /// The file format is deduced from the file extension.
    /// Supported fields:
    /// - id: The id of the street
    /// - source: The id of the source node
    /// - target: The id of the target node
    /// - length: The length of the street, in meters
    /// - nlanes: The number of lanes of the street
    /// - maxspeed: The street's speed limit, in km/h
    /// - name: The name of the street
    /// - geometry: The geometry of the street, as a LINESTRING
    ///
    ///   Next columns are optional (meaning that their absence will not -hopefully- cause any pain):
    ///
    /// - type: The type of the street (e.g. residential, primary, secondary, etc.)
    /// - forbiddenTurns: The forbidden turns of the street, encoding information about street into which the street cannot output agents. The format is a string "sourceId1-targetid1, sourceId2-targetid2,..."
    /// - coilcode: An integer code to identify the coil located on the street
    /// - customWeight: will be stored in the `weight` parameter of the Edge class. You can use it for the shortest path via dsf::weight_functions::customWeight.
    /// - priority: boolean, whether the street is a priority road or not. This information can be used in the traffic light cycle generation.
    /// @param args Additional arguments
    template <typename... TArgs>
    void importEdges(const std::string& fileName, TArgs&&... args);
    /// @brief Import the graph's nodes properties from a file
    /// @param fileName The name of the file to import the nodes properties from.
    /// @param args Additional arguments
    /// @details Supports csv file format. Please specify the separator as second parameter.
    /// Supported fields:
    /// - id: The id of the node
    /// - type: The type of the node, e.g. roundabout, traffic_signals, etc.
    /// - geometry: The geometry of the node, as a POINT
    template <typename... TArgs>
    void importNodeProperties(const std::string& fileName, TArgs&&... args);
    /// @brief Import the graph's traffic lights from a file
    /// @param fileName The name of the file to import the traffic lights from.
    /// @details The file format is csv-like with the ';' separator. Supported columns (in order):
    /// - id: The id of the TrafficLight node
    /// - sourceId: The id of the source node of the incoming street
    /// - cycleTime: The traffic light's cycle time, in seconds
    /// - greenTime: The green time of the considered phase, in time-steps
    /// @throws std::invalid_argument if the file is not found, invalid or the format is not supported
    /// @details The traffic lights are imported from the specified file. If the file does not provide
    ///           sufficient parameters, the behavior of the traffic light initialization is undefined.
    ///           Ensure the file contains valid and complete data for accurate traffic light configuration.
    ///           Street priorities may be assigned based on additional parameters such as the number of lanes
    ///           and the speed limit, if such data is available in the file.
    void importTrafficLights(const std::string& fileName);

    template <typename T1, typename... Tn>
      requires is_node_v<std::remove_reference_t<T1>> &&
               (is_node_v<std::remove_reference_t<Tn>> && ...)
    void addNodes(T1&& node, Tn&&... nodes);

    /// @brief Convert an existing node to a traffic light
    /// @param nodeId The id of the node to convert to a traffic light
    /// @param cycleTime The traffic light's cycle time
    /// @param counter The traffic light's counter initial value. Default is 0
    /// @return A reference to the traffic light
    /// @throws std::invalid_argument if the node does not exist
    TrafficLight& makeTrafficLight(Id const nodeId,
                                   Delay const cycleTime,
                                   Delay const counter = 0);
    /// @brief Convert an existing node into a roundabout
    /// @param nodeId The id of the node to convert to a roundabout
    /// @return A reference to the roundabout
    /// @throws std::invalid_argument if the node does not exist
    Roundabout& makeRoundabout(Id nodeId);

    /// @brief Add a coil (dsf::Counter sensor) on the street with streetId
    /// @param streetId The id of the street to add the coil to
    /// @param name The coil name
    /// @throws std::invalid_argument if the street does not exist
    void addCoil(Id streetId, std::string const& name = std::string());
    /// @brief Convert an existing node into a station
    /// @param nodeId The id of the node to convert to a station
    /// @param managementTime The station's management time
    /// @return A reference to the station
    /// @throws std::invalid_argument if the node does not exist
    Station& makeStation(Id nodeId, const unsigned int managementTime);

    /// @brief Add a street to the graph
    /// @param street A reference to the street to add
    void addStreet(Street&& street);

    template <typename T1>
      requires is_street_v<std::remove_reference_t<T1>>
    void addStreets(T1&& street);

    template <typename T1, typename... Tn>
      requires is_street_v<std::remove_reference_t<T1>> &&
               (is_street_v<std::remove_reference_t<Tn>> && ...)
    void addStreets(T1&& street, Tn&&... streets);

    /// @brief Set the street's status by its id
    /// @param streetId The id of the street
    /// @param status The status to set
    void setStreetStatusById(Id const streetId, RoadStatus const status);
    /// @brief Set the street's status of all streets with the given name
    /// @param name The name to match
    /// @param status The status to set
    void setStreetStatusByName(std::string const& name, RoadStatus const status);
    /// @brief Change the street's number of lanes by its id
    /// @param streetId The id of the street
    /// @param nLanes The new number of lanes
    /// @param speedFactor Optional, The factor to multiply the max speed of the street
    void changeStreetNLanesById(Id const streetId,
                                int const nLanes,
                                std::optional<double> const speedFactor = std::nullopt);
    /// @brief Change the street's number of lanes of all streets with the given name
    /// @param name The name to match
    /// @param nLanes The new number of lanes
    /// @param speedFactor Optional, The factor to multiply the max speed of the street
    void changeStreetNLanesByName(std::string const& name,
                                  int const nLanes,
                                  std::optional<double> const speedFactor = std::nullopt);
    /// @brief Change the street's capacity by its id
    /// @param streetId The id of the street
    /// @param factor The factor to multiply the capacity by
    void changeStreetCapacityById(Id const streetId, double const factor);
    /// @brief Change the street's capacity of all streets with the given name
    /// @param name The name to match
    /// @param factor The factor to multiply the capacity by
    void changeStreetCapacityByName(std::string const& name, double const factor);

    /// @brief Set the streets' stationary weights
    /// @param streetWeights A map where the key is the street id and the value is the street stationary weight. If a street id is not present in the map, its stationary weight is set to 1.0.
    void setStreetStationaryWeights(std::unordered_map<Id, double> const& streetWeights);

    /// @brief Get a street from the graph
    /// @param source The source node
    /// @param destination The destination node
    /// @return A pointer to the street if it exists, nullptr otherwise
    Street const* street(Id source, Id destination) const;

    /// @brief Get the maximum agent capacity
    /// @return std::size_t The maximum agent capacity of the graph
    inline auto capacity() const noexcept { return m_capacity; }

    /// @brief Perform a global Dijkstra search to a target node from all other nodes in the graph
    /// @tparam DynamicsFunc A callable type that takes a const reference to a Street and returns a double representing the edge weight
    /// @param targetId The id of the target node
    /// @param getEdgeWeight A callable that takes a const reference to a Street and returns a double representing the edge weight
    /// @param threshold A threshold value to consider alternative paths
    /// @return A map where each key is a node id and the value is a vector of next hop node ids toward the target
    /// @throws std::out_of_range if the target node does not exist
    template <typename DynamicsFunc>
      requires(std::is_invocable_r_v<double, DynamicsFunc, Street const&>)
    PathCollection allPathsTo(Id const targetId,
                              DynamicsFunc getEdgeWeight,
                              double const threshold = 1e-9) const;

    /// @brief Find the shortest path between two nodes using Dijkstra's algorithm
    /// @tparam DynamicsFunc A callable type that takes a const reference to a Street and returns a double representing the edge weight
    /// @param sourceId The id of the source node
    /// @param targetId The id of the target node
    /// @param getEdgeWeight A callable that takes a const reference to a Street and returns a double representing the edge weight
    /// @param threshold A threshold value to consider alternative paths
    /// @return A map where each key is a node id and the value is a vector of next hop node ids toward the target. Returns an empty map if no path exists
    /// @throws std::out_of_range if the source or target node does not exist
    /// @details Uses Dijkstra's algorithm to find shortest paths from source to target.
    ///          Like allPathsTo, this method tracks all equivalent paths within the threshold, allowing for multiple next hops per node.
    template <typename DynamicsFunc>
      requires(std::is_invocable_r_v<double, DynamicsFunc, Street const&>)
    PathCollection shortestPath(Id const sourceId,
                                Id const targetId,
                                DynamicsFunc getEdgeWeight,
                                double const threshold = 1e-9) const;
  };

  template <typename... TArgs>
  void RoadNetwork::importEdges(const std::string& fileName, TArgs&&... args) {
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }
    auto const fileExt = fileName.substr(fileName.find_last_of('.') + 1);
    if (!fileExtMap.contains(fileExt)) {
      throw std::invalid_argument(
          std::format("File extension ({}) not supported", fileExt));
    }
    switch (fileExtMap.at(fileExt)) {
      case FileExt::CSV:
        spdlog::debug("Importing nodes from CSV file: {}", fileName);
        this->m_csvEdgesImporter(fileName, std::forward<TArgs>(args)...);
        break;
      case FileExt::GEOJSON:
      case FileExt::JSON:
        spdlog::debug("Importing nodes from JSON file: {}", fileName);
        this->m_jsonEdgesImporter(file);
        break;
      default:
        throw std::invalid_argument(
            std::format("File extension ({}) not supported", fileExt));
    }

    spdlog::debug("Successfully imported {} edges", this->nEdges());
  }
  template <typename... TArgs>
  void RoadNetwork::importNodeProperties(const std::string& fileName, TArgs&&... args) {
    if (this->nNodes() == 0) {
      throw std::runtime_error(
          "Cannot import node properties when there are no nodes in the network. Please "
          "import edges or construct network first.");
    }
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }
    auto const fileExt = fileName.substr(fileName.find_last_of('.') + 1);
    if (!fileExtMap.contains(fileExt)) {
      throw std::invalid_argument(
          std::format("File extension ({}) not supported", fileExt));
    }
    switch (fileExtMap.at(fileExt)) {
      case FileExt::CSV:
        spdlog::debug("Importing node properties from CSV file: {}", fileName);
        this->m_csvNodePropertiesImporter(fileName, std::forward<TArgs>(args)...);
        break;
      case FileExt::JSON:
      case FileExt::GEOJSON:
        throw std::invalid_argument(
            "Importing node properties from JSON or GEOJSON files is not supported.");
      default:
        throw std::invalid_argument(
            std::format("File extension ({}) not supported", fileExt));
    }
    spdlog::debug("Successfully imported node properties for {} nodes", nNodes());
  }

  template <typename T1, typename... Tn>
    requires is_node_v<std::remove_reference_t<T1>> &&
             (is_node_v<std::remove_reference_t<Tn>> && ...)
  void RoadNetwork::addNodes(T1&& node, Tn&&... nodes) {
    addNode(std::forward<T1>(node));
    addNodes(std::forward<Tn>(nodes)...);
  }

  template <typename T1>
    requires is_street_v<std::remove_reference_t<T1>>
  void RoadNetwork::addStreets(T1&& street) {
    addStreet(std::move(street));
  }

  template <typename T1, typename... Tn>
    requires is_street_v<std::remove_reference_t<T1>> &&
             (is_street_v<std::remove_reference_t<Tn>> && ...)
  void RoadNetwork::addStreets(T1&& street, Tn&&... streets) {
    addStreet(std::move(street));
    addStreets(std::forward<Tn>(streets)...);
  }

  template <typename DynamicsFunc>
    requires(std::is_invocable_r_v<double, DynamicsFunc, Street const&>)
  PathCollection RoadNetwork::allPathsTo(Id const targetId,
                                         DynamicsFunc f,
                                         double const threshold) const {
    auto const& nodes = this->nodes();

    // Distance from each node to the source (going backward)
    std::unordered_map<Id, double> distToTarget;
    distToTarget.reserve(nNodes());
    // Next hop from each node toward the source
    PathCollection nextHopsToTarget;

    // Priority queue: pair<distance, nodeId> (min-heap)
    std::priority_queue<std::pair<double, Id>,
                        std::vector<std::pair<double, Id>>,
                        std::greater<>>
        pq;

    // Initialize all nodes with infinite distance
    std::for_each(nodes.cbegin(), nodes.cend(), [&](auto const& pair) {
      distToTarget[pair.first] = std::numeric_limits<double>::infinity();
      nextHopsToTarget[pair.first] = std::vector<Id>();
    });

    // Target has distance 0 to itself
    distToTarget[targetId] = 0.0;
    pq.push({0.0, targetId});

    while (!pq.empty()) {
      auto [currentDist, currentNode] = pq.top();
      pq.pop();

      // Skip if we've already found a better path to this node
      if (currentDist > distToTarget[currentNode]) {
        continue;
      }

      // Explore all incoming edges (nodes that can reach currentNode)
      auto const& inEdges = node(currentNode).ingoingEdges();
      for (auto const& inEdgeId : inEdges) {
        // Skip closed roads
        if (edge(inEdgeId).roadStatus() == RoadStatus::CLOSED) {
          continue;
        }
        Id neighborId = edge(inEdgeId).source();

        // Calculate the weight of the edge from neighbor to currentNode using the dynamics function
        double edgeWeight = f(this->edge(inEdgeId));
        double newDistToTarget = distToTarget[currentNode] + edgeWeight;

        // If we found a shorter path from neighborId to source
        if (newDistToTarget < distToTarget[neighborId]) {
          distToTarget[neighborId] = newDistToTarget;
          nextHopsToTarget[neighborId].clear();
          nextHopsToTarget[neighborId].push_back(currentNode);
          pq.push({newDistToTarget, neighborId});
        }
        // If we found an equally good path, add it as alternative
        else if (newDistToTarget < (1. + threshold) * distToTarget[neighborId]) {
          spdlog::debug(
              "Found alternative path to node {} with distance {:.6f} (existing: {:.6f}) "
              "for threshold {:.6f}",
              neighborId,
              newDistToTarget,
              distToTarget[neighborId],
              threshold);
          // Check if currentNode is not already in the nextHops
          auto& hops = nextHopsToTarget[neighborId];
          if (std::find(hops.begin(), hops.end(), currentNode) == hops.end()) {
            hops.push_back(currentNode);
          }
        }
      }
    }

    // Build result: only include reachable nodes (excluding source)
    PathCollection result;
    for (auto const& [nodeId, hops] : nextHopsToTarget) {
      if (nodeId != targetId &&
          distToTarget[nodeId] != std::numeric_limits<double>::infinity() &&
          !hops.empty()) {
        result[nodeId] = hops;
      }
    }

    return result;
  }

  template <typename DynamicsFunc>
    requires(std::is_invocable_r_v<double, DynamicsFunc, Street const&>)
  PathCollection RoadNetwork::shortestPath(Id const sourceId,
                                           Id const targetId,
                                           DynamicsFunc f,
                                           double const threshold) const {
    // If source equals target, return empty map (no intermediate hops needed)
    if (sourceId == targetId) {
      return PathCollection{};
    }
    // Check if source node exists
    if (!this->nodes().contains(sourceId)) {
      throw std::out_of_range(
          std::format("Source node with id {} does not exist in the graph", sourceId));
    }
    // Check if target node exists
    if (!this->nodes().contains(targetId)) {
      throw std::out_of_range(
          std::format("Target node with id {} does not exist in the graph", targetId));
    }
    auto const& nodes = this->nodes();

    // Distance from each node to the target (going backward)
    std::unordered_map<Id, double> distToTarget;
    distToTarget.reserve(nNodes());
    // Next hop from each node toward the target
    PathCollection nextHopsToTarget;

    // Priority queue: pair<distance, nodeId> (min-heap)
    std::priority_queue<std::pair<double, Id>,
                        std::vector<std::pair<double, Id>>,
                        std::greater<>>
        pq;

    // Initialize all nodes with infinite distance
    std::for_each(nodes.cbegin(), nodes.cend(), [&](auto const& pair) {
      distToTarget[pair.first] = std::numeric_limits<double>::infinity();
      nextHopsToTarget[pair.first] = std::vector<Id>();
    });

    // Target has distance 0 to itself
    distToTarget[targetId] = 0.0;
    pq.push({0.0, targetId});

    while (!pq.empty()) {
      auto [currentDist, currentNode] = pq.top();
      pq.pop();

      // Skip if we've already found a better path to this node
      if (currentDist > distToTarget[currentNode]) {
        continue;
      }

      // If we've reached the source, we can stop early
      if (currentNode == sourceId) {
        break;
      }

      // Explore all incoming edges (nodes that can reach currentNode)
      auto const& inEdges = node(currentNode).ingoingEdges();
      for (auto const& inEdgeId : inEdges) {
        // Skip closed roads
        if (edge(inEdgeId).roadStatus() == RoadStatus::CLOSED) {
          continue;
        }
        Id neighborId = edge(inEdgeId).source();

        // Calculate the weight of the edge from neighbor to currentNode using the dynamics function
        double edgeWeight = f(this->edge(inEdgeId));
        double newDistToTarget = distToTarget[currentNode] + edgeWeight;

        // If we found a shorter path from neighborId to target
        if (newDistToTarget < distToTarget[neighborId]) {
          distToTarget[neighborId] = newDistToTarget;
          nextHopsToTarget[neighborId].clear();
          nextHopsToTarget[neighborId].push_back(currentNode);
          pq.push({newDistToTarget, neighborId});
        }
        // If we found an equally good path, add it as alternative
        else if (newDistToTarget < (1. + threshold) * distToTarget[neighborId]) {
          spdlog::debug(
              "Found alternative path to node {} with distance {:.6f} (existing: {:.6f}) "
              "for threshold {:.6f}",
              neighborId,
              newDistToTarget,
              distToTarget[neighborId],
              threshold);
          // Check if currentNode is not already in the nextHops
          auto& hops = nextHopsToTarget[neighborId];
          if (std::find(hops.begin(), hops.end(), currentNode) == hops.end()) {
            hops.push_back(currentNode);
          }
        }
      }
    }

    // Check if target is reachable from source
    if (distToTarget[sourceId] == std::numeric_limits<double>::infinity()) {
      return PathCollection{};
    }

    // Build result: only include nodes on the path from source to target
    PathCollection result;
    std::unordered_set<Id> nodesOnPath;

    // Start from source and traverse to target using BFS to find all nodes on valid paths
    std::queue<Id> queue;
    queue.push(sourceId);
    nodesOnPath.insert(sourceId);

    while (!queue.empty()) {
      Id current = queue.front();
      queue.pop();

      if (current == targetId) {
        continue;
      }

      // Add this node's next hops to the result if they exist
      if (nextHopsToTarget.contains(current) && !nextHopsToTarget[current].empty()) {
        result[current] = nextHopsToTarget[current];

        // Add next hops to the queue if not already visited
        for (Id nextHop : nextHopsToTarget[current]) {
          if (!nodesOnPath.contains(nextHop)) {
            nodesOnPath.insert(nextHop);
            queue.push(nextHop);
          }
        }
      }
    }

    return result;
  }
};  // namespace dsf::mobility
