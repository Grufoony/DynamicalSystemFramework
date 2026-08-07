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
#include "Roundabout.hpp"
#include "Station.hpp"
#include "Street.hpp"
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
    std::size_t m_capacity = 0;

    std::unordered_map<Id, double> m_computeEdgeDistancesToTarget(
        Id const targetEdgeId) const final;

    /// @brief If every node has coordinates, set the street angles
    /// @details The street angles are set using the node's coordinates.
    void m_setStreetAngles();

    void m_updateMaxAgentCapacity();

    void m_csvEdgesImporter(const std::string& fileName, const char separator = ';');
    void m_csvNodePropertiesImporter(const std::string& fileName,
                                     const char separator = ';');

    void m_jsonEdgesImporter(std::ifstream& file);

  public:
    RoadNetwork();
    // Disable copy constructor and copy assignment operator
    RoadNetwork(const RoadNetwork&) = delete;
    RoadNetwork& operator=(const RoadNetwork&) = delete;
    // Enable move constructor and move assignment operator
    RoadNetwork(RoadNetwork&&) = default;
    RoadNetwork& operator=(RoadNetwork&&) = default;

    virtual ~RoadNetwork() = default;

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
    /// @brief Auto-initialise traffic light phases from street geometry.
    /// @param mainRoadPercentage Fraction of the cycle time allocated to
    ///        priority streets (default 0.6).
    /// @param defaultCycleDuration The default cycle duration in ticks (default 90).
    /// @throws std::invalid_argument If mainRoadPercentage is not in the range (0, 1).
    /// @details For each TrafficLight node that has no phases yet:
    ///   - Nodes with fewer than 3 ingoing edges are downgraded to plain
    ///     Intersection nodes.
    ///   - Priority streets are detected by name, speed limit, lane count
    ///     or angle (in that order of precedence).
    ///   - Two phases are created:
    ///       Phase 0 (priority streets)    — duration = mainRoadPercentage * DEFAULT_CYCLE
    ///       Phase 1 (non-priority streets) — duration = remaining ticks
    ///   - Every street is added to its phase with Direction::ANY
    ///   - Nodes whose phases have been set manually (via JSON config or
    ///     importTrafficLights()) are skipped.
    void autoInitTrafficLights(double const mainRoadPercentage = 0.6,
                               dsf::Delay const defaultCycleDuration = 90);
    /// @brief Automatically re-maps street lanes basing on network's topology
    /// @details For example, if one street has the right turn forbidden, then the right lane becomes a straight one
    void autoMapStreetLanes();
    /// @brief Automatically assigns road priorities at intersections, basing on road types
    void autoAssignRoadPriorities();
    /// @brief Set the edge weight function based on a string identifier
    /// @param strv_weight The string identifier of the weight function. Supported values are "travelTime", "length" and any custom attribute name.
    /// @param threshold An optional threshold to apply to the weight function. The effective weight will be weight * (1 + threshold). This can be used to increase the weight of certain paths and thus make them less likely to be chosen by agents when using a weight-based path update strategy.
    void setEdgeWeight(std::string_view const strv_weight,
                       std::optional<double> const threshold = std::nullopt) final;

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
    /// - priority: boolean, whether the street is a priority road or not. This information can be used in the traffic light cycle generation.
    /// - any additional CSV column or JSON field will be imported as an edge attribute, with automatic type inference among bool, int64, double, string and null.
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
    /// @brief Import traffic light phases from a legacy CSV file.
    /// @param fileName The path to the CSV file.
    /// @details The file uses ';' as separator. Expected columns (in order):
    ///   - id        : The TrafficLight node id
    ///   - sourceId  : The source node id of the ingoing street
    ///   - cycleTime : Total cycle duration in ticks
    ///   - greenTime : Green duration for this street in ticks
    ///
    /// The importer reconstructs a two-phase TrafficLight from each node's
    /// entries:
    ///   - Streets whose greenTime equals the first greenTime seen for that
    ///     node → Phase 0 (duration = firstGreenTime).
    ///   - Remaining streets → Phase 1 (duration = cycleTime − firstGreenTime).
    ///
    /// All streets are added with Direction::ANY. Use the JSON config block
    /// under road_network.traffic_lights for direction-level control.
    /// @throws std::runtime_error if the file cannot be opened.
    void importTrafficLights(const std::string& fileName);

    template <typename T1, typename... Tn>
      requires is_node_v<std::remove_reference_t<T1>> &&
               (is_node_v<std::remove_reference_t<Tn>> && ...)
    void addNodes(T1&& node, Tn&&... nodes);

    /// @brief Convert an existing node to a traffic light.
    /// @param nodeId The id of the node to convert.
    /// @return A reference to the new TrafficLight node.
    /// @throws std::invalid_argument if the node does not exist.
    /// @note Phases are NOT set here. Call autoInitTrafficLights() afterwards
    ///       for geometry-based auto-deduction, or configure phases explicitly
    ///       via TrafficLight::setPhases() / TrafficLight::addPhase().
    TrafficLight& makeTrafficLight(Id const nodeId);
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

    /// @brief Get a street from the graph
    /// @param source The source node
    /// @param destination The destination node
    /// @return A pointer to the street if it exists, nullptr otherwise
    Street const* street(Id source, Id destination) const;

    /// @brief Get the maximum agent capacity
    /// @return std::size_t The maximum agent capacity of the graph
    inline auto capacity() const noexcept { return m_capacity; }

    PathCollection allEdgePathsTo(Id const targetEdgeId) const final;

    void exportTrafficLights(std::string_view const fileName) const;

    /// @brief Export the graph's edges and nodes to two CSV files in the specified folder
    /// @param folder The folder to export the files to
    void exportCSV(std::string_view const folder) const;
  };

  inline std::unordered_map<Id, double> RoadNetwork::m_computeEdgeDistancesToTarget(
      Id const targetEdgeId) const {
    std::unordered_map<Id, double> distToTarget;
    distToTarget.reserve(nEdges());
    for (auto const edgeId : m_edges.keys()) {
      distToTarget.emplace(edgeId, std::numeric_limits<double>::infinity());
    }

    std::priority_queue<std::pair<double, Id>,
                        std::vector<std::pair<double, Id>>,
                        std::greater<>>
        pq;

    distToTarget[targetEdgeId] = 0.0;
    pq.push({0.0, targetEdgeId});

    while (!pq.empty()) {
      auto const [currentDist, currentEdgeId] = pq.top();
      pq.pop();

      if (currentDist > distToTarget.at(currentEdgeId)) {
        continue;
      }

      auto const& currentEdge = this->edge(currentEdgeId);
      // The actual junction where the turn happens is the SOURCE of the current edge in backward search
      auto const& turningNode = this->node(currentEdge.source());

      for (auto const& inEdgeId : turningNode.ingoingEdges()) {
        auto const& inEdge = this->edge(inEdgeId);
        if (!inEdge.isActive() || inEdge.forbiddenTurns().contains(currentEdgeId)) {
          continue;
        }

        // 1. Detect if this structural pair constitutes a U-turn
        if ((currentEdge.target() == inEdge.source()) && (!turningNode.isRoundabout())) {
          // 2. Check if there are any ALTERNATIVE legal moves out of this dead end
          bool hasAlternativeMove = false;
          for (auto const& altOutEdgeId : turningNode.outgoingEdges()) {
            if (altOutEdgeId == currentEdgeId) {
              continue;  // Ignore the U-turn road itself
            }

            auto const& altOutEdge = this->edge(altOutEdgeId);
            if (altOutEdge.isActive() &&
                !inEdge.forbiddenTurns().contains(altOutEdgeId)) {
              hasAlternativeMove = true;
              break;  // An alternative escape route exists!
            }
          }

          // 3. If an alternative exists, enforce the restriction.
          // If NO alternative exists (dead end), we skip this 'continue' and allow the U-turn!
          if (hasAlternativeMove) {
            continue;
          }
        }

        // Standard Dijkstra relaxation continues below...
        auto const candidateDistance = currentDist + m_weightFunction(currentEdge);
        auto& neighborDist = distToTarget.at(inEdgeId);
        if (candidateDistance < neighborDist) {
          neighborDist = candidateDistance;
          pq.push({candidateDistance, inEdgeId});
        }
      }
    }

    return distToTarget;
  }

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

  inline PathCollection RoadNetwork::allEdgePathsTo(Id const targetEdgeId) const {
    spdlog::debug("Computing all edge paths to target edge {}", targetEdgeId);
    auto const distToTarget = m_computeEdgeDistancesToTarget(targetEdgeId);
    PathCollection result;
    for (auto const& pEdge : m_edges.values()) {
      auto const edgeId{pEdge->id()};
      if (edgeId == targetEdgeId) {
        continue;
      }

      auto const edgeDistToTarget = distToTarget.at(edgeId);
      if (edgeDistToTarget == std::numeric_limits<double>::infinity()) {
        continue;
      }

      double edgeBudget = edgeDistToTarget;
      if (m_weightThreshold.has_value()) {
        edgeBudget *= (1.0 + *m_weightThreshold);
      }

      auto const& targetNode = this->node(pEdge->target());
      auto const& outgoingEdges = targetNode.outgoingEdges();
      std::vector<Id> hops;
      hops.reserve(outgoingEdges.size());

      for (auto const& nextEdgeId : outgoingEdges) {
        auto const& pNextEdge = this->edge(nextEdgeId);
        if (!pNextEdge.isActive() || pEdge->forbiddenTurns().contains(nextEdgeId)) {
          continue;
        }

        // 1. Detect if this structural pair constitutes a forward U-turn
        if ((pNextEdge.target() == pEdge->source()) && (!targetNode.isRoundabout())) {
          // 2. Scan for alternative valid moves out of this intersection
          bool hasAlternativeMove = false;
          for (auto const& altNextEdgeId : outgoingEdges) {
            if (altNextEdgeId == nextEdgeId) {
              continue;  // Ignore the U-turn road itself
            }

            auto const& altNextEdge = this->edge(altNextEdgeId);
            if (altNextEdge.isActive() &&
                !pEdge->forbiddenTurns().contains(altNextEdgeId)) {
              hasAlternativeMove = true;
              break;  // An alternative legal escape route exists!
            }
          }

          // 3. If an alternative exists, enforce the restriction.
          // If no alternative exists (dead end cul-de-sac), we skip the continue and allow the U-turn!
          if (hasAlternativeMove) {
            continue;
          }
        }

        auto const nextDistToTarget = distToTarget.at(nextEdgeId);
        if (nextDistToTarget == std::numeric_limits<double>::infinity()) {
          continue;
        }

        // Keep hop transitions acyclic so path expansion remains finite.
        if (nextDistToTarget + 1e-12 >= edgeDistToTarget) {
          continue;
        }

        auto const fullPathCost = m_weightFunction(pNextEdge) + nextDistToTarget;
        if (fullPathCost <= edgeBudget + 1e-12 &&
            std::find(hops.begin(), hops.end(), nextEdgeId) == hops.end()) {
          hops.push_back(nextEdgeId);
        }
      }

      if (!hops.empty()) {
        result[edgeId] = hops;
      }
    }

    return result;
  }

};  // namespace dsf::mobility
