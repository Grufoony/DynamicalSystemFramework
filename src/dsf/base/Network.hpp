#pragma once

#include "Edge.hpp"
#include "Node.hpp"
#include "PathCollection.hpp"
#include "../utility/Typedef.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <span>
#include <set>
#include <stack>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/combinable.h>
#include <tbb/parallel_for.h>

namespace dsf {
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  class Network {
  protected:
    std::unordered_map<Id, std::unique_ptr<node_t>> m_nodes;
    std::unordered_map<Id, std::unique_ptr<edge_t>> m_edges;

    std::function<double(edge_t const&)> m_weightFunction =
        []([[maybe_unused]] edge_t const& edge) {
          return 1.0;  // Default = unweighted graph
        };
    std::optional<double> m_weightThreshold = std::nullopt;

    constexpr inline auto m_cantorHash(Id u, Id v) const {
      return ((u + v) * (u + v + 1)) / 2 + v;
    }
    constexpr inline auto m_cantorHash(std::pair<Id, Id> const& idPair) const {
      return m_cantorHash(idPair.first, idPair.second);
    }

  private:
    std::unordered_map<Id, double> m_computeDistancesToTarget(Id const targetId) const;
    std::unordered_map<Id, double> m_computeDistancesFromSource(Id const sourceId) const;

  public:
    /// @brief Construct a new empty Network object
    Network() = default;

    /// @brief Get the nodes as an unordered map
    inline auto const& nodes() const noexcept { return m_nodes; }
    /// @brief Get the edges as an unordered map
    inline auto const& edges() const noexcept { return m_edges; }
    /// @brief Get the number of nodes
    inline auto nNodes() const noexcept { return m_nodes.size(); }
    /// @brief Get the number of edges
    inline auto nEdges() const noexcept { return m_edges.size(); }

    template <typename TNode = node_t, typename... TArgs>
      requires(std::is_base_of_v<node_t, TNode> &&
               std::constructible_from<TNode, TArgs...>)
    void addNode(TArgs&&... args);

    template <typename TNode = node_t>
      requires(std::is_base_of_v<node_t, TNode>)
    void addNDefaultNodes(std::size_t const n);

    template <typename TEdge = edge_t, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> &&
               std::constructible_from<TEdge, TArgs...>)
    void addEdge(TArgs&&... args);

    virtual void setEdgeWeight(std::string_view const strv_weight,
                               std::optional<double> const threshold = std::nullopt) = 0;

    /// @brief Get a node by id
    /// @param nodeId The id of the node to get
    /// @return const node_t& A const reference to the node with the given id
    inline const auto& node(Id const nodeId) const { return *m_nodes.at(nodeId); };
    /// @brief Get a node by id
    /// @param nodeId The id of the node to get
    /// @return node_t& A reference to the node with the given id
    inline auto& node(Id const nodeId) { return *m_nodes.at(nodeId); };
    /// @brief Get an edge by id
    /// @param edgeId The id of the edge to get
    /// @return const edge_t& A const reference to the edge with the given id
    inline const auto& edge(Id const edgeId) const { return *m_edges.at(edgeId); };
    /// @brief Get an edge by id
    /// @param edgeId The id of the edge to get
    /// @return edge_t& A reference to the edge with the given id
    inline auto& edge(Id const edgeId) { return *m_edges.at(edgeId); }

    edge_t& edge(Id const source, Id const target) const;
    /// @brief Get a node by id and cast it to a derived type
    /// @tparam TNode The expected type of the node
    /// @param nodeId The id of the node to get
    /// @return TNode& A reference to the node with the given id, cast to the expected type
    template <typename TNode>
      requires(std::is_base_of_v<node_t, TNode>)
    inline auto& node(Id const nodeId) {
      return dynamic_cast<TNode&>(node(nodeId));
    }
    /// @brief Get an edge by id and cast it to a derived type
    /// @tparam TEdge The expected type of the edge
    /// @param edgeId The id of the edge to get
    /// @return TEdge& A reference to the edge with the given id, cast to the expected type
    template <typename TEdge>
      requires(std::is_base_of_v<edge_t, TEdge>)
    inline auto& edge(Id const edgeId) {
      return dynamic_cast<TEdge&>(edge(edgeId));
    }

    /// @brief Perform a global Dijkstra search to a target node from all other nodes in the graph
    /// @param threshold Relative tolerance on full path cost from each node to the target
    /// @return A map where each key is a node id and the value is a vector of next hop node ids toward the target
    /// @throws std::out_of_range if the target node does not exist
    /// @details Keeps only transitions that strictly decrease the precomputed
    ///          shortest distance to target. This makes the hop graph acyclic,
    ///          so PathCollection::explode remains finite.
    PathCollection allPathsTo(Id const targetId) const;

    /// @brief Find the shortest path between two nodes using Dijkstra's algorithm
    /// @param sourceId The id of the source node
    /// @param targetId The id of the target node
    /// @return A map where each key is a node id and the value is a vector of next hop node ids toward the target. Returns an empty map if no path exists
    /// @throws std::out_of_range if the source or target node does not exist
    /// @details Uses Dijkstra's algorithm to compute strict distances to target, then
    ///          includes only transitions that both: (1) strictly decrease the
    ///          target distance (acyclic), and (2) are consistent with shortest
    ///          source-distance labels. The second constraint keeps the returned
    ///          PathCollection sound when exploded, i.e. it avoids combining
    ///          prefix-dependent hops into over-budget paths.
    PathCollection shortestPath(Id const sourceId, Id const targetId) const;

    /// @brief Compute node weighted betweenness centralities using Brandes' algorithm
    void computeBetweennessCentralities();

    /// @brief Compute edge weighted betweenness centralities using Brandes' algorithm
    void computeEdgeBetweennessCentralities();

    /// @brief Compute edge betweenness centralities using Yen's K-shortest paths.
    ///
    /// @details For every ordered source–target pair (s, t) with s ≠ t the method
    ///          finds up to K loopless shortest paths using Yen's algorithm.
    ///          Each edge e that appears in at least one of those paths receives
    ///          an increment equal to the number of those K paths that traverse it.
    ///          After all pairs are processed the raw counts are normalised by
    ///          (N−1)·(N−2), the number of ordered pairs in a directed graph of N
    ///          nodes (same denominator used by the standard Brandes formulation).
    ///
    ///          The computation is parallelised over source nodes with Intel TBB:
    ///          each thread maintains its own accumulator map, which are merged
    ///          sequentially once all threads finish.
    ///
    /// @param  K              Maximum number of shortest paths per (s, t) pair.
    ///                        K = 1 reproduces single-shortest-path betweenness.
    void computeEdgeKBetweennessCentralities(std::size_t const K);
  };

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TNode, typename... TArgs>
    requires(std::is_base_of_v<node_t, TNode> && std::constructible_from<TNode, TArgs...>)
  void Network<node_t, edge_t>::addNode(TArgs&&... args) {
    auto pNode = std::make_unique<TNode>(std::forward<TArgs>(args)...);
    if (m_nodes.contains(pNode->id())) {
      throw std::invalid_argument(
          std::format("Node with id {} already exists in the network.", pNode->id()));
    }
    m_nodes[pNode->id()] = std::move(pNode);
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TNode>
    requires(std::is_base_of_v<node_t, TNode>)
  void Network<node_t, edge_t>::addNDefaultNodes(std::size_t const n) {
    auto const currentSize{m_nodes.size()};
    for (std::size_t i = 0; i < n; ++i) {
      addNode<TNode>(static_cast<Id>(currentSize + i));
    }
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge, typename... TArgs>
    requires(std::is_base_of_v<edge_t, TEdge> && std::constructible_from<TEdge, TArgs...>)
  void Network<node_t, edge_t>::addEdge(TArgs&&... args) {
    TEdge tmpEdge(std::forward<TArgs>(args)...);
    if (m_edges.contains(tmpEdge.id())) {
      throw std::invalid_argument(
          std::format("Edge with id {} already exists in the network.", tmpEdge.id()));
    }
    auto const& geometry{tmpEdge.geometry()};
    auto const& sourceNodeId = tmpEdge.source();
    auto const& targetNodeId = tmpEdge.target();

    if (!m_nodes.contains(sourceNodeId)) {
      if (!geometry.empty()) {
        addNode(tmpEdge.source(), geometry.front());
      } else {
        addNode(tmpEdge.source());
      }
    }

    if (!m_nodes.contains(targetNodeId)) {
      if (!geometry.empty()) {
        addNode(tmpEdge.target(), geometry.back());
      } else {
        addNode(tmpEdge.target());
      }
    }

    auto& sourceNode = node(sourceNodeId);
    auto& targetNode = node(targetNodeId);
    sourceNode.addOutgoingEdge(tmpEdge.id());
    targetNode.addIngoingEdge(tmpEdge.id());
    if (geometry.empty()) {
      if (sourceNode.geometry().has_value() && targetNode.geometry().has_value()) {
        tmpEdge.setGeometry(
            dsf::geometry::PolyLine{*sourceNode.geometry(), *targetNode.geometry()});
      }
    }
    m_edges.emplace(tmpEdge.id(), std::make_unique<TEdge>(std::move(tmpEdge)));
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  edge_t& Network<node_t, edge_t>::edge(Id const source, Id const target) const {
    auto const it = std::find_if(
        m_edges.cbegin(), m_edges.cend(), [source, target](auto const& pair) {
          return pair.second->source() == source && pair.second->target() == target;
        });
    if (it == m_edges.cend()) {
      throw std::out_of_range(
          std::format("Edge with source {} and target {} not found.", source, target));
    }
    return *it->second;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  inline std::unordered_map<Id, double>
  Network<node_t, edge_t>::m_computeDistancesToTarget(Id const targetId) const {
    std::unordered_map<Id, double> distToTarget;
    distToTarget.reserve(nNodes());
    for (auto const& pair : m_nodes) {
      distToTarget.emplace(pair.first, std::numeric_limits<double>::infinity());
    }

    std::priority_queue<std::pair<double, Id>,
                        std::vector<std::pair<double, Id>>,
                        std::greater<>>
        pq;

    distToTarget[targetId] = 0.0;
    pq.push({0.0, targetId});

    while (!pq.empty()) {
      auto const [currentDist, currentNode] = pq.top();
      pq.pop();

      if (currentDist > distToTarget.at(currentNode)) {
        continue;
      }

      for (auto const& inEdgeId : this->node(currentNode).ingoingEdges()) {
        auto const& inEdge = this->edge(inEdgeId);
        if (!inEdge.isActive()) {
          continue;
        }

        auto const neighborId = inEdge.source();
        auto const candidateDistance = currentDist + m_weightFunction(inEdge);
        auto& neighborDist = distToTarget.at(neighborId);
        if (candidateDistance < neighborDist) {
          neighborDist = candidateDistance;
          pq.push({candidateDistance, neighborId});
        }
      }
    }

    return distToTarget;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  inline std::unordered_map<Id, double>
  Network<node_t, edge_t>::m_computeDistancesFromSource(Id const sourceId) const {
    std::unordered_map<Id, double> distFromSource;
    distFromSource.reserve(nNodes());
    for (auto const& pair : m_nodes) {
      distFromSource.emplace(pair.first, std::numeric_limits<double>::infinity());
    }

    std::priority_queue<std::pair<double, Id>,
                        std::vector<std::pair<double, Id>>,
                        std::greater<>>
        pq;

    distFromSource[sourceId] = 0.0;
    pq.push({0.0, sourceId});

    while (!pq.empty()) {
      auto const [currentDist, currentNode] = pq.top();
      pq.pop();

      if (currentDist > distFromSource.at(currentNode)) {
        continue;
      }

      for (auto const& outEdgeId : this->node(currentNode).outgoingEdges()) {
        auto const& outEdge = this->edge(outEdgeId);
        if (!outEdge.isActive()) {
          continue;
        }

        auto const neighborId = outEdge.target();
        auto const candidateDistance = currentDist + m_weightFunction(outEdge);
        auto& neighborDist = distFromSource.at(neighborId);
        if (candidateDistance < neighborDist) {
          neighborDist = candidateDistance;
          pq.push({candidateDistance, neighborId});
        }
      }
    }

    return distFromSource;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  inline PathCollection Network<node_t, edge_t>::allPathsTo(Id const targetId) const {
    auto const distToTarget = m_computeDistancesToTarget(targetId);
    PathCollection result;
    for (auto const& [nodeId, pNode] : m_nodes) {
      if (nodeId == targetId) {
        continue;
      }

      auto const nodeDistToTarget = distToTarget.at(nodeId);
      if (nodeDistToTarget == std::numeric_limits<double>::infinity()) {
        continue;
      }

      double nodeBudget = nodeDistToTarget;
      if (m_weightThreshold.has_value()) {
        nodeBudget *= (1.0 + *m_weightThreshold);
      }
      std::vector<Id> hops;
      hops.reserve(pNode->outgoingEdges().size());

      for (auto const& outEdgeId : pNode->outgoingEdges()) {
        auto const& outEdge = this->edge(outEdgeId);
        if (!outEdge.isActive()) {
          continue;
        }

        auto const nextNodeId = outEdge.target();
        auto const nextDistToTarget = distToTarget.at(nextNodeId);
        if (nextDistToTarget == std::numeric_limits<double>::infinity()) {
          continue;
        }

        // Keep hop transitions acyclic so path expansion remains finite.
        if (nextDistToTarget + 1e-12 >= nodeDistToTarget) {
          continue;
        }

        auto const fullPathCost = m_weightFunction(outEdge) + nextDistToTarget;
        if (fullPathCost <= nodeBudget + 1e-12 &&
            std::find(hops.begin(), hops.end(), nextNodeId) == hops.end()) {
          hops.push_back(nextNodeId);
        }
      }

      if (!hops.empty()) {
        result[nodeId] = hops;
      }
    }

    return result;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  inline PathCollection Network<node_t, edge_t>::shortestPath(Id const sourceId,
                                                              Id const targetId) const {
    if (sourceId == targetId) {
      return PathCollection{};
    }
    if (!this->nodes().contains(sourceId)) {
      throw std::out_of_range(
          std::format("Source node with id {} does not exist in the graph", sourceId));
    }
    if (!this->nodes().contains(targetId)) {
      throw std::out_of_range(
          std::format("Target node with id {} does not exist in the graph", targetId));
    }

    auto const distToTarget = m_computeDistancesToTarget(targetId);

    auto const sourceBestDistance = distToTarget.at(sourceId);
    if (sourceBestDistance == std::numeric_limits<double>::infinity()) {
      return PathCollection{};
    }

    double sourceBudget = sourceBestDistance;
    if (m_weightThreshold.has_value()) {
      sourceBudget *= (1.0 + *m_weightThreshold);
    }
    auto const distFromSource = m_computeDistancesFromSource(sourceId);

    PathCollection candidate;
    std::unordered_map<Id, std::vector<Id>> reverseCandidate;

    for (auto const& [nodeId, pNode] : this->nodes()) {
      auto const nodeDistFromSource = distFromSource.at(nodeId);
      auto const nodeDistToTarget = distToTarget.at(nodeId);
      if (nodeDistFromSource == std::numeric_limits<double>::infinity() ||
          nodeDistToTarget == std::numeric_limits<double>::infinity()) {
        continue;
      }

      for (auto const& outEdgeId : pNode->outgoingEdges()) {
        auto const& outEdge = this->edge(outEdgeId);
        if (!outEdge.isActive()) {
          continue;
        }

        auto const nextNodeId = outEdge.target();
        auto const nextDistToTarget = distToTarget.at(nextNodeId);
        if (nextDistToTarget == std::numeric_limits<double>::infinity()) {
          continue;
        }

        // Keep transitions acyclic and convergent for finite path expansion.
        if (nextDistToTarget + 1e-12 >= nodeDistToTarget) {
          continue;
        }

        auto const edgeWeight = m_weightFunction(outEdge);
        auto const nextDistFromSource = distFromSource.at(nextNodeId);
        auto const projectedDistFromSource = nodeDistFromSource + edgeWeight;

        // Keep intermediate transitions source-distance-consistent so all
        // prefixes to a node share the same cost label.
        if (nextNodeId != targetId &&
            (projectedDistFromSource > nextDistFromSource + 1e-12 ||
             projectedDistFromSource + 1e-12 < nextDistFromSource)) {
          continue;
        }

        auto const optimisticCost = projectedDistFromSource + nextDistToTarget;
        if (optimisticCost > sourceBudget + 1e-12) {
          continue;
        }

        auto& hops = candidate[nodeId];
        if (std::find(hops.begin(), hops.end(), nextNodeId) == hops.end()) {
          hops.push_back(nextNodeId);

          auto& reverseHops = reverseCandidate[nextNodeId];
          if (std::find(reverseHops.begin(), reverseHops.end(), nodeId) ==
              reverseHops.end()) {
            reverseHops.push_back(nodeId);
          }
        }
      }
    }

    std::unordered_set<Id> reachableFromSource;
    std::vector<Id> stack{sourceId};
    while (!stack.empty()) {
      auto const currentNode = stack.back();
      stack.pop_back();

      if (!reachableFromSource.insert(currentNode).second) {
        continue;
      }

      auto const it = candidate.find(currentNode);
      if (it == candidate.end()) {
        continue;
      }

      for (auto const nextNodeId : it->second) {
        if (!reachableFromSource.contains(nextNodeId)) {
          stack.push_back(nextNodeId);
        }
      }
    }

    std::unordered_set<Id> canReachTarget;
    stack.push_back(targetId);
    while (!stack.empty()) {
      auto const currentNode = stack.back();
      stack.pop_back();

      if (!canReachTarget.insert(currentNode).second) {
        continue;
      }

      auto const it = reverseCandidate.find(currentNode);
      if (it == reverseCandidate.end()) {
        continue;
      }

      for (auto const previousNodeId : it->second) {
        if (!canReachTarget.contains(previousNodeId)) {
          stack.push_back(previousNodeId);
        }
      }
    }

    if (!reachableFromSource.contains(targetId)) {
      return PathCollection{};
    }

    PathCollection result;
    for (auto const& [nodeId, hops] : candidate) {
      if (!reachableFromSource.contains(nodeId) || !canReachTarget.contains(nodeId)) {
        continue;
      }

      std::vector<Id> filteredHops;
      filteredHops.reserve(hops.size());
      for (auto const nextNodeId : hops) {
        if (reachableFromSource.contains(nextNodeId) &&
            canReachTarget.contains(nextNodeId) &&
            std::find(filteredHops.begin(), filteredHops.end(), nextNodeId) ==
                filteredHops.end()) {
          filteredHops.push_back(nextNodeId);
        }
      }

      if (!filteredHops.empty()) {
        result[nodeId] = std::move(filteredHops);
      }
    }

    return result;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  void Network<node_t, edge_t>::computeBetweennessCentralities() {
    for (auto& [nodeId, pNode] : m_nodes) {
      pNode->setAttribute("betweennessCentrality", 0.0);
    }

    auto const N_NODES = m_nodes.size();

    // Build an indexed list of source node ids so TBB can address them by integer.
    std::vector<Id> sourceIds;
    sourceIds.reserve(N_NODES);
    for (auto const& [id, _] : m_nodes) {
      sourceIds.push_back(id);
    }

    // Each TBB thread accumulates its own BC increments; we merge afterwards.
    tbb::combinable<std::unordered_map<Id, double>> localBC;

    tbb::parallel_for(std::size_t(0), sourceIds.size(), [&](std::size_t idx) {
      auto& local = localBC.local();
      auto const sourceId = sourceIds[idx];

      struct PathDataHelper {
        std::vector<Id> P;  // predecessor node ids on shortest paths
        double sigma{0.0};
        double dist{std::numeric_limits<double>::infinity()};
        double delta{0.0};
      };

      std::unordered_map<Id, PathDataHelper> pathData;
      pathData.reserve(N_NODES);
      for (auto const& [nId, _] : m_nodes) {
        pathData.emplace(nId, PathDataHelper{});
      }
      // Initialise source node data.
      {
        auto& src = pathData.at(sourceId);
        src.sigma = 1.0;
        src.dist = 0.0;
      }

      std::stack<Id> S;
      std::priority_queue<std::pair<double, Id>,
                          std::vector<std::pair<double, Id>>,
                          std::greater<>>
          pq;
      pq.push({0.0, sourceId});

      std::unordered_set<Id> visited;

      while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();

        if (visited.contains(v)) {
          continue;
        }
        visited.insert(v);
        S.push(v);

        auto const& vData = pathData.at(v);
        for (auto const& edgeId : m_nodes.at(v)->outgoingEdges()) {
          auto const& edgeObj = *m_edges.at(edgeId);
          auto const w = edgeObj.target();
          auto& wData = pathData.at(w);
          if (visited.contains(w)) {
            continue;
          }
          double const edgeWeight = m_weightFunction(edgeObj);
          double const newDist = vData.dist + edgeWeight;

          if (newDist < wData.dist) {
            wData.dist = newDist;
            wData.sigma = vData.sigma;
            wData.P = {v};
            pq.push({newDist, w});
          } else if (std::abs(newDist - wData.dist) < 1e-12 * std::max(1.0, wData.dist)) {
            wData.sigma += vData.sigma;
            wData.P.push_back(v);
          }
        }
      }

      // Back-propagate dependency scores.
      while (!S.empty()) {
        auto const w = S.top();
        auto& wData = pathData.at(w);
        S.pop();
        for (auto const v : wData.P) {
          auto& vData = pathData.at(v);
          vData.delta += (vData.sigma / wData.sigma) * (1.0 + wData.delta);
        }
        if (w != sourceId) {
          local[w] += wData.delta;
        }
      }
    });

    // Merge thread-local accumulations into node attributes (sequential).
    localBC.combine_each([&](auto const& localMap) {
      for (auto const& [nodeId, value] : localMap) {
        auto& nd = this->node(nodeId);
        auto const bc = nd.template getAttribute<double>("betweennessCentrality");
        nd.setAttribute("betweennessCentrality", *bc + value);
      }
    });
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Brandes edge betweenness  (parallelised over sources with TBB)
  // ─────────────────────────────────────────────────────────────────────────
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  void Network<node_t, edge_t>::computeEdgeBetweennessCentralities() {
    // Initialise all edge BC values to 0.
    for (auto& [edgeId, pEdge] : m_edges) {
      pEdge->setAttribute("betweennessCentrality", 0.0);
    }

    auto const N_NODES = m_nodes.size();

    std::vector<Id> sourceIds;
    sourceIds.reserve(N_NODES);
    for (auto const& [id, _] : m_nodes) {
      sourceIds.push_back(id);
    }

    // Each TBB thread accumulates its own edge BC increments.
    tbb::combinable<std::unordered_map<Id, double>> localBC;

    tbb::parallel_for(size_t(0), sourceIds.size(), [&](size_t idx) {
      auto& local = localBC.local();
      auto const sourceId = sourceIds[idx];

      struct PathDataHelper {
        std::vector<Id> P;  // predecessor edge ids on shortest paths
        double sigma{0.0};
        double dist{std::numeric_limits<double>::infinity()};
        double delta{0.0};
      };

      std::unordered_map<Id, PathDataHelper> pathData;
      pathData.reserve(N_NODES);
      for (auto const& [nId, _] : m_nodes) {
        pathData.emplace(nId, PathDataHelper{});
      }
      // Initialise source node data.
      {
        auto& src = pathData.at(sourceId);
        src.sigma = 1.0;
        src.dist = 0.0;
      }

      std::stack<Id> S;
      std::priority_queue<std::pair<double, Id>,
                          std::vector<std::pair<double, Id>>,
                          std::greater<>>
          pq;
      pq.push({0.0, sourceId});

      std::unordered_set<Id> visited;

      while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();

        if (visited.contains(v)) {
          continue;
        }
        visited.insert(v);
        S.push(v);

        auto const& vData = pathData.at(v);
        for (auto const& eId : m_nodes.at(v)->outgoingEdges()) {
          auto const& edgeObj = *m_edges.at(eId);
          auto const w = edgeObj.target();
          auto& wData = pathData.at(w);
          if (visited.contains(w)) {
            continue;
          }
          double const edgeWeight = m_weightFunction(edgeObj);
          double const newDist = vData.dist + edgeWeight;

          if (newDist < wData.dist) {
            wData.dist = newDist;
            wData.sigma = vData.sigma;
            wData.P = {eId};
            pq.push({newDist, w});
          } else if (std::abs(newDist - wData.dist) < 1e-12 * std::max(1.0, wData.dist)) {
            wData.sigma += vData.sigma;
            wData.P.push_back(eId);
          }
        }
      }

      // Back-propagate dependency scores; accumulate per-edge contributions.
      while (!S.empty()) {
        auto const w = S.top();
        auto& wData = pathData.at(w);
        S.pop();
        for (auto const eId : wData.P) {
          auto& vData = pathData.at(m_edges.at(eId)->source());
          double const contrib = (vData.sigma / wData.sigma) * (1.0 + wData.delta);
          vData.delta += contrib;
          local[eId] += contrib;
        }
      }
    });

    // Merge thread-local accumulations into edge attributes (sequential).
    localBC.combine_each([&](auto const& localMap) {
      for (auto const& [edgeId, value] : localMap) {
        auto& ed = this->edge(edgeId);
        auto const bc =
            ed.template getAttribute<double>("betweennessCentrality").value_or(0.0);
        ed.setAttribute("betweennessCentrality", bc + value);
      }
    });
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Yen K-shortest-paths edge betweenness  (parallelised over sources, TBB)
  // ─────────────────────────────────────────────────────────────────────────
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  void Network<node_t, edge_t>::computeEdgeKBetweennessCentralities(std::size_t const K) {
    for (auto& [eId, pEdge] : m_edges) {
      pEdge->setAttribute("betweennessCentrality", 0.0);
    }

    auto const N_NODES = m_nodes.size();
    if (N_NODES < 2 || K == 0) {
      return;
    }

    // K == 1 reduces exactly to standard Brandes edge betweenness.
    // K=1 delegates to the Brandes algorithm (computeEdgeBetweennessCentralities),
    // which accounts for tied shortest paths via σ fractions. Both K=1 and K>1
    // paths use the same (N-1)*(N-2) normalization, making K=1 equivalent to
    // single-source Brandes betweenness. Results are normalized by (N-1)*(N-2).
    // K>1 uses Yen's algorithm to find K shortest edge-disjoint paths and counts
    // edge contributions across all K paths.
    if (K == 1) {
      computeEdgeBetweennessCentralities();
      // Re-normalise: computeEdgeBetweennessCentralities leaves raw Brandes
      // values; apply the same norm as the K>1 path below.
      auto const norm = static_cast<double>((N_NODES - 1) * (N_NODES - 2));
      if (norm > 0.0) {
        for (auto& [_, pEdge] : m_edges) {
          auto const bc =
              pEdge->template getAttribute<double>("betweennessCentrality").value_or(0.0);
          pEdge->setAttribute("betweennessCentrality", bc / norm);
        }
      }
      return;
    }

    // K > 1: Yen's algorithm, parallelised over source nodes.
    std::vector<Id> nodeIds;
    nodeIds.reserve(N_NODES);
    for (auto const& [id, _] : m_nodes) {
      nodeIds.push_back(id);
    }

    std::unordered_map<Id, size_t> nodeIndex;
    nodeIndex.reserve(N_NODES);
    for (std::size_t i = 0; i < N_NODES; ++i) {
      nodeIndex.emplace(nodeIds[i], i);
    }

    tbb::combinable<std::unordered_map<Id, double>> localAccum;

    tbb::parallel_for(std::size_t(0), N_NODES, [&](std::size_t const sourceIndex) {
      auto& local = localAccum.local();

      struct PathData {
        std::vector<Id> edges;
        double cost{0.0};
      };

      for (std::size_t targetIndex = 0; targetIndex < N_NODES; ++targetIndex) {
        if (sourceIndex == targetIndex) {
          continue;
        }

        std::vector<PathData> acceptedPaths;

        // ── Global candidate set for Yen's algorithm across all ranks ─────
        // Standard Yen's algorithm maintains a single priority queue of candidate
        // paths across all K rank iterations. Candidates generated but not selected
        // in earlier ranks remain available for later ranks (B-heap approach).
        std::priority_queue<std::pair<double, std::vector<Id>>,
                            std::vector<std::pair<double, std::vector<Id>>>,
                            std::greater<>>
            globalCandidates;

        // ── Initial shortest path (Dijkstra) ──────────────────────────────
        std::vector<double> dist(N_NODES, std::numeric_limits<double>::infinity());
        std::vector<Id> parentEdge(N_NODES, Id{});
        std::priority_queue<std::pair<double, std::size_t>,
                            std::vector<std::pair<double, std::size_t>>,
                            std::greater<>>
            pq;
        dist[sourceIndex] = 0.0;
        pq.push({0.0, sourceIndex});

        while (!pq.empty()) {
          auto const [currentDist, currentIndex] = pq.top();
          pq.pop();
          if (currentDist > dist[currentIndex]) {
            continue;
          }
          if (currentIndex == targetIndex) {
            break;
          }

          auto const currentNodeId = nodeIds[currentIndex];
          for (auto const edgeId : m_nodes.at(currentNodeId)->outgoingEdges()) {
            auto const& edgeObj = this->edge(edgeId);
            auto const nextIdx = nodeIndex.at(edgeObj.target());
            auto const nd = currentDist + m_weightFunction(edgeObj);
            if (nd < dist[nextIdx]) {
              dist[nextIdx] = nd;
              parentEdge[nextIdx] = edgeId;
              pq.push({nd, nextIdx});
            }
          }
        }

        if (!std::isfinite(dist[targetIndex])) {
          continue;
        }

        // Reconstruct first shortest path.
        PathData firstPath;
        firstPath.cost = dist[targetIndex];
        for (size_t cursor = targetIndex; cursor != sourceIndex;) {
          Id const eId = parentEdge[cursor];
          firstPath.edges.push_back(eId);
          cursor = nodeIndex.at(m_edges.at(eId)->source());
        }
        std::reverse(firstPath.edges.begin(), firstPath.edges.end());
        acceptedPaths.push_back(std::move(firstPath));

        // ── Yen's spur loop ───────────────────────────────────────────────
        for (size_t pathRank = 1; pathRank < K; ++pathRank) {
          auto const& lastPath = acceptedPaths.back().edges;

          for (size_t prefixSize = 0; prefixSize < lastPath.size(); ++prefixSize) {
            std::vector<Id> rootPath(
                lastPath.begin(),
                lastPath.begin() + static_cast<std::ptrdiff_t>(prefixSize));

            // Determine spur node.
            Id spurNodeId = nodeIds[sourceIndex];
            for (auto const eId : rootPath) {
              spurNodeId = m_edges.at(eId)->target();
            }
            size_t const spurIndex = nodeIndex.at(spurNodeId);

            // Build banned-node set: all root-path nodes except the spur node.
            std::unordered_set<Id> bannedNodes;
            Id visitedNodeId = nodeIds[sourceIndex];
            for (auto const eId : rootPath) {
              Id const srcNodeId = visitedNodeId;
              visitedNodeId = m_edges.at(eId)->target();
              if (srcNodeId != spurNodeId) {
                bannedNodes.insert(srcNodeId);
              }
            }

            // Ban the outgoing edge at the spur position for every accepted
            // path that shares the same root prefix.
            std::unordered_set<Id> bannedEdges;
            for (auto const& path : acceptedPaths) {
              if (path.edges.size() <= prefixSize) {
                continue;
              }
              bool samePrefix = true;
              for (size_t i = 0; i < prefixSize; ++i) {
                if (path.edges[i] != rootPath[i]) {
                  samePrefix = false;
                  break;
                }
              }
              if (samePrefix) {
                bannedEdges.insert(path.edges[prefixSize]);
              }
            }

            if (bannedNodes.contains(spurNodeId)) {
              continue;
            }

            // Dijkstra from the spur node with banned nodes/edges.
            std::vector<double> spurDist(N_NODES,
                                         std::numeric_limits<double>::infinity());
            std::vector<Id> spurParentEdge(N_NODES, Id{});
            std::priority_queue<std::pair<double, std::size_t>,
                                std::vector<std::pair<double, std::size_t>>,
                                std::greater<>>
                spurQueue;
            spurDist[spurIndex] = 0.0;
            spurQueue.push({0.0, spurIndex});

            while (!spurQueue.empty()) {
              auto const [currentDist, currentIndex] = spurQueue.top();
              spurQueue.pop();
              if (currentDist > spurDist[currentIndex]) {
                continue;
              }
              if (currentIndex == targetIndex) {
                break;
              }

              auto const curNodeId = nodeIds[currentIndex];
              if (bannedNodes.contains(curNodeId)) {
                continue;
              }

              for (auto const eId : m_nodes.at(curNodeId)->outgoingEdges()) {
                if (bannedEdges.contains(eId)) {
                  continue;
                }
                auto const& edgeObj = *m_edges.at(eId);
                Id const nextNode = edgeObj.target();
                if (bannedNodes.contains(nextNode)) {
                  continue;
                }
                auto const nextIdx = nodeIndex.at(nextNode);
                auto const nd = currentDist + m_weightFunction(edgeObj);
                if (nd < spurDist[nextIdx]) {
                  spurDist[nextIdx] = nd;
                  spurParentEdge[nextIdx] = eId;
                  spurQueue.push({nd, nextIdx});
                }
              }
            }

            if (!std::isfinite(spurDist[targetIndex])) {
              continue;
            }

            // Build candidate = rootPath + spurPath.
            PathData candidatePath;
            candidatePath.edges = rootPath;
            candidatePath.cost = 0.0;
            for (auto const eId : rootPath) {
              candidatePath.cost += m_weightFunction(*m_edges.at(eId));
            }

            std::vector<Id> spurPath;
            for (std::size_t cursor = targetIndex; cursor != spurIndex;) {
              Id const eId = spurParentEdge[cursor];
              spurPath.push_back(eId);
              cursor = nodeIndex.at(m_edges.at(eId)->source());
            }
            std::reverse(spurPath.begin(), spurPath.end());

            candidatePath.cost += spurDist[targetIndex];
            candidatePath.edges.insert(
                candidatePath.edges.end(), spurPath.begin(), spurPath.end());

            bool duplicate = false;
            for (auto const& path : acceptedPaths) {
              if (path.edges == candidatePath.edges) {
                duplicate = true;
                break;
              }
            }
            if (!duplicate) {
              globalCandidates.push({candidatePath.cost, std::move(candidatePath.edges)});
            }
          }  // end prefix loop

          bool foundPath = false;
          while (!globalCandidates.empty() && !foundPath) {
            auto [candidateCost, candidateEdges] = globalCandidates.top();
            globalCandidates.pop();

            bool alreadyAccepted = false;
            for (auto const& path : acceptedPaths) {
              if (path.edges == candidateEdges) {
                alreadyAccepted = true;
                break;
              }
            }

            if (!alreadyAccepted) {
              acceptedPaths.push_back(PathData{std::move(candidateEdges), candidateCost});
              foundPath = true;
            }
          }

          if (!foundPath && globalCandidates.empty()) {
            break;  // No more candidates; K-path generation complete
          }
        }  // end Yen rank loop

        // Accumulate edge counts.
        for (auto const& path : acceptedPaths) {
          for (auto const eId : path.edges) {
            local[eId] += 1.0;
          }
        }
      }  // end target loop
    });  // end parallel_for

    // Merge thread-local accumulations into edge attributes (sequential).
    localAccum.combine_each([&](auto const& localMap) {
      for (auto const& [edgeId, value] : localMap) {
        auto& ed = this->edge(edgeId);
        auto const bc =
            ed.template getAttribute<double>("betweennessCentrality").value_or(0.0);
        ed.setAttribute("betweennessCentrality", bc + value);
      }
    });

    // Normalise by (N-1)·(N-2).
    auto const norm = static_cast<double>((N_NODES - 1) * (N_NODES - 2));
    if (norm > 0.0) {
      for (auto& [_, pEdge] : m_edges) {
        auto const bc =
            pEdge->template getAttribute<double>("betweennessCentrality").value_or(0.0);
        pEdge->setAttribute("betweennessCentrality", bc / norm);
      }
    }
  }

}  // namespace dsf