#pragma once

#include "Edge.hpp"
#include "Node.hpp"
#include "../utility/PtrMapView.hpp"
#include "../utility/Typedef.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <ranges>
#include <span>
#include <set>
#include <stack>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <spdlog/spdlog.h>
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

    static constexpr auto m_cantorHash(Id u, Id v) noexcept {
      return ((u + v) * (u + v + 1)) / 2 + v;
    }
    static constexpr auto m_cantorHash(std::pair<Id, Id> const& idPair) noexcept {
      return m_cantorHash(idPair.first, idPair.second);
    }
    /// @brief Find the edge going from source to target
    /// @details Only the source node's outgoing edges are scanned, instead of all edges
    /// @return edge_t* A pointer to the edge, or nullptr if it does not exist
    edge_t* m_findEdge(Id const source, Id const target) const;
    // Typed access to the links between nodes and edges: the network only links its own
    // node_t and edge_t objects, so the casts are safe.
    static auto m_inEdges(node_t const& node) noexcept {
      return node.Node::ingoingEdges() | std::views::transform([](Edge* pEdge) {
               return static_cast<edge_t*>(pEdge);
             });
    }
    static auto m_outEdges(node_t const& node) noexcept {
      return node.Node::outgoingEdges() | std::views::transform([](Edge* pEdge) {
               return static_cast<edge_t*>(pEdge);
             });
    }
    static node_t* m_targetOf(edge_t const& edge) noexcept {
      return static_cast<node_t*>(edge.Edge::target());
    }
    /// @brief Replace a node with a new object of type TNode, built from the old one
    /// @details The new node keeps the old node's edges, whose source/target node
    ///          pointers are re-linked to it. Any other reference to the old node dangles.
    ///          Safe to call concurrently for different nodes: for an edge u -> v, u only
    ///          writes the edge's source node and v only its target node.
    /// @param pNode The network's owning pointer to the node to replace
    /// @param args Extra arguments forwarded to TNode's constructor, after the old node
    /// @return TNode& A reference to the new node
    template <typename TNode, typename... TArgs>
      requires(std::is_base_of_v<node_t, TNode> &&
               std::constructible_from<TNode, node_t const&, TArgs...>)
    TNode& m_replaceNode(std::unique_ptr<node_t>& pNode, TArgs&&... args) {
      auto pNewNode =
          std::make_unique<TNode>(std::as_const(*pNode), std::forward<TArgs>(args)...);
      for (auto* pEdge : pNewNode->Node::ingoingEdges()) {
        pEdge->setTarget(pNewNode.get());
      }
      for (auto* pEdge : pNewNode->Node::outgoingEdges()) {
        pEdge->setSource(pNewNode.get());
      }
      auto& newNode{*pNewNode};
      pNode = std::move(pNewNode);
      return newNode;
    }

  public:
    /// @brief An edge cost function
    using EdgeWeight = std::function<double(edge_t const&)>;

    /// @brief Construct a new empty Network object
    Network() = default;

    /// @brief Get the nodes as a view of (id, pointer to const node) pairs
    inline auto nodes() const noexcept { return PtrMapView<node_t const>{m_nodes}; }
    /// @brief Get the nodes as a view of (id, pointer to node) pairs
    inline auto nodes() noexcept { return PtrMapView<node_t>{m_nodes}; }
    /// @brief Get the edges as a view of (id, pointer to const edge) pairs
    inline auto edges() const noexcept { return PtrMapView<edge_t const>{m_edges}; }
    /// @brief Get the edges as a view of (id, pointer to edge) pairs
    inline auto edges() noexcept { return PtrMapView<edge_t>{m_edges}; }
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

    /// @brief Add an edge between two nodes, creating the missing ones
    /// @tparam TEdge The type of the edge
    /// @tparam TNode The type of the nodes created for the edge
    /// @param edgeId The edge's id
    /// @param nodeIds The ids of the edge's (source, target) nodes
    /// @param args Extra arguments forwarded to TEdge's constructor, after the nodes
    /// @return TEdge& A reference to the new edge
    /// @throws std::invalid_argument if an edge with the same id already exists
    template <typename TEdge = edge_t, typename TNode = node_t, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> && std::is_base_of_v<node_t, TNode> &&
               std::constructible_from<TEdge, Id, std::pair<Node*, Node*>, TArgs...>)
    TEdge& addEdge(Id const edgeId, std::pair<Id, Id> const nodeIds, TArgs&&... args);

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

    /// @brief Compute node weighted betweenness centralities using Brandes' algorithm
    /// @param weight The cost of each edge
    void computeBetweennessCentralities(EdgeWeight const& weight);

    /// @brief Compute edge weighted betweenness centralities using Brandes' algorithm
    /// @param weight The cost of each edge
    void computeEdgeBetweennessCentralities(EdgeWeight const& weight);

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
    /// @param  weight         The cost of each edge.
    /// @param  K              Maximum number of shortest paths per (s, t) pair.
    ///                        K = 1 reproduces single-shortest-path betweenness.
    void computeEdgeKBetweennessCentralities(EdgeWeight const& weight,
                                             std::size_t const K);
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
  template <typename TEdge, typename TNode, typename... TArgs>
    requires(std::is_base_of_v<edge_t, TEdge> && std::is_base_of_v<node_t, TNode> &&
             std::constructible_from<TEdge, Id, std::pair<Node*, Node*>, TArgs...>)
  TEdge& Network<node_t, edge_t>::addEdge(Id const edgeId,
                                          std::pair<Id, Id> const nodeIds,
                                          TArgs&&... args) {
    if (m_edges.contains(edgeId)) {
      throw std::invalid_argument(
          std::format("Edge with id {} already exists in the network.", edgeId));
    }
    auto const [sourceNodeId, targetNodeId] = nodeIds;
    bool const bNewSource{!m_nodes.contains(sourceNodeId)};
    if (bNewSource) {
      addNode<TNode>(sourceNodeId);
    }
    bool const bNewTarget{!m_nodes.contains(targetNodeId)};
    if (bNewTarget) {
      addNode<TNode>(targetNodeId);
    }
    auto& sourceNode = node(sourceNodeId);
    auto& targetNode = node(targetNodeId);

    std::unique_ptr<TEdge> pEdge;
    try {
      pEdge = std::make_unique<TEdge>(edgeId,
                                      std::pair<Node*, Node*>{&sourceNode, &targetNode},
                                      std::forward<TArgs>(args)...);
    } catch (...) {
      // Leave the network unchanged
      if (bNewSource) {
        m_nodes.erase(sourceNodeId);
      }
      if (bNewTarget) {
        m_nodes.erase(targetNodeId);
      }
      throw;
    }
    auto const& geometry{pEdge->geometry()};
    if (!geometry.empty()) {
      // Nodes created for this edge are placed at its ends
      if (bNewSource) {
        sourceNode.setGeometry(geometry.front());
      }
      if (bNewTarget) {
        targetNode.setGeometry(geometry.back());
      }
    } else if (sourceNode.geometry().has_value() && targetNode.geometry().has_value()) {
      pEdge->setGeometry(
          dsf::geometry::PolyLine{*sourceNode.geometry(), *targetNode.geometry()});
    }
    auto& newEdge{*pEdge};
    m_edges.emplace(edgeId, std::move(pEdge));
    sourceNode.addOutgoingEdge(&newEdge);
    targetNode.addIngoingEdge(&newEdge);
    // Link the reverse edge (v -> u), if any
    if (auto* pReverse{m_findEdge(targetNodeId, sourceNodeId)};
        pReverse != nullptr && pReverse != &newEdge && pReverse->reverse() == nullptr) {
      newEdge.setReverse(pReverse);
      pReverse->setReverse(&newEdge);
    }
    return newEdge;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  edge_t* Network<node_t, edge_t>::m_findEdge(Id const source, Id const target) const {
    auto const itNode = m_nodes.find(source);
    auto const itTarget = m_nodes.find(target);
    if (itNode == m_nodes.cend() || itTarget == m_nodes.cend()) {
      return nullptr;
    }
    for (auto* pEdge : m_outEdges(*itNode->second)) {
      if (m_targetOf(*pEdge) == itTarget->second.get()) {
        return pEdge;
      }
    }
    return nullptr;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  edge_t& Network<node_t, edge_t>::edge(Id const source, Id const target) const {
    auto* pEdge{m_findEdge(source, target)};
    if (pEdge == nullptr) {
      throw std::out_of_range(
          std::format("Edge with source {} and target {} not found.", source, target));
    }
    return *pEdge;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  void Network<node_t, edge_t>::computeBetweennessCentralities(EdgeWeight const& weight) {
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
        for (auto const* pEdgeObj : m_outEdges(*m_nodes.at(v))) {
          auto const& edgeObj{*pEdgeObj};
          if (!edgeObj.isActive()) {
            continue;
          }
          auto const w = edgeObj.target()->id();
          auto& wData = pathData.at(w);
          if (visited.contains(w)) {
            continue;
          }
          double const edgeWeight = weight(edgeObj);
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
  void Network<node_t, edge_t>::computeEdgeBetweennessCentralities(
      EdgeWeight const& weight) {
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
        for (auto const* pEdgeObj : m_outEdges(*m_nodes.at(v))) {
          auto const& edgeObj{*pEdgeObj};
          auto const eId{edgeObj.id()};
          if (!edgeObj.isActive()) {
            continue;
          }
          auto const w = edgeObj.target()->id();
          auto& wData = pathData.at(w);
          if (visited.contains(w)) {
            continue;
          }
          double const edgeWeight = weight(edgeObj);
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
          auto& vData = pathData.at(m_edges.at(eId)->source()->id());
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
  void Network<node_t, edge_t>::computeEdgeKBetweennessCentralities(
      EdgeWeight const& weight, std::size_t const K) {
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
    // K>1 uses Yen's algorithm to find K shortest loopless paths and counts
    // edge contributions across all K paths.
    if (K == 1) {
      computeEdgeBetweennessCentralities(weight);
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
          for (auto const* pEdgeObj : m_outEdges(*m_nodes.at(currentNodeId))) {
            auto const& edgeObj{*pEdgeObj};
            auto const edgeId{edgeObj.id()};
            if (!edgeObj.isActive()) {
              continue;
            }
            auto const nextIdx = nodeIndex.at(edgeObj.target()->id());
            auto const nd = currentDist + weight(edgeObj);
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
          cursor = nodeIndex.at(m_edges.at(eId)->source()->id());
        }
        std::ranges::reverse(firstPath.edges);
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
              spurNodeId = m_edges.at(eId)->target()->id();
            }
            size_t const spurIndex = nodeIndex.at(spurNodeId);

            // Build banned-node set: all root-path nodes except the spur node.
            std::unordered_set<Id> bannedNodes;
            Id visitedNodeId = nodeIds[sourceIndex];
            for (auto const eId : rootPath) {
              Id const srcNodeId = visitedNodeId;
              visitedNodeId = m_edges.at(eId)->target()->id();
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

              for (auto const* pEdgeObj : m_outEdges(*m_nodes.at(curNodeId))) {
                auto const eId{pEdgeObj->id()};
                if (bannedEdges.contains(eId)) {
                  continue;
                }
                auto const& edgeObj{*pEdgeObj};
                if (!edgeObj.isActive()) {
                  continue;
                }
                Id const nextNode = edgeObj.target()->id();
                if (bannedNodes.contains(nextNode)) {
                  continue;
                }
                auto const nextIdx = nodeIndex.at(nextNode);
                auto const nd = currentDist + weight(edgeObj);
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
              candidatePath.cost += weight(*m_edges.at(eId));
            }

            std::vector<Id> spurPath;
            for (std::size_t cursor = targetIndex; cursor != spurIndex;) {
              Id const eId = spurParentEdge[cursor];
              spurPath.push_back(eId);
              cursor = nodeIndex.at(m_edges.at(eId)->source()->id());
            }
            std::ranges::reverse(spurPath);

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