#pragma once

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <stack>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/combinable.h>
#include <tbb/parallel_for.h>

#include <spdlog/spdlog.h>

#include "Edge.hpp"
#include "Node.hpp"

namespace dsf {
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  class Network {
  protected:
    std::unordered_map<Id, std::unique_ptr<node_t>> m_nodes;
    std::unordered_map<Id, std::unique_ptr<edge_t>> m_edges;

    constexpr inline auto m_cantorHash(Id u, Id v) const {
      return ((u + v) * (u + v + 1)) / 2 + v;
    }
    constexpr inline auto m_cantorHash(std::pair<Id, Id> const& idPair) const {
      return m_cantorHash(idPair.first, idPair.second);
    }

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
    void addNDefaultNodes(size_t n);

    template <typename TEdge = edge_t, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> &&
               std::constructible_from<TEdge, TArgs...>)
    void addEdge(TArgs&&... args);

    inline const auto& node(Id nodeId) const { return *m_nodes.at(nodeId); };
    inline auto& node(Id nodeId) { return *m_nodes.at(nodeId); };
    inline const auto& edge(Id edgeId) const { return *m_edges.at(edgeId); };
    inline auto& edge(Id edgeId) { return *m_edges.at(edgeId); }

    edge_t& edge(Id source, Id target) const;

    template <typename TNode>
      requires(std::is_base_of_v<node_t, TNode>)
    inline auto& node(Id nodeId) {
      return dynamic_cast<TNode&>(node(nodeId));
    }

    template <typename TEdge>
      requires(std::is_base_of_v<edge_t, TEdge>)
    inline auto& edge(Id edgeId) {
      return dynamic_cast<TEdge&>(edge(edgeId));
    }

    /// @brief Compute node betweenness centralities using Brandes' algorithm
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeBetweennessCentralities(WeightFunc getEdgeWeight);

    /// @brief Compute edge betweenness centralities using Brandes' algorithm
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeEdgeBetweennessCentralities(WeightFunc getEdgeWeight);

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
    /// @tparam WeightFunc  A callable (const edge_t&) → double.  Must return a
    ///                     strictly-positive value for every edge.
    /// @param  getEdgeWeight  Edge-weight extractor.
    /// @param  K              Maximum number of shortest paths per (s, t) pair.
    ///                        K = 1 reproduces single-shortest-path betweenness.
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeEdgeKBetweennessCentralities(WeightFunc getEdgeWeight, size_t K);
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
  void Network<node_t, edge_t>::addNDefaultNodes(size_t n) {
    auto const currentSize{m_nodes.size()};
    for (size_t i = 0; i < n; ++i) {
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
  edge_t& Network<node_t, edge_t>::edge(Id source, Id target) const {
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
  template <typename TNode>
    requires(std::is_base_of_v<node_t, TNode>)
  TNode& Network<node_t, edge_t>::node(Id nodeId) {
    return dynamic_cast<TNode&>(node(nodeId));
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge>
    requires(std::is_base_of_v<edge_t, TEdge>)
  TEdge& Network<node_t, edge_t>::edge(Id edgeId) {
    return dynamic_cast<TEdge&>(edge(edgeId));
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Brandes node betweenness
  // ──────────────────────────────────────────────────────────────────────────

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename WeightFunc>
    requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
  void Network<node_t, edge_t>::computeBetweennessCentralities(WeightFunc getEdgeWeight) {
    for (auto& [nodeId, pNode] : m_nodes) {
      pNode->setAttribute("betweennessCentrality", 0.0);
    }

    struct PathDataHelper {
      std::vector<Id> P;
      double sigma{0.0};
      double dist{std::numeric_limits<double>::infinity()};
      double delta{0.0};
    };

    for (auto const& [sourceId, sourceNode] : m_nodes) {
      std::stack<Id> S;
      std::unordered_map<Id, PathDataHelper> pathData;
      pathData.reserve(this->nNodes());

      for (auto const& [nId, _] : m_nodes) {
        pathData.emplace(nId, PathDataHelper());
      }
      {
        auto& sourceData = pathData[sourceId];
        sourceData.sigma = 1.0;
        sourceData.dist = 0.0;
      }

      std::priority_queue<std::pair<double, Id>,
                          std::vector<std::pair<double, Id>>,
                          std::greater<>>
          pq;
      pq.push({0.0, sourceId});

      std::unordered_set<Id> visited;

      while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();

        if (visited.contains(v))
          continue;
        visited.insert(v);
        S.push(v);
        auto& vData = pathData[v];

        for (auto const& edgeId : m_nodes.at(v)->outgoingEdges()) {
          auto const& edgeObj = *m_edges.at(edgeId);
          Id w = edgeObj.target();
          auto& wData = pathData[w];
          if (visited.contains(w))
            continue;
          double edgeWeight = getEdgeWeight(edgeObj);
          double newDist = vData.dist + edgeWeight;

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

      while (!S.empty()) {
        Id w = S.top();
        auto const& wData = pathData[w];
        S.pop();
        for (auto const v : wData.P) {
          auto& vData = pathData[v];
          vData.delta += (vData.sigma / wData.sigma) * (1.0 + wData.delta);
        }
        if (w != sourceId) {
          auto& currentNode = this->node(w);
          auto const optBC =
              currentNode.template getAttribute<double>("betweennessCentrality");
          // BC must exist since we initialized it for all nodes at the start
          currentNode.setAttribute("betweennessCentrality", *optBC + wData.delta);
        }
      }
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Brandes edge betweenness
  // ──────────────────────────────────────────────────────────────────────────

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename WeightFunc>
    requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
  void Network<node_t, edge_t>::computeEdgeBetweennessCentralities(
      WeightFunc getEdgeWeight) {
    for (auto& [edgeId, pEdge] : m_edges) {
      pEdge->setAttribute("betweennessCentrality", 0.0);
    }

    struct PathDataHelper {
      std::vector<Id> P;  // predecessor edge ids on shortest paths
      double sigma{0.0};
      double dist{std::numeric_limits<double>::infinity()};
      double delta{0.0};
    };

    for (auto const& [sourceId, sourceNode] : m_nodes) {
      std::stack<Id> S;
      std::unordered_map<Id, PathDataHelper> pathData;
      pathData.reserve(this->nNodes());

      for (auto const& [nId, _] : m_nodes) {
        pathData.emplace(nId, PathDataHelper());
      }
      {
        auto& sourceData = pathData.at(sourceId);
        sourceData.sigma = 1.0;
        sourceData.dist = 0.0;
      }

      std::priority_queue<std::pair<double, Id>,
                          std::vector<std::pair<double, Id>>,
                          std::greater<>>
          pq;
      pq.push({0.0, sourceId});

      std::unordered_set<Id> visited;

      while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();

        if (visited.contains(v))
          continue;
        visited.insert(v);
        S.push(v);

        auto& vData = pathData.at(v);
        for (auto const& eId : this->node(v).outgoingEdges()) {
          auto const& edgeObj = this->edge(eId);
          Id w = edgeObj.target();
          auto& wData = pathData.at(w);
          if (visited.contains(w))
            continue;
          double edgeWeight = getEdgeWeight(edgeObj);
          double newDist = vData.dist + edgeWeight;

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

      while (!S.empty()) {
        Id w = S.top();
        auto& wData = pathData.at(w);
        S.pop();
        for (auto const eId : wData.P) {
          auto& vData = pathData.at(this->edge(eId).source());
          double contrib = (vData.sigma / wData.sigma) * (1.0 + wData.delta);
          vData.delta += contrib;
          auto& currentEdge = this->edge(eId);
          auto const optBC =
              currentEdge.template getAttribute<double>("betweennessCentrality");
          // BC must exist since we initialized it for all edges at the start
          currentEdge.setAttribute("betweennessCentrality", *optBC + contrib);
        }
      }
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Yen K-shortest-paths edge betweenness  (parallel, TBB)
  // ──────────────────────────────────────────────────────────────────────────

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename WeightFunc>
    requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
  void Network<node_t, edge_t>::computeEdgeKBetweennessCentralities(
      WeightFunc getEdgeWeight, size_t K) {
    // ── 0. Trivial cases ──────────────────────────────────────────────────
    for (auto& [eId, pEdge] : m_edges)
      pEdge->setAttribute("betweennessCentrality", 0.0);

    size_t const N = m_nodes.size();
    if (N < 2 || K == 0)
      return;

    // Snapshot all node IDs so parallel threads can index them safely.
    std::vector<Id> nodeIds;
    nodeIds.reserve(N);
    for (auto const& [id, _] : m_nodes)
      nodeIds.push_back(id);

    auto const progressStart = std::chrono::steady_clock::now();
    constexpr auto progressLogInterval = std::chrono::seconds(5);
    constexpr auto progressLogIntervalMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(progressLogInterval).count();
    std::atomic<size_t> processedSources{0};
    std::atomic<long long> nextProgressLogMs{progressLogIntervalMs};

    // ── 1. Dijkstra helper ────────────────────────────────────────────────
    //
    // Returns the sequence of edge IDs that form the shortest path from
    // `src` to `tgt` in the graph with the supplied forbidden edges and
    // nodes removed.  Returns std::nullopt when no path exists.
    //
    // forbidden_nodes excludes a node from relaxation but never blocks src
    // or tgt themselves (caller is responsible for not passing those in).
    auto dijkstra = [&](Id src,
                        Id tgt,
                        std::unordered_set<Id> const& forbiddenEdges,
                        std::unordered_set<Id> const& forbiddenNodes)
        -> std::optional<std::pair<std::vector<Id>, double>>  // (edgePath, cost)
    {
      using PQEntry = std::pair<double, Id>;

      std::unordered_map<Id, double> dist;
      // prev[w] = { predecessor node, edge used to reach w }
      std::unordered_map<Id, std::pair<Id, Id>> prev;

      dist.reserve(N);
      for (auto const& [nId, _] : m_nodes)
        dist[nId] = std::numeric_limits<double>::infinity();
      dist[src] = 0.0;

      std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
      pq.push({0.0, src});
      std::unordered_set<Id> visited;

      while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();

        if (visited.contains(v))
          continue;
        // A forbidden node can be the start of relaxation only if it is src.
        // It must never be settled as an intermediate or final step (unless tgt).
        if (v != src && v != tgt && forbiddenNodes.contains(v))
          continue;
        visited.insert(v);
        if (v == tgt)
          break;

        for (auto const& eId : m_nodes.at(v)->outgoingEdges()) {
          if (forbiddenEdges.contains(eId))
            continue;
          auto const& edgeObj = *m_edges.at(eId);
          Id w = edgeObj.target();
          // Respect forbidden nodes for non-target neighbours.
          if (w != tgt && forbiddenNodes.contains(w))
            continue;
          double nd = d + getEdgeWeight(edgeObj);
          if (nd < dist[w]) {
            dist[w] = nd;
            prev[w] = {v, eId};
            pq.push({nd, w});
          }
        }
      }

      if (!std::isfinite(dist[tgt]))
        return std::nullopt;

      // Reconstruct edge sequence from tgt back to src.
      std::vector<Id> edgePath;
      for (Id v = tgt; v != src;) {
        auto const& [u, eId] = prev.at(v);
        edgePath.push_back(eId);
        v = u;
      }
      std::reverse(edgePath.begin(), edgePath.end());
      return std::make_pair(std::move(edgePath), dist[tgt]);
    };

    // ── 2. Yen's K-shortest paths ─────────────────────────────────────────
    //
    // Returns up to K loopless shortest paths between src and tgt as
    // sequences of edge IDs.  Paths are ordered by non-decreasing cost.
    auto yenKShortest = [&](Id src, Id tgt) -> std::vector<std::vector<Id>> {
      if (src == tgt)
        return {};

      // A[k] = (edgePath, cost) of the k-th shortest path confirmed so far.
      std::vector<std::pair<std::vector<Id>, double>> A;

      // Candidate heap: (cost, edgePath).  std::greater makes it a min-heap.
      using Candidate = std::pair<double, std::vector<Id>>;
      std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> B;

      // Deduplication set so the same path isn't enqueued twice.
      std::set<std::vector<Id>> seen;

      // ── Find the first (shortest) path ─────────────────────────────────
      auto first = dijkstra(src, tgt, {}, {});
      if (!first)
        return {};
      A.push_back(std::move(*first));

      // ── Iterate to find paths 2 … K ────────────────────────────────────
      for (size_t k = 1; k < K; ++k) {
        auto const& [prevEdges, prevCost] = A[k - 1];

        // Reconstruct the node sequence of the (k-1)-th path.
        // nodeSeq[0] = src, nodeSeq[i+1] = target of prevEdges[i].
        std::vector<Id> nodeSeq;
        nodeSeq.reserve(prevEdges.size() + 1);
        nodeSeq.push_back(src);
        for (auto const& eId : prevEdges)
          nodeSeq.push_back(m_edges.at(eId)->target());

        // ── Spur-node loop ────────────────────────────────────────────────
        // spurIdx runs over every node in the path except the last (target).
        for (size_t spurIdx = 0; spurIdx + 1 < nodeSeq.size(); ++spurIdx) {
          Id const spurNode = nodeSeq[spurIdx];

          // root edges = prefix of prevEdges up to (but not including) spurIdx.
          std::vector<Id> rootEdges(
              prevEdges.begin(),
              prevEdges.begin() + static_cast<std::ptrdiff_t>(spurIdx));
          double rootCost = 0.0;
          for (auto const& eId : rootEdges)
            rootCost += getEdgeWeight(*m_edges.at(eId));

          // Build the set of edges to suppress at position spurIdx:
          // Any path already in A that shares the same root prefix must have
          // its spurIdx-th edge removed so the new spur diverges from it.
          std::unordered_set<Id> forbiddenEdges;
          for (auto const& [aEdges, _] : A) {
            if (aEdges.size() <= spurIdx)
              continue;
            // Check whether aEdges[0..spurIdx-1] == rootEdges[0..spurIdx-1].
            bool rootMatch = true;
            for (size_t r = 0; r < spurIdx; ++r) {
              if (aEdges[r] != rootEdges[r]) {
                rootMatch = false;
                break;
              }
            }
            if (rootMatch)
              forbiddenEdges.insert(aEdges[spurIdx]);
          }

          // Root-path nodes (indices 0 … spurIdx-1) are suppressed to prevent
          // the spur path from looping back through the root.
          std::unordered_set<Id> forbiddenNodes;
          for (size_t r = 0; r < spurIdx; ++r)
            forbiddenNodes.insert(nodeSeq[r]);

          // Run Dijkstra for the spur portion: spurNode → tgt.
          auto spurResult = dijkstra(spurNode, tgt, forbiddenEdges, forbiddenNodes);
          if (!spurResult)
            continue;

          auto& [spurEdges, spurCost] = *spurResult;

          // Total path = rootEdges ++ spurEdges.
          std::vector<Id> totalEdges = rootEdges;
          totalEdges.insert(totalEdges.end(), spurEdges.begin(), spurEdges.end());
          double totalCost = rootCost + spurCost;

          // Enqueue only if not already seen.
          if (!seen.contains(totalEdges)) {
            seen.insert(totalEdges);
            B.push({totalCost, std::move(totalEdges)});
          }
        }

        if (B.empty())
          break;

        // Accept the cheapest candidate as the k-th shortest path.
        auto [bestCost, bestEdges] = std::move(const_cast<Candidate&>(B.top()));
        B.pop();
        seen.erase(bestEdges);  // no longer needed in the dedup set
        A.push_back({std::move(bestEdges), bestCost});
      }

      // Return edge-path sequences only (costs not needed by the caller).
      std::vector<std::vector<Id>> result;
      result.reserve(A.size());
      for (auto& [ep, _] : A)
        result.push_back(std::move(ep));
      return result;
    };

    // ── 3. Parallel accumulation (TBB) ───────────────────────────────────
    //
    // Each thread keeps its own edgeId → count map to avoid synchronisation
    // on hot paths.  Maps are combined serially after the parallel region.
    tbb::combinable<std::unordered_map<Id, double>> localAccum;

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, N), [&](tbb::blocked_range<size_t> const& range) {
          auto& localMap = localAccum.local();
          for (size_t i = range.begin(); i != range.end(); ++i) {
            Id const srcId = nodeIds[i];
            for (size_t j = 0; j < N; ++j) {
              if (i == j)
                continue;
              Id const tgtId = nodeIds[j];

              auto paths = yenKShortest(srcId, tgtId);
              for (auto const& edgePath : paths) {
                for (Id const eId : edgePath) {
                  localMap[eId] += 1.0;
                }
              }
            }

            auto const doneSources =
                processedSources.fetch_add(1, std::memory_order_relaxed) + 1;
            if (spdlog::should_log(spdlog::level::info)) {
              auto const elapsedMs =
                  std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - progressStart)
                      .count();
              auto dueMs = nextProgressLogMs.load(std::memory_order_relaxed);
              while (elapsedMs >= dueMs) {
                if (nextProgressLogMs.compare_exchange_weak(dueMs,
                                                            dueMs + progressLogIntervalMs,
                                                            std::memory_order_relaxed,
                                                            std::memory_order_relaxed)) {
                  auto const pct =
                      100.0 * static_cast<double>(doneSources) / static_cast<double>(N);
                  spdlog::info(
                      "computeEdgeKBetweennessCentralities progress: "
                      "{:.1f}% ({}/{})",
                      pct,
                      doneSources,
                      N);
                  break;
                }
              }
            }
          }
        });

    if (spdlog::should_log(spdlog::level::info)) {
      auto const elapsedSeconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - progressStart)
              .count();
      spdlog::info(
          "computeEdgeKBetweennessCentralities progress: 100.0% ({}/{}) in "
          "{:.2f}s",
          N,
          N,
          elapsedSeconds);
    }

    // Merge thread-local maps into the edge objects (sequential, safe).
    localAccum.combine_each([&](std::unordered_map<Id, double> const& localMap) {
      for (auto const& [eId, val] : localMap) {
        auto bccurrent = m_edges.at(eId)
                             ->template getAttribute<double>("betweennessCentrality")
                             .value_or(0.0);
        m_edges.at(eId)->setAttribute("betweennessCentrality", bccurrent + val);
      }
    });

    // ── 4. Normalisation ──────────────────────────────────────────────────
    //
    // Divide by (N-1)(N-2) — the same denominator used by the standard
    // directed-graph edge-betweenness formulation (Brandes 2001).
    // This ensures the value lies in [0, 1] when K = 1 (single shortest
    // path), and scales proportionally for K > 1.
    double const norm = static_cast<double>((N - 1) * (N - 2));
    if (norm > 0.0) {
      for (auto& [eId, pEdge] : m_edges) {
        auto bc =
            pEdge->template getAttribute<double>("betweennessCentrality").value_or(0.0);
        pEdge->setAttribute("betweennessCentrality", bc / norm);
      }
    }
  }

}  // namespace dsf