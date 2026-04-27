#pragma once

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
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

  private:
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeEdgeBetweennessBrandes(WeightFunc getEdgeWeight);
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
  // Parallel Brandes edge betweenness helper (for K = 1 fast path)
  // ──────────────────────────────────────────────────────────────────────────

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename WeightFunc>
    requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
  void Network<node_t, edge_t>::computeEdgeBetweennessBrandes(WeightFunc getEdgeWeight) {
    for (auto& [edgeId, pEdge] : m_edges)
      pEdge->setAttribute("betweennessCentrality", 0.0);

    size_t const N = m_nodes.size();
    if (N < 2)
      return;

    std::vector<Id> nodeIds;
    nodeIds.reserve(N);
    for (auto const& [id, _] : m_nodes)
      nodeIds.push_back(id);

    tbb::combinable<std::unordered_map<Id, double>> localAccum;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, N),
                      [&](tbb::blocked_range<size_t> const& range) {
                        auto& localMap = localAccum.local();

                        for (size_t i = range.begin(); i != range.end(); ++i) {
                          Id const sourceId = nodeIds[i];

                          std::vector<Id> S;
                          S.reserve(N);
                          std::unordered_map<Id, std::vector<std::pair<Id, Id>>> P;
                          std::unordered_map<Id, double> sigma;
                          std::unordered_map<Id, double> dist;

                          P.reserve(N);
                          sigma.reserve(N);
                          dist.reserve(N);

                          for (Id const nId : nodeIds) {
                            P.emplace(nId, std::vector<std::pair<Id, Id>>{});
                            sigma[nId] = 0.0;
                            dist[nId] = std::numeric_limits<double>::infinity();
                          }

                          sigma[sourceId] = 1.0;
                          dist[sourceId] = 0.0;

                          std::priority_queue<std::pair<double, Id>,
                                              std::vector<std::pair<double, Id>>,
                                              std::greater<>>
                              pq;
                          pq.push({0.0, sourceId});

                          while (!pq.empty()) {
                            auto [d, v] = pq.top();
                            pq.pop();
                            if (d > dist[v])
                              continue;

                            S.push_back(v);

                            for (auto const& eId : m_nodes.at(v)->outgoingEdges()) {
                              auto const& edgeObj = *m_edges.at(eId);
                              Id const w = edgeObj.target();
                              double const newDist = d + getEdgeWeight(edgeObj);

                              if (newDist < dist[w]) {
                                dist[w] = newDist;
                                pq.push({newDist, w});
                                sigma[w] = sigma[v];
                                auto& preds = P[w];
                                preds.clear();
                                preds.push_back({v, eId});
                              } else if (std::abs(newDist - dist[w]) <
                                         1e-12 * std::max(1.0, std::abs(dist[w]))) {
                                sigma[w] += sigma[v];
                                P[w].push_back({v, eId});
                              }
                            }
                          }

                          std::unordered_map<Id, double> delta;
                          delta.reserve(N);
                          for (Id const nId : nodeIds)
                            delta[nId] = 0.0;

                          for (auto it = S.rbegin(); it != S.rend(); ++it) {
                            Id const w = *it;
                            if (sigma[w] <= 0.0)
                              continue;

                            for (auto const& [v, eId] : P[w]) {
                              double const c = (sigma[v] / sigma[w]) * (1.0 + delta[w]);
                              delta[v] += c;
                              localMap[eId] += c;
                            }
                          }
                        }
                      });

    localAccum.combine_each([&](std::unordered_map<Id, double> const& localMap) {
      for (auto const& [eId, value] : localMap) {
        auto current = m_edges.at(eId)
                           ->template getAttribute<double>("betweennessCentrality")
                           .value_or(0.0);
        m_edges.at(eId)->setAttribute("betweennessCentrality", current + value);
      }
    });

    double const norm = static_cast<double>((N - 1) * (N - 2));
    if (norm > 0.0) {
      for (auto& [eId, pEdge] : m_edges) {
        auto const bc =
            pEdge->template getAttribute<double>("betweennessCentrality").value_or(0.0);
        pEdge->setAttribute("betweennessCentrality", bc / norm);
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
    for (auto& [eId, pEdge] : m_edges)
      pEdge->setAttribute("betweennessCentrality", 0.0);

    size_t const N = m_nodes.size();
    if (N < 2 || K == 0)
      return;
    if (K == 1) {
      computeEdgeBetweennessBrandes(getEdgeWeight);
      return;
    }

    // ---- Node indexing ----
    std::vector<Id> nodeIds;
    nodeIds.reserve(N);
    for (auto const& [id, _] : m_nodes)
      nodeIds.push_back(id);

    std::unordered_map<Id, size_t> idx;
    for (size_t i = 0; i < N; ++i)
      idx[nodeIds[i]] = i;

    // ---- Parallel accumulation ----
    tbb::combinable<std::unordered_map<Id, double>> localAccum;

    tbb::parallel_for(size_t(0), N, [&](size_t si) {
      auto& local = localAccum.local();

      // ---- Single-source shortest path tree ----
      std::vector<double> dist(N, std::numeric_limits<double>::infinity());
      std::vector<Id> parentEdge(N, Id{});
      std::vector<bool> inTree(N, false);

      using PQ = std::pair<double, size_t>;
      std::priority_queue<PQ, std::vector<PQ>, std::greater<>> pq;

      dist[si] = 0.0;
      pq.push({0.0, si});

      while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u])
          continue;

        Id uid = nodeIds[u];

        for (auto eId : m_nodes.at(uid)->outgoingEdges()) {
          auto const& e = *m_edges.at(eId);
          size_t v = idx[e.target()];
          double nd = d + getEdgeWeight(e);

          if (nd < dist[v]) {
            dist[v] = nd;
            parentEdge[v] = eId;
            pq.push({nd, v});
          }
        }
      }

      // ---- Build shortest path tree flags ----
      for (size_t i = 0; i < N; ++i)
        if (i != si && std::isfinite(dist[i]))
          inTree[i] = true;

      // ---- Process each target ----
      for (size_t ti = 0; ti < N; ++ti) {
        if (ti == si || !inTree[ti])
          continue;

        // ---- Reconstruct shortest path ----
        std::vector<Id> shortestPath;
        for (size_t v = ti; v != si;) {
          Id eId = parentEdge[v];
          shortestPath.push_back(eId);
          v = idx[m_edges.at(eId)->source()];
        }
        std::reverse(shortestPath.begin(), shortestPath.end());

        // ---- Collect sidetrack edges ----
        struct Sidetrack {
          Id edge;
          double delta;
        };

        std::vector<Sidetrack> sidetracks;

        for (auto const& [eId, ePtr] : m_edges) {
          Id u = ePtr->source();
          Id v = ePtr->target();

          size_t ui = idx[u];
          size_t vi = idx[v];

          if (!std::isfinite(dist[ui]) || !std::isfinite(dist[vi]))
            continue;

          double w = getEdgeWeight(*ePtr);

          double delta = w + dist[ui] - dist[vi];
          if (delta > 1e-12) {
            sidetracks.push_back({eId, delta});
          }
        }

        // ---- Candidate paths heap ----
        using Path = std::pair<double, std::vector<Id>>;
        std::priority_queue<Path, std::vector<Path>, std::greater<>> heap;

        heap.push({0.0, shortestPath});

        size_t produced = 0;

        while (!heap.empty() && produced < K) {
          auto [cost, path] = heap.top();
          heap.pop();

          // ---- Accumulate ----
          for (auto eId : path)
            local[eId] += 1.0;

          ++produced;

          // ---- Expand using sidetracks ----
          for (auto const& st : sidetracks) {
            std::vector<Id> newPath = path;
            newPath.push_back(st.edge);

            heap.push({cost + st.delta, std::move(newPath)});
          }
        }
      }
    });

    // ---- Merge ----
    localAccum.combine_each([&](auto const& map) {
      for (auto const& [e, v] : map) {
        auto cur = m_edges.at(e)
                       ->template getAttribute<double>("betweennessCentrality")
                       .value_or(0.0);
        m_edges.at(e)->setAttribute("betweennessCentrality", cur + v);
      }
    });

    // ---- Normalisation ----
    double norm = double((N - 1) * (N - 2));
    if (norm > 0) {
      for (auto& [_, e] : m_edges) {
        double bc =
            e->template getAttribute<double>("betweennessCentrality").value_or(0.0);
        e->setAttribute("betweennessCentrality", bc / norm);
      }
    }
  }

}  // namespace dsf