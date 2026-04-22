#pragma once

#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <stack>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
    TNode& node(Id nodeId);

    template <typename TEdge>
      requires(std::is_base_of_v<edge_t, TEdge>)
    TEdge& edge(Id edgeId);

    /// @brief Compute node betweenness centralities using Brandes' algorithm
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeBetweennessCentralities(WeightFunc getEdgeWeight);

    /// @brief Compute edge betweenness centralities using Brandes' algorithm
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeEdgeBetweennessCentralities(WeightFunc getEdgeWeight);
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

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename WeightFunc>
    requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
  void Network<node_t, edge_t>::computeBetweennessCentralities(WeightFunc getEdgeWeight) {
    for (auto& [nodeId, pNode] : m_nodes) {
      pNode->setAttribute("betweennessCentrality", 0.0);
    }

    for (auto const& [sourceId, sourceNode] : m_nodes) {
      std::stack<Id> S;
      std::unordered_map<Id, std::vector<Id>> P;
      std::unordered_map<Id, double> sigma;
      std::unordered_map<Id, double> dist;

      for (auto const& [nId, _] : m_nodes) {
        P[nId] = {};
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

      std::unordered_set<Id> visited;

      while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();

        if (visited.contains(v))
          continue;
        visited.insert(v);
        S.push(v);

        for (auto const& edgeId : m_nodes.at(v)->outgoingEdges()) {
          auto const& edgeObj = *m_edges.at(edgeId);
          Id w = edgeObj.target();
          if (visited.contains(w))
            continue;
          double edgeWeight = getEdgeWeight(edgeObj);
          double newDist = dist[v] + edgeWeight;

          if (newDist < dist[w]) {
            dist[w] = newDist;
            sigma[w] = sigma[v];
            P[w] = {v};
            pq.push({newDist, w});
          } else if (std::abs(newDist - dist[w]) < 1e-12 * std::max(1.0, dist[w])) {
            sigma[w] += sigma[v];
            P[w].push_back(v);
          }
        }
      }

      std::unordered_map<Id, double> delta;
      for (auto const& [nId, _] : m_nodes)
        delta[nId] = 0.0;

      while (!S.empty()) {
        Id w = S.top();
        S.pop();
        for (Id v : P[w]) {
          delta[v] += (sigma[v] / sigma[w]) * (1.0 + delta[w]);
        }
        if (w != sourceId) {
          auto currentBC = m_nodes.at(w)
                               ->template getAttribute<double>("betweennessCentrality")
                               .value_or(0.0);
          m_nodes.at(w)->setAttribute("betweennessCentrality", currentBC + delta[w]);
        }
      }
    }
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename WeightFunc>
    requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
  void Network<node_t, edge_t>::computeEdgeBetweennessCentralities(
      WeightFunc getEdgeWeight) {
    for (auto& [edgeId, pEdge] : m_edges) {
      pEdge->setAttribute("betweennessCentrality", 0.0);
    }

    for (auto const& [sourceId, sourceNode] : m_nodes) {
      std::stack<Id> S;
      std::unordered_map<Id, std::vector<std::pair<Id, Id>>> P;
      std::unordered_map<Id, double> sigma;
      std::unordered_map<Id, double> dist;

      for (auto const& [nId, _] : m_nodes) {
        P[nId] = {};
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

      std::unordered_set<Id> visited;

      while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();

        if (visited.contains(v))
          continue;
        visited.insert(v);
        S.push(v);

        for (auto const& eId : m_nodes.at(v)->outgoingEdges()) {
          auto const& edgeObj = *m_edges.at(eId);
          Id w = edgeObj.target();
          if (visited.contains(w))
            continue;
          double edgeWeight = getEdgeWeight(edgeObj);
          double newDist = dist[v] + edgeWeight;

          if (newDist < dist[w]) {
            dist[w] = newDist;
            sigma[w] = sigma[v];
            P[w] = {{v, eId}};
            pq.push({newDist, w});
          } else if (std::abs(newDist - dist[w]) < 1e-12 * std::max(1.0, dist[w])) {
            sigma[w] += sigma[v];
            P[w].push_back({v, eId});
          }
        }
      }

      std::unordered_map<Id, double> delta;
      for (auto const& [nId, _] : m_nodes)
        delta[nId] = 0.0;

      while (!S.empty()) {
        Id w = S.top();
        S.pop();
        for (auto const& [v, eId] : P[w]) {
          double c = (sigma[v] / sigma[w]) * (1.0 + delta[w]);
          delta[v] += c;
          auto currentBC = m_edges.at(eId)
                               ->template getAttribute<double>("betweennessCentrality")
                               .value_or(0.0);
          m_edges.at(eId)->setAttribute("betweennessCentrality", currentBC + c);
        }
      }
    }
  }
}  // namespace dsf