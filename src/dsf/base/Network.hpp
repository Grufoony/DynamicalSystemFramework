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

    Id m_cantorHash(Id u, Id v) const;
    Id m_cantorHash(std::pair<Id, Id> const& idPair) const;

  public:
    /// @brief Construct a new empty Network object
    Network() = default;

    /// @brief Get the nodes as an unordered map
    /// @return std::unordered_map<Id, std::unique_ptr<node_t>> The nodes
    std::unordered_map<Id, std::unique_ptr<node_t>> const& nodes() const;
    /// @brief Get the edges as an unordered map
    /// @return std::unordered_map<Id, std::unique_ptr<edge_t>> The edges
    std::unordered_map<Id, std::unique_ptr<edge_t>> const& edges() const;
    /// @brief Get the number of nodes
    /// @return size_t The number of nodes
    size_t nNodes() const;
    /// @brief Get the number of edges
    /// @return size_t The number of edges
    size_t nEdges() const;

    /// @brief Add a node to the network
    /// @tparam TNode The type of the node (default is node_t)
    /// @tparam TArgs The types of the arguments
    /// @param args The arguments to pass to the node's constructor
    template <typename TNode = node_t, typename... TArgs>
      requires(std::is_base_of_v<node_t, TNode> &&
               std::constructible_from<TNode, TArgs...>)
    void addNode(TArgs&&... args);

    template <typename TNode = node_t>
      requires(std::is_base_of_v<node_t, TNode>)
    void addNDefaultNodes(size_t n);

    /// @brief Add an edge to the network
    /// @tparam TEdge The type of the edge (default is edge_t)
    /// @tparam TArgs The types of the arguments
    /// @param args The arguments to pass to the edge's constructor
    template <typename TEdge = edge_t, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> &&
               std::constructible_from<TEdge, TArgs...>)
    void addEdge(TArgs&&... args);

    /// @brief Get a node by id
    /// @param nodeId The node's id
    /// @return node_t& A reference to the node
    inline node_t& node(Id nodeId) const { return *m_nodes.at(nodeId); };
    /// @brief Get a node by id
    /// @param nodeId The node's id
    /// @return node_t& A reference to the node
    inline node_t& node(Id nodeId) { return *m_nodes.at(nodeId); };
    /// @brief Get an edge by id
    /// @param edgeId The edge's id
    /// @return edge_t& A reference to the edge
    inline edge_t& edge(Id edgeId) const { return *m_edges.at(edgeId); };
    /// @brief Get an edge by id
    /// @param edgeId The edge's id
    /// @return edge_t& A reference to the edge
    inline edge_t& edge(Id edgeId) { return *m_edges.at(edgeId); }

    edge_t& edge(Id source, Id target) const;
    /// @brief Get a node by id
    /// @tparam TNode The type of the node
    /// @param nodeId The node's id
    /// @return TNode& A reference to the node
    template <typename TNode>
      requires(std::is_base_of_v<node_t, TNode>)
    TNode& node(Id nodeId);
    /// @brief Get an edge by id
    /// @tparam TEdge The type of the edge
    /// @param edgeId The edge's id
    /// @return TEdge& A reference to the edge
    template <typename TEdge>
      requires(std::is_base_of_v<edge_t, TEdge>)
    TEdge& edge(Id edgeId);

    /// @brief Compute betweenness centralities for all nodes using Brandes' algorithm
    /// @tparam WeightFunc A callable type that takes a const reference to edge_t and returns a double representing the edge weight
    /// @param getEdgeWeight A callable that takes a const reference to edge_t and returns a double (must be positive)
    /// @details Implements Brandes' algorithm for directed weighted graphs.
    ///          The computed centrality for each node v is:
    ///          C_B(v) = sum_{s != v != t} sigma_st(v) / sigma_st
    ///          where sigma_st is the number of shortest paths from s to t,
    ///          and sigma_st(v) is the number of those paths passing through v.
    ///          Results are stored via Node::setBetweennessCentrality.
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeBetweennessCentralities(WeightFunc getEdgeWeight);

    /// @brief Compute edge betweenness centralities for all edges using Brandes' algorithm
    /// @tparam WeightFunc A callable type that takes a const reference to edge_t and returns a double representing the edge weight
    /// @param getEdgeWeight A callable that takes a const reference to edge_t and returns a double (must be positive)
    /// @details Implements Brandes' algorithm for directed weighted graphs.
    ///          The computed centrality for each edge e is:
    ///          C_B(e) = sum_{s != t} sigma_st(e) / sigma_st
    ///          where sigma_st is the number of shortest paths from s to t,
    ///          and sigma_st(e) is the number of those paths using edge e.
    ///          Results are stored via Edge::setBetweennessCentrality.
    template <typename WeightFunc>
      requires(std::is_invocable_r_v<double, WeightFunc, edge_t const&>)
    void computeEdgeBetweennessCentralities(WeightFunc getEdgeWeight);
  };
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Id Network<node_t, edge_t>::m_cantorHash(Id u, Id v) const {
    return ((u + v) * (u + v + 1)) / 2 + v;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Id Network<node_t, edge_t>::m_cantorHash(std::pair<Id, Id> const& idPair) const {
    return m_cantorHash(idPair.first, idPair.second);
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unordered_map<Id, std::unique_ptr<node_t>> const& Network<node_t, edge_t>::nodes()
      const {
    return m_nodes;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unordered_map<Id, std::unique_ptr<edge_t>> const& Network<node_t, edge_t>::edges()
      const {
    return m_edges;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  size_t Network<node_t, edge_t>::nNodes() const {
    return m_nodes.size();
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  size_t Network<node_t, edge_t>::nEdges() const {
    return m_edges.size();
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TNode, typename... TArgs>
    requires(std::is_base_of_v<node_t, TNode> && std::constructible_from<TNode, TArgs...>)
  void Network<node_t, edge_t>::addNode(TArgs&&... args) {
    // Create unique_ptr directly with perfect forwarding
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

    // Check if source node exists, add if not
    if (!m_nodes.contains(sourceNodeId)) {
      if (!geometry.empty()) {
        addNode(tmpEdge.source(), geometry.front());
      } else {
        addNode(tmpEdge.source());
      }
    }

    // Check if target node exists, add if not
    if (!m_nodes.contains(targetNodeId)) {
      if (!geometry.empty()) {
        addNode(tmpEdge.target(), geometry.back());
      } else {
        addNode(tmpEdge.target());
      }
    }

    // Get fresh references to both nodes after all potential vector reallocations
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
    // Initialize all node betweenness centralities to 0
    for (auto& [nodeId, pNode] : m_nodes) {
      pNode->setBetweennessCentrality(0.0);
    }

    // Brandes' algorithm: run single-source Dijkstra from each node
    for (auto const& [sourceId, sourceNode] : m_nodes) {
      std::stack<Id> S;  // nodes in order of non-increasing distance
      std::unordered_map<Id, std::vector<Id>> P;  // predecessors on shortest paths
      std::unordered_map<Id, double> sigma;       // number of shortest paths
      std::unordered_map<Id, double> dist;        // distance from source

      for (auto const& [nId, _] : m_nodes) {
        P[nId] = {};
        sigma[nId] = 0.0;
        dist[nId] = std::numeric_limits<double>::infinity();
      }
      sigma[sourceId] = 1.0;
      dist[sourceId] = 0.0;

      // Min-heap priority queue: (distance, nodeId)
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

        for (auto const& edgeId : m_nodes.at(v)->outgoingEdges()) {
          auto const& edgeObj = *m_edges.at(edgeId);
          Id w = edgeObj.target();
          if (visited.contains(w)) {
            continue;
          }
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

      // Dependency accumulation (backward pass)
      std::unordered_map<Id, double> delta;
      for (auto const& [nId, _] : m_nodes) {
        delta[nId] = 0.0;
      }

      while (!S.empty()) {
        Id w = S.top();
        S.pop();
        for (Id v : P[w]) {
          delta[v] += (sigma[v] / sigma[w]) * (1.0 + delta[w]);
        }
        if (w != sourceId) {
          auto currentBC = m_nodes.at(w)->betweennessCentrality();
          m_nodes.at(w)->setBetweennessCentrality(*currentBC + delta[w]);
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
    // Initialize all edge betweenness centralities to 0
    for (auto& [edgeId, pEdge] : m_edges) {
      pEdge->setBetweennessCentrality(0.0);
    }

    // Brandes' algorithm: run single-source Dijkstra from each node
    for (auto const& [sourceId, sourceNode] : m_nodes) {
      std::stack<Id> S;  // nodes in order of non-increasing distance
      // predecessors: P[w] = list of (predecessor node id, edge id from pred to w)
      std::unordered_map<Id, std::vector<std::pair<Id, Id>>> P;
      std::unordered_map<Id, double> sigma;  // number of shortest paths
      std::unordered_map<Id, double> dist;   // distance from source

      for (auto const& [nId, _] : m_nodes) {
        P[nId] = {};
        sigma[nId] = 0.0;
        dist[nId] = std::numeric_limits<double>::infinity();
      }
      sigma[sourceId] = 1.0;
      dist[sourceId] = 0.0;

      // Min-heap priority queue: (distance, nodeId)
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

        for (auto const& eId : m_nodes.at(v)->outgoingEdges()) {
          auto const& edgeObj = *m_edges.at(eId);
          Id w = edgeObj.target();
          if (visited.contains(w)) {
            continue;
          }
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

      // Dependency accumulation (backward pass)
      std::unordered_map<Id, double> delta;
      for (auto const& [nId, _] : m_nodes) {
        delta[nId] = 0.0;
      }

      while (!S.empty()) {
        Id w = S.top();
        S.pop();
        for (auto const& [v, eId] : P[w]) {
          double c = (sigma[v] / sigma[w]) * (1.0 + delta[w]);
          delta[v] += c;
          // Accumulate edge betweenness
          auto currentBC = m_edges.at(eId)->betweennessCentrality();
          m_edges.at(eId)->setBetweennessCentrality(*currentBC + c);
        }
      }
    }
  }
}  // namespace dsf