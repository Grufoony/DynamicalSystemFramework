#include "Router.hpp"

#include "RoadNetwork.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <spdlog/spdlog.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

namespace dsf::mobility {
  namespace {
    constexpr std::uint32_t NO_STREET{std::numeric_limits<std::uint32_t>::max()};
    // Relative margin on the heuristic factor, so that rounding never makes the heuristic
    // exceed the cost of the street it was calibrated on
    constexpr double HEURISTIC_MARGIN{1e-12};

    // Pops the entry with the lowest f first; on ties, the deepest one (highest g), which
    // is closer to the goal, then the lowest street index, for reproducibility
    struct HeapOrder {
      template <typename TEntry>
      bool operator()(TEntry const& lhs, TEntry const& rhs) const noexcept {
        if (lhs.f != rhs.f) {
          return lhs.f > rhs.f;
        }
        if (lhs.g != rhs.g) {
          return lhs.g < rhs.g;
        }
        return lhs.street > rhs.street;
      }
    };
  }  // namespace

  void Router::SearchBuffers::prepare(std::size_t const nStreets) {
    if (stamp.size() != nStreets) {
      cost.assign(nStreets, 0.);
      parent.assign(nStreets, NO_STREET);
      stamp.assign(nStreets, 0);
      banStamp.assign(nStreets, 0);
      generation = 0;
    }
    if (++generation == 0) {
      std::ranges::fill(stamp, 0);
      std::ranges::fill(banStamp, 0);
      generation = 1;
    }
    heap.clear();
  }

  void Router::build(RoadNetwork const& network) {
    // Nodes and streets are indexed by ascending id, so that ties are broken the same way
    // whatever the iteration order of the network's containers
    std::vector<Id> nodeIds;
    nodeIds.reserve(network.nNodes());
    for (auto const& [nodeId, pNode] : network.nodes()) {
      nodeIds.push_back(nodeId);
    }
    std::ranges::sort(nodeIds);
    m_nodeIndices.clear();
    m_nodeIndices.reserve(nodeIds.size());
    for (std::uint32_t i{0}; i < nodeIds.size(); ++i) {
      m_nodeIndices.emplace(nodeIds[i], i);
    }

    m_streetIds.clear();
    m_streetIds.reserve(network.nEdges());
    for (auto const& [streetId, pStreet] : network.edges()) {
      m_streetIds.push_back(streetId);
    }
    std::ranges::sort(m_streetIds);
    auto const nStreets{m_streetIds.size()};
    m_streetIndices.clear();
    m_streetIndices.reserve(nStreets);
    m_streets.resize(nStreets);
    m_streetSources.resize(nStreets);
    m_streetTargets.resize(nStreets);
    for (std::uint32_t i{0}; i < nStreets; ++i) {
      auto const& street{network.edge(m_streetIds[i])};
      m_streetIndices.emplace(m_streetIds[i], i);
      m_streets[i] = &street;
      m_streetSources[i] = m_nodeIndices.at(street.source()->id());
      m_streetTargets[i] = m_nodeIndices.at(street.target()->id());
    }

    // Allowed turns: into an open street, following the network's turn rules
    m_turnOffsets.assign(nStreets + 1, 0);
    m_turns.clear();
    for (std::uint32_t i{0}; i < nStreets; ++i) {
      auto const& street{*m_streets[i]};
      for (auto const* pNext : street.target()->outgoingEdges()) {
        if (pNext->isActive() && RoadNetwork::isTurnAllowed(street, *pNext)) {
          m_turns.push_back(m_streetIndices.at(pNext->id()));
        }
      }
      m_turnOffsets[i + 1] = static_cast<std::uint32_t>(m_turns.size());
    }
    // Open outgoing streets of each node, for the routes starting from a node
    m_nodeOutOffsets.assign(nodeIds.size() + 1, 0);
    m_nodeOut.clear();
    for (std::uint32_t i{0}; i < nodeIds.size(); ++i) {
      for (auto const* pStreet : network.node(nodeIds[i]).outgoingEdges()) {
        if (pStreet->isActive()) {
          m_nodeOut.push_back(m_streetIndices.at(pStreet->id()));
        }
      }
      m_nodeOutOffsets[i + 1] = static_cast<std::uint32_t>(m_nodeOut.size());
    }

    m_buildHeuristic(network, nodeIds);

    m_freeFlow.weights.resize(nStreets);
    for (std::size_t i{0}; i < nStreets; ++i) {
      m_freeFlow.weights[i] = m_streets[i]->length() / m_streets[i]->maxSpeed();
    }
    m_freeFlow.heuristicFactor = m_heuristicFactor(m_freeFlow.weights);

    updateCurrentWeights(network);
    spdlog::debug("Built the routing graph: {} streets, {} turns, heuristic {}",
                  nStreets,
                  m_turns.size(),
                  m_bHeuristic ? "on" : "off");
  }

  void Router::m_buildHeuristic(RoadNetwork const& network, std::span<Id const> nodeIds) {
    m_bHeuristic = false;
    m_nodeX.clear();
    m_nodeY.clear();
    m_streetDistances.clear();
    // Only the nodes touching a street can be on a route
    std::vector<bool> bTouched(nodeIds.size(), false);
    for (std::size_t i{0}; i < m_streetIds.size(); ++i) {
      bTouched[m_streetSources[i]] = true;
      bTouched[m_streetTargets[i]] = true;
    }
    bool bLonLat{true};
    double sumY{0.};
    std::size_t nTouched{0};
    for (std::size_t i{0}; i < nodeIds.size(); ++i) {
      if (!bTouched[i]) {
        continue;
      }
      auto const& optGeometry{network.node(nodeIds[i]).geometry()};
      if (!optGeometry.has_value()) {
        spdlog::debug("Node {} has no coordinates: routes are planned without heuristic.",
                      nodeIds[i]);
        return;
      }
      bLonLat = bLonLat && std::abs(optGeometry->x()) <= 180. &&
                std::abs(optGeometry->y()) <= 90.;
      sumY += optGeometry->y();
      ++nTouched;
    }
    if (nTouched == 0) {
      return;
    }
    // Longitude degrees shrink with the latitude: rescale them to keep distances isotropic.
    // Any metric keeps the heuristic admissible, as its scale is calibrated on the streets.
    auto const xScale{
        bLonLat ? std::cos(sumY / static_cast<double>(nTouched) * std::numbers::pi / 180.)
                : 1.};
    m_nodeX.assign(nodeIds.size(), 0.);
    m_nodeY.assign(nodeIds.size(), 0.);
    for (std::size_t i{0}; i < nodeIds.size(); ++i) {
      if (bTouched[i]) {
        auto const& point{*network.node(nodeIds[i]).geometry()};
        m_nodeX[i] = point.x() * xScale;
        m_nodeY[i] = point.y();
      }
    }
    m_streetDistances.resize(m_streetIds.size());
    for (std::size_t i{0}; i < m_streetIds.size(); ++i) {
      m_streetDistances[i] =
          std::hypot(m_nodeX[m_streetTargets[i]] - m_nodeX[m_streetSources[i]],
                     m_nodeY[m_streetTargets[i]] - m_nodeY[m_streetSources[i]]);
    }
    m_bHeuristic = true;
  }

  double Router::m_heuristicFactor(std::span<double const> weights) const {
    if (!m_bHeuristic) {
      return 0.;
    }
    auto factor{std::numeric_limits<double>::infinity()};
    for (std::size_t i{0}; i < weights.size(); ++i) {
      if (m_streetDistances[i] > 0.) {
        factor = std::min(factor, weights[i] / m_streetDistances[i]);
      }
    }
    // No street with a positive length, or a non-positive cost: no usable heuristic
    if (!std::isfinite(factor) || !(factor > 0.)) {
      return 0.;
    }
    return factor * (1. - HEURISTIC_MARGIN);
  }

  void Router::updateCurrentWeights(RoadNetwork const& network) {
    auto const nStreets{m_streetIds.size()};
    m_current.weights.resize(nStreets);
    if (!m_bHeuristic) {
      tbb::parallel_for(tbb::blocked_range<std::size_t>(0, nStreets),
                        [&](tbb::blocked_range<std::size_t> const& range) {
                          for (auto i{range.begin()}; i != range.end(); ++i) {
                            m_current.weights[i] = network.edgeWeight(*m_streets[i]);
                          }
                        });
      m_current.heuristicFactor = 0.;
      return;
    }
    // Take the snapshot and calibrate the heuristic in the same pass
    auto const factor{tbb::parallel_reduce(
        tbb::blocked_range<std::size_t>(0, nStreets),
        std::numeric_limits<double>::infinity(),
        [&](tbb::blocked_range<std::size_t> const& range, double minFactor) {
          for (auto i{range.begin()}; i != range.end(); ++i) {
            auto const weight{network.edgeWeight(*m_streets[i])};
            m_current.weights[i] = weight;
            if (m_streetDistances[i] > 0.) {
              minFactor = std::min(minFactor, weight / m_streetDistances[i]);
            }
          }
          return minFactor;
        },
        [](double const lhs, double const rhs) { return std::min(lhs, rhs); })};
    m_current.heuristicFactor =
        (std::isfinite(factor) && factor > 0.) ? factor * (1. - HEURISTIC_MARGIN) : 0.;
  }

  std::uint32_t Router::m_streetIndex(Id const streetId) const {
    auto const it{m_streetIndices.find(streetId)};
    if (it == m_streetIndices.cend()) {
      throw std::out_of_range(std::format(
          "Street {} is not in the routing graph: rebuild it after changing the network.",
          streetId));
    }
    return it->second;
  }
  std::uint32_t Router::m_nodeIndex(Id const nodeId) const {
    auto const it{m_nodeIndices.find(nodeId)};
    if (it == m_nodeIndices.cend()) {
      throw std::out_of_range(std::format(
          "Node {} is not in the routing graph: rebuild it after changing the network.",
          nodeId));
    }
    return it->second;
  }
  Router::WeightSet const& Router::m_weightSet(RouteWeights const weights) const noexcept {
    return weights == RouteWeights::CURRENT ? m_current : m_freeFlow;
  }

  std::span<std::uint32_t const> Router::m_nodeOutgoing(std::uint32_t const node) const {
    return std::span{m_nodeOut}.subspan(
        m_nodeOutOffsets[node], m_nodeOutOffsets[node + 1] - m_nodeOutOffsets[node]);
  }
  std::vector<Id> Router::m_toIds(std::span<std::uint32_t const> streets) const {
    std::vector<Id> ids;
    ids.reserve(streets.size());
    for (auto const street : streets) {
      ids.push_back(m_streetIds[street]);
    }
    return ids;
  }

  std::vector<Id> Router::route(Id const fromStreetId,
                                Id const goalNodeId,
                                RouteWeights const weights) const {
    auto const start{m_streetIndex(fromStreetId)};
    return m_toIds(m_search(
        std::span{&start, 1}, false, m_nodeIndex(goalNodeId), m_weightSet(weights)));
  }

  std::vector<Id> Router::routeFromNode(Id const fromNodeId,
                                        Id const goalNodeId,
                                        RouteWeights const weights) const {
    auto const node{m_nodeIndex(fromNodeId)};
    return m_toIds(m_search(
        m_nodeOutgoing(node), true, m_nodeIndex(goalNodeId), m_weightSet(weights)));
  }

  std::vector<Route> Router::kShortestRoutes(Id const fromStreetId,
                                             Id const goalNodeId,
                                             RouteWeights const weights,
                                             std::size_t const k) const {
    auto const start{m_streetIndex(fromStreetId)};
    return m_kShortest(
        std::span{&start, 1}, false, m_nodeIndex(goalNodeId), m_weightSet(weights), k);
  }

  std::vector<Route> Router::kShortestRoutesFromNode(Id const fromNodeId,
                                                     Id const goalNodeId,
                                                     RouteWeights const weights,
                                                     std::size_t const k) const {
    auto const node{m_nodeIndex(fromNodeId)};
    return m_kShortest(
        m_nodeOutgoing(node), true, m_nodeIndex(goalNodeId), m_weightSet(weights), k);
  }

  double Router::routeCost(std::span<Id const> route, RouteWeights const weights) const {
    auto const& weightSet{m_weightSet(weights)};
    double cost{0.};
    for (std::size_t i{1}; i < route.size(); ++i) {
      cost += weightSet.weights[m_streetIndex(route[i])];
    }
    return cost;
  }

  bool Router::isTurnAllowed(Id const fromStreetId, Id const toStreetId) const {
    auto const itFrom{m_streetIndices.find(fromStreetId)};
    auto const itTo{m_streetIndices.find(toStreetId)};
    if (itFrom == m_streetIndices.cend() || itTo == m_streetIndices.cend()) {
      return false;
    }
    auto const turns{std::span{m_turns}.subspan(
        m_turnOffsets[itFrom->second],
        m_turnOffsets[itFrom->second + 1] - m_turnOffsets[itFrom->second])};
    return std::ranges::find(turns, itTo->second) != turns.end();
  }

  std::vector<std::uint32_t> Router::m_search(
      std::span<std::uint32_t const> starts,
      bool const bCountStartCost,
      std::uint32_t const goalNode,
      WeightSet const& weightSet,
      std::span<std::uint32_t const> bannedStreets,
      std::span<std::uint32_t const> bannedFirstTurns) const {
    assert(weightSet.weights.size() == m_streetIds.size());
    auto& buffers{m_buffers.local()};
    buffers.prepare(m_streetIds.size());
    auto const generation{buffers.generation};
    auto& cost{buffers.cost};
    auto& parent{buffers.parent};
    auto& stamp{buffers.stamp};
    auto& heap{buffers.heap};
    auto& banStamp{buffers.banStamp};
    for (auto const street : bannedStreets) {
      banStamp[street] = generation;
    }

    auto const factor{weightSet.heuristicFactor};
    auto const goalX{factor > 0. ? m_nodeX[goalNode] : 0.};
    auto const goalY{factor > 0. ? m_nodeY[goalNode] : 0.};
    auto const heuristic = [&](std::uint32_t const street) {
      if (factor == 0.) {
        return 0.;
      }
      auto const node{m_streetTargets[street]};
      return factor * std::hypot(m_nodeX[node] - goalX, m_nodeY[node] - goalY);
    };
    auto const& streetCost{weightSet.weights};
    // Record a cheaper way to reach a street, and queue it
    auto const relax =
        [&](std::uint32_t const street, double const g, std::uint32_t const from) {
          if (banStamp[street] == generation ||
              (stamp[street] == generation && cost[street] <= g)) {
            return;
          }
          stamp[street] = generation;
          cost[street] = g;
          parent[street] = from;
          heap.push_back({g + heuristic(street), g, street});
          std::ranges::push_heap(heap, HeapOrder{});
        };

    for (auto const street : starts) {
      relax(street, bCountStartCost ? streetCost[street] : 0., NO_STREET);
    }
    while (!heap.empty()) {
      std::ranges::pop_heap(heap, HeapOrder{});
      auto const g{heap.back().g};
      auto const street{heap.back().street};
      heap.pop_back();
      if (g > cost[street]) {
        continue;  // A cheaper way to this street was found after queueing this one
      }
      if (m_streetTargets[street] == goalNode) {
        std::vector<std::uint32_t> route;
        for (auto current{street}; current != NO_STREET; current = parent[current]) {
          route.push_back(current);
        }
        std::ranges::reverse(route);
        return route;
      }
      bool const bStart{parent[street] == NO_STREET};
      for (auto i{m_turnOffsets[street]}; i < m_turnOffsets[street + 1]; ++i) {
        auto const next{m_turns[i]};
        if (bStart &&
            std::ranges::find(bannedFirstTurns, next) != bannedFirstTurns.end()) {
          continue;
        }
        relax(next, g + streetCost[next], street);
      }
    }
    return {};
  }

  std::vector<Route> Router::m_kShortest(std::span<std::uint32_t const> starts,
                                         bool const bNodeStart,
                                         std::uint32_t const goalNode,
                                         WeightSet const& weightSet,
                                         std::size_t const k) const {
    auto const& weights{weightSet.weights};
    // The cost of a route: from a node, its first street counts too
    auto const costOf = [&](std::span<std::uint32_t const> streets) {
      double cost{0.};
      for (std::size_t i{bNodeStart ? 0u : 1u}; i < streets.size(); ++i) {
        cost += weights[streets[i]];
      }
      return cost;
    };
    std::vector<IndexRoute> accepted;
    if (k == 0) {
      return {};
    }
    auto firstRoute{m_search(starts, bNodeStart, goalNode, weightSet)};
    if (firstRoute.empty()) {
      return {};
    }
    auto const firstCost{costOf(firstRoute)};
    accepted.push_back({std::move(firstRoute), firstCost});
    // Candidates, in order of discovery: the cheapest one is accepted at each round
    std::vector<IndexRoute> candidates;
    auto const isKnown = [&](std::vector<std::uint32_t> const& streets) {
      auto const sameStreets = [&](IndexRoute const& route) {
        return route.streets == streets;
      };
      return std::ranges::any_of(accepted, sameStreets) ||
             std::ranges::any_of(candidates, sameStreets);
    };
    std::vector<std::uint32_t> bannedTurns, otherStarts;
    while (accepted.size() < k) {
      auto const previous{accepted.back().streets};
      if (bNodeStart) {
        // Spur at the start node: the first streets of the accepted routes are banned
        otherStarts.clear();
        for (auto const street : starts) {
          if (std::ranges::none_of(accepted, [street](IndexRoute const& route) {
                return route.streets.front() == street;
              })) {
            otherStarts.push_back(street);
          }
        }
        auto spurRoute{m_search(otherStarts, true, goalNode, weightSet)};
        if (!spurRoute.empty() && !isKnown(spurRoute)) {
          auto const cost{costOf(spurRoute)};
          candidates.push_back({std::move(spurRoute), cost});
        }
      }
      for (std::size_t spur{0}; spur + 1 < previous.size(); ++spur) {
        auto const root{std::span{previous}.first(spur + 1)};
        // The route may not go back on the root, nor follow an accepted route sharing it
        bannedTurns.clear();
        for (auto const& route : accepted) {
          if (route.streets.size() > spur + 1 &&
              std::ranges::equal(std::span{route.streets}.first(spur + 1), root)) {
            bannedTurns.push_back(route.streets[spur + 1]);
          }
        }
        auto const spurRoute{m_search(std::span{&previous[spur], 1},
                                      false,
                                      goalNode,
                                      weightSet,
                                      root.first(spur),
                                      bannedTurns)};
        if (spurRoute.empty()) {
          continue;
        }
        std::vector<std::uint32_t> streets(root.begin(), root.end() - 1);
        streets.insert(streets.end(), spurRoute.begin(), spurRoute.end());
        if (!isKnown(streets)) {
          auto const cost{costOf(streets)};
          candidates.push_back({std::move(streets), cost});
        }
      }
      if (candidates.empty()) {
        break;
      }
      auto const itBest{std::ranges::min_element(candidates, {}, &IndexRoute::cost)};
      accepted.push_back(std::move(*itBest));
      candidates.erase(itBest);
    }
    std::vector<Route> routes;
    routes.reserve(accepted.size());
    for (auto const& route : accepted) {
      routes.push_back({m_toIds(route.streets), route.cost});
    }
    return routes;
  }

  std::size_t Router::chooseRoute(std::span<Route const> routes,
                                  double const randomValue) {
    if (routes.size() < 2) {
      return 0;
    }
    auto const lowestCost{std::ranges::min(routes, {}, &Route::cost).cost};
    if (!(lowestCost > 0.) || !std::isfinite(lowestCost)) {
      return static_cast<std::size_t>(std::ranges::distance(
          routes.begin(), std::ranges::min_element(routes, {}, &Route::cost)));
    }
    // exp(-cost / lowest) is proportional to exp(-(cost - lowest) / lowest), which cannot
    // underflow for the best route
    std::vector<double> cumulativeWeights;
    cumulativeWeights.reserve(routes.size());
    double total{0.};
    for (auto const& route : routes) {
      total += std::exp(-(route.cost - lowestCost) / lowestCost);
      cumulativeWeights.push_back(total);
    }
    auto const target{randomValue * total};
    auto const it{std::ranges::upper_bound(cumulativeWeights, target)};
    if (it == cumulativeWeights.end()) {
      return routes.size() - 1;
    }
    return static_cast<std::size_t>(std::ranges::distance(cumulativeWeights.begin(), it));
  }
}  // namespace dsf::mobility
