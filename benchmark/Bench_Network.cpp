#include "dsf/mobility/RoadNetwork.hpp"
#include "dsf/mobility/Router.hpp"

#include <algorithm>
#include <filesystem>
#include <random>
#include <utility>
#include <vector>

#include <benchmark/benchmark.h>
#include <spdlog/spdlog.h>

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "test/data";

[[maybe_unused]] static const bool SPDLOG_SILENCED = [] {
  spdlog::set_level(spdlog::level::off);
  return true;
}();

static dsf::mobility::RoadNetwork makeForliNetwork() {
  dsf::mobility::RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  return network;
}

// Importing Forlì is expensive, while Google Benchmark calls each benchmark function
// several times (repetitions, iteration estimation): import it once and share it. The
// benchmarks using it either do not modify it or apply idempotent operations.
static dsf::mobility::RoadNetwork& sharedForliNetwork() {
  static auto network = makeForliNetwork();
  return network;
}

static void BM_RoadNetwork_AddNode(benchmark::State& state) {
  dsf::mobility::RoadNetwork network;
  dsf::Id nodeId{0};
  for (auto _ : state) {
    network.addNode(nodeId++);
  }
}
static void BM_RoadNetwork_AddEdge(benchmark::State& state) {
  dsf::mobility::RoadNetwork network;
  dsf::Id edgeId{0};
  for (auto _ : state) {
    // Edge i goes from node i to node i + 1
    network.addEdge(edgeId, std::make_pair(edgeId, edgeId + 1));
    ++edgeId;
  }
}
static void BM_RoadNetwork_CSVImport(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::RoadNetwork network;
    network.importEdges((DATA_FOLDER / "postua_edges.csv").string());
    network.importNodeProperties((DATA_FOLDER / "postua_nodes.csv").string());
  }
}
static void BM_RoadNetwork_GeoJSONImport(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::RoadNetwork network;
    network.importEdges((DATA_FOLDER / "postua_edges.geojson").string());
    network.importNodeProperties((DATA_FOLDER / "postua_nodes.csv").string());
  }
}
static void BM_RoadNetwork_NodesLooping(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  for (auto _ : state) {
    for (auto const& [id, node] : network.nodes()) {
      auto nodeId{static_cast<dsf::Id>(id)};
      auto* pNode{node};
      benchmark::DoNotOptimize(nodeId);
      benchmark::DoNotOptimize(pNode);
    }
  }
}
static void BM_RoadNetwork_EdgesLooping(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  for (auto _ : state) {
    for (auto const& [id, edge] : network.edges()) {
      auto edgeId{static_cast<dsf::Id>(id)};
      auto* pEdge{edge};
      benchmark::DoNotOptimize(edgeId);
      benchmark::DoNotOptimize(pEdge);
    }
  }
}
static void BM_RoadNetwork_NodeRandomAccess(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  auto itNode = network.nodes().cbegin();
  for (auto _ : state) {
    benchmark::DoNotOptimize(network.node((*itNode).first));
    ++itNode;
    if (itNode == network.nodes().cend()) {
      itNode = network.nodes().cbegin();
    }
  }
}
static void BM_RoadNetwork_EdgeRandomAccess(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  auto itEdge = network.edges().cbegin();
  for (auto _ : state) {
    benchmark::DoNotOptimize(network.edge((*itEdge).first));
    ++itEdge;
    if (itEdge == network.edges().cend()) {
      itEdge = network.edges().cbegin();
    }
  }
}
static void BM_RoadNetwork_CSVImport_Forli(benchmark::State& state) {
  for (auto _ : state) {
    auto network = makeForliNetwork();
    benchmark::DoNotOptimize(network.nEdges());
  }
}
static void BM_RoadNetwork_AdjustNodeCapacities(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  for (auto _ : state) {
    network.adjustNodeCapacities();
  }
}
static void BM_RoadNetwork_AutoAssignRoadPriorities(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  for (auto _ : state) {
    network.autoAssignRoadPriorities();
  }
}
static void BM_RoadNetwork_StreetLookup(benchmark::State& state) {
  auto const& network = sharedForliNetwork();
  std::vector<std::pair<dsf::Id, dsf::Id>> endpoints;
  endpoints.reserve(network.nEdges());
  for (auto const& [edgeId, pEdge] : network.edges()) {
    endpoints.emplace_back(pEdge->source()->id(), pEdge->target()->id());
  }
  std::sort(endpoints.begin(), endpoints.end());
  std::size_t i{0};
  for (auto _ : state) {
    auto const& [source, target] = endpoints[i];
    benchmark::DoNotOptimize(network.street(source, target));
    i = (i + 997) % endpoints.size();
  }
}
// Random (origin street, goal node) pairs, the same in every run
static std::vector<std::pair<dsf::Id, dsf::Id>> routeQueries(
    dsf::mobility::RoadNetwork const& network, std::size_t const nQueries) {
  std::vector<dsf::Id> streetIds, nodeIds;
  for (auto const& [streetId, pStreet] : network.edges()) {
    streetIds.push_back(streetId);
  }
  for (auto const& [nodeId, pNode] : network.nodes()) {
    nodeIds.push_back(nodeId);
  }
  std::sort(streetIds.begin(), streetIds.end());
  std::sort(nodeIds.begin(), nodeIds.end());
  std::mt19937_64 rng{42};
  std::uniform_int_distribution<std::size_t> streetDist{0, streetIds.size() - 1};
  std::uniform_int_distribution<std::size_t> nodeDist{0, nodeIds.size() - 1};
  std::vector<std::pair<dsf::Id, dsf::Id>> queries;
  queries.reserve(nQueries);
  for (std::size_t i{0}; i < nQueries; ++i) {
    queries.emplace_back(streetIds[streetDist(rng)], nodeIds[nodeDist(rng)]);
  }
  return queries;
}
static void BM_Router_Build(benchmark::State& state) {
  auto const& network = sharedForliNetwork();
  dsf::mobility::Router router;
  for (auto _ : state) {
    router.build(network);
  }
}
static void BM_Router_UpdateCurrentWeights(benchmark::State& state) {
  auto const& network = sharedForliNetwork();
  dsf::mobility::Router router;
  router.build(network);
  for (auto _ : state) {
    router.updateCurrentWeights();
  }
}
// One query per iteration; range(0) selects the street costs (see RouteWeights; the current
// ones are the lengths), range(1) the algorithm (see RouteAlgorithm)
static void BM_Router_Route(benchmark::State& state) {
  auto const& network = sharedForliNetwork();
  dsf::mobility::Router router;
  router.setEdgeWeight("length");
  router.setAlgorithm(static_cast<dsf::mobility::RouteAlgorithm>(state.range(1)));
  router.build(network);
  auto const weights{static_cast<dsf::mobility::RouteWeights>(state.range(0))};
  // range(2): distance cost of the routes, in thousandths of s/m
  auto const distanceFactor{static_cast<double>(state.range(2)) / 1000.};
  auto const queries{routeQueries(network, 1000)};
  std::size_t i{0}, nRoutes{0}, nReachable{0};
  for (auto _ : state) {
    auto const& [streetId, nodeId] = queries[i];
    auto route{router.route(streetId, nodeId, weights, distanceFactor)};
    nReachable += route.empty() ? 0 : 1;
    ++nRoutes;
    benchmark::DoNotOptimize(route);
    i = (i + 1) % queries.size();
  }
  state.counters["reachable"] =
      static_cast<double>(nReachable) / static_cast<double>(nRoutes);
}
BENCHMARK(BM_RoadNetwork_AddNode);
BENCHMARK(BM_RoadNetwork_AddEdge);
BENCHMARK(BM_RoadNetwork_CSVImport);
BENCHMARK(BM_RoadNetwork_GeoJSONImport);
BENCHMARK(BM_RoadNetwork_NodesLooping);
BENCHMARK(BM_RoadNetwork_EdgesLooping);
BENCHMARK(BM_RoadNetwork_NodeRandomAccess);
BENCHMARK(BM_RoadNetwork_EdgeRandomAccess);
BENCHMARK(BM_Router_Build)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Router_UpdateCurrentWeights)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_Router_Route)
    ->ArgsProduct({{0, 1}, {0, 1}, {0, 100}})
    ->ArgNames({"weights", "dijkstra", "distance_mspm"})
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_RoadNetwork_CSVImport_Forli)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_RoadNetwork_AdjustNodeCapacities)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_RoadNetwork_AutoAssignRoadPriorities)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_RoadNetwork_StreetLookup)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();