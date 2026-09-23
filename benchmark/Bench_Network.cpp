#include "dsf/mobility/RoadNetwork.hpp"

#include <algorithm>
#include <filesystem>
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
  static auto network = [] {
    auto network = makeForliNetwork();
    network.setEdgeWeight("length");
    return network;
  }();
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
  dsf::Id source{0}, target{1};
  for (auto _ : state) {
    network.addEdge(source, std::make_pair(source++, target++));
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
      benchmark::DoNotOptimize(static_cast<dsf::Id>(id));
      benchmark::DoNotOptimize(node.get());
    }
  }
}
static void BM_RoadNetwork_EdgesLooping(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  for (auto _ : state) {
    for (auto const& [id, edge] : network.edges()) {
      benchmark::DoNotOptimize(static_cast<dsf::Id>(id));
      benchmark::DoNotOptimize(edge.get());
    }
  }
}
static void BM_RoadNetwork_NodeRandomAccess(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  auto itNode = network.nodes().cbegin();
  for (auto _ : state) {
    benchmark::DoNotOptimize(network.node(itNode->first));
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
    benchmark::DoNotOptimize(network.edge(itEdge->first));
    ++itEdge;
    if (itEdge == network.edges().cend()) {
      itEdge = network.edges().cbegin();
    }
  }
}
static void BM_RoadNetwork_AllPathsTo(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  auto itNode = network.nodes().cbegin();
  for (auto _ : state) {
    auto paths = network.allPathsTo(itNode->first);
    ++itNode;
    if (itNode == network.nodes().cend()) {
      itNode = network.nodes().cbegin();
    }
  }
}
static void BM_RoadNetwork_AllEdgePathsTo(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  auto itEdge = network.edges().cbegin();
  for (auto _ : state) {
    auto paths = network.allEdgePathsTo(itEdge->first);
    ++itEdge;
    if (itEdge == network.edges().cend()) {
      itEdge = network.edges().cbegin();
    }
  }
}
static void BM_RoadNetwork_ShortestPath(benchmark::State& state) {
  auto& network = sharedForliNetwork();
  auto itSource = network.nodes().cbegin();
  auto itTarget = std::next(network.nodes().cbegin(), network.nodes().size() / 2);
  for (auto _ : state) {
    auto path = network.shortestPath(itSource->first, itTarget->first);
    benchmark::DoNotOptimize(path);
    ++itSource;
    ++itTarget;
    if (itTarget == network.nodes().cend()) {
      itTarget = network.nodes().cbegin();
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
    endpoints.emplace_back(pEdge->source(), pEdge->target());
  }
  std::sort(endpoints.begin(), endpoints.end());
  std::size_t i{0};
  for (auto _ : state) {
    auto const& [source, target] = endpoints[i];
    benchmark::DoNotOptimize(network.street(source, target));
    i = (i + 997) % endpoints.size();
  }
}
BENCHMARK(BM_RoadNetwork_AddNode);
BENCHMARK(BM_RoadNetwork_AddEdge);
BENCHMARK(BM_RoadNetwork_CSVImport);
BENCHMARK(BM_RoadNetwork_GeoJSONImport);
BENCHMARK(BM_RoadNetwork_NodesLooping);
BENCHMARK(BM_RoadNetwork_EdgesLooping);
BENCHMARK(BM_RoadNetwork_NodeRandomAccess);
BENCHMARK(BM_RoadNetwork_EdgeRandomAccess);
BENCHMARK(BM_RoadNetwork_ShortestPath);
BENCHMARK(BM_RoadNetwork_AllPathsTo);
BENCHMARK(BM_RoadNetwork_AllEdgePathsTo);
BENCHMARK(BM_RoadNetwork_CSVImport_Forli)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_RoadNetwork_AdjustNodeCapacities)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_RoadNetwork_AutoAssignRoadPriorities)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_RoadNetwork_StreetLookup)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();