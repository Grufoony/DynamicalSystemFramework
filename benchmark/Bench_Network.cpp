#include "dsf/mobility/RoadNetwork.hpp"

#include <filesystem>

#include <benchmark/benchmark.h>

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "test/data";

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
  dsf::mobility::RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  for (auto _ : state) {
    for (auto const& [id, node] : network.nodes()) {
      benchmark::DoNotOptimize(id);
      benchmark::DoNotOptimize(node);
    }
  }
}
static void BM_RoadNetwork_EdgesLooping(benchmark::State& state) {
  dsf::mobility::RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  for (auto _ : state) {
    for (auto const& [id, edge] : network.edges()) {
      benchmark::DoNotOptimize(id);
      benchmark::DoNotOptimize(edge);
    }
  }
}
static void BM_RoadNetwork_AllPathsTo(benchmark::State& state) {
  dsf::mobility::RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  auto itNode = network.nodes().cbegin();
  for (auto _ : state) {
    auto paths =
        network.allPathsTo(itNode->first, [](auto const& edge) { return edge.length(); });
    ++itNode;
  }
}
static void BM_RoadNetwork_ShortestPath(benchmark::State& state) {
  dsf::mobility::RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  auto itSource = network.nodes().cbegin();
  auto itTarget = std::next(network.nodes().cbegin(), network.nodes().size() / 2);
  for (auto _ : state) {
    auto path = network.shortestPath(
        itSource->first, itTarget->first, [](auto const& edge) { return edge.length(); });
    benchmark::DoNotOptimize(path);
    ++itSource;
    ++itTarget;
    if (itTarget == network.nodes().cend()) {
      itTarget = network.nodes().cbegin();
    }
  }
}
BENCHMARK(BM_RoadNetwork_AddNode);
BENCHMARK(BM_RoadNetwork_AddEdge);
BENCHMARK(BM_RoadNetwork_CSVImport);
BENCHMARK(BM_RoadNetwork_GeoJSONImport);
BENCHMARK(BM_RoadNetwork_NodesLooping);
BENCHMARK(BM_RoadNetwork_EdgesLooping);
BENCHMARK(BM_RoadNetwork_ShortestPath);
BENCHMARK(BM_RoadNetwork_AllPathsTo);

BENCHMARK_MAIN();