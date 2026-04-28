#include "dsf/mobility/FirstOrderDynamics.hpp"

#include <filesystem>

#include <benchmark/benchmark.h>

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "test/data";

static void BM_FirstOrderDynamics_Empty_Evolve(benchmark::State& state) {
  dsf::mobility::RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  dsf::mobility::FirstOrderDynamics dynamics(std::move(network));
  for (auto _ : state) {
    dynamics.evolve();
  }
}

BENCHMARK(BM_FirstOrderDynamics_Empty_Evolve);

BENCHMARK_MAIN();