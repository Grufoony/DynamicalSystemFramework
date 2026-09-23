#include "dsf/mobility/FirstOrderDynamics.hpp"

#include <algorithm>
#include <filesystem>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <benchmark/benchmark.h>
#include <spdlog/spdlog.h>

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "test/data";

[[maybe_unused]] static const bool SPDLOG_SILENCED = [] {
  spdlog::set_level(spdlog::level::off);
  return true;
}();

using dsf::mobility::AgentInsertionMethod;
using dsf::mobility::FirstOrderDynamics;
using dsf::mobility::RoadNetwork;

static constexpr unsigned int SEED{42};
// Number of steps run before timing a loaded scenario, to reach a steady state
static constexpr std::size_t WARMUP_STEPS{300};

static RoadNetwork makeForliNetwork() {
  RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  return network;
}

static std::unique_ptr<FirstOrderDynamics> makeForliDynamics() {
  auto pDynamics = std::make_unique<FirstOrderDynamics>(makeForliNetwork(), false, SEED);
  pDynamics->prepareNetwork();
  pDynamics->setUpdatePathsThrowOnEmpty(false);
  return pDynamics;
}

// Pick n street ids evenly spaced among the sorted ids, each with unit weight, so that
// the selection is deterministic regardless of the edges' container order
static std::unordered_map<dsf::Id, double> evenlySpacedStreets(
    FirstOrderDynamics const& dynamics, std::size_t n, std::size_t offset = 0) {
  std::vector<dsf::Id> streetIds;
  streetIds.reserve(dynamics.graph().nEdges());
  for (auto const& [streetId, pStreet] : dynamics.graph().edges()) {
    streetIds.push_back(streetId);
  }
  std::sort(streetIds.begin(), streetIds.end());
  auto const step{std::max<std::size_t>(1, streetIds.size() / n)};
  std::unordered_map<dsf::Id, double> streets;
  for (std::size_t i = offset % step; i < streetIds.size() && streets.size() < n;
       i += step) {
    streets.emplace(streetIds[i], 1.);
  }
  return streets;
}

// Agents traveling between 5% of the streets (as origins) and 100 destinations
static std::unique_ptr<FirstOrderDynamics> makeODDynamics() {
  auto pDynamics = makeForliDynamics();
  auto const nOrigins{pDynamics->graph().nEdges() / 20};
  pDynamics->setOrigins(evenlySpacedStreets(*pDynamics, nOrigins));
  pDynamics->setDestinations(evenlySpacedStreets(*pDynamics, 100, 1));
  pDynamics->updatePaths();
  return pDynamics;
}

// A dynamics whose agents are topped up at every step, like TrafficSimulator does, so
// that the load stays constant.
// NOTE: FirstOrderDynamics::setReinsertAgents(true) is not used on purpose, since
// reinserting agents during the parallel street evolution is not thread-safe.
struct Scenario {
  std::unique_ptr<FirstOrderDynamics> pDynamics;
  std::size_t nAgents;
  AgentInsertionMethod insertionMethod;

  void step(dsf::mobility::StepDataRequest const& request = {}) {
    auto const nCurrentAgents{pDynamics->nAgents()};
    if (nCurrentAgents < nAgents) {
      pDynamics->addAgents(nAgents - nCurrentAgents, insertionMethod);
    }
    auto result = pDynamics->evolve(request);
    benchmark::DoNotOptimize(result);
  }
};

static Scenario makeODScenario(std::size_t nAgents) {
  return Scenario{makeODDynamics(), nAgents, AgentInsertionMethod::RANDOM_ODS};
}

// Random agents (no itinerary), traveling for ~5 km on average
static Scenario makeRandomScenario(std::size_t nAgents) {
  auto pDynamics = makeForliDynamics();
  pDynamics->setMeanTravelDistance(5000.);
  return Scenario{std::move(pDynamics), nAgents, AgentInsertionMethod::RANDOM};
}

// Scenarios are expensive to build, while Google Benchmark calls each benchmark
// function several times (repetitions, iteration estimation). Build each scenario once,
// warm it up and keep evolving it: its load is stationary.
using ScenarioCache = std::map<std::int64_t, Scenario>;

template <typename TFactory>
static Scenario& cachedScenario(ScenarioCache& cache,
                                std::int64_t key,
                                TFactory&& factory) {
  auto it = cache.find(key);
  if (it == cache.end()) {
    it = cache.emplace(key, factory()).first;
    for (std::size_t i = 0; i < WARMUP_STEPS; ++i) {
      it->second.step();
    }
  }
  return it->second;
}

static void setAgentCounters(benchmark::State& state, Scenario const& scenario) {
  state.counters["agents"] = static_cast<double>(scenario.pDynamics->nAgents());
}

static void BM_FirstOrderDynamics_Empty_Evolve(benchmark::State& state) {
  dsf::mobility::RoadNetwork network;
  network.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
  network.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
  dsf::mobility::FirstOrderDynamics dynamics(std::move(network));
  for (auto _ : state) {
    dynamics.evolve();
  }
}

static void BM_FirstOrderDynamics_UpdatePaths(benchmark::State& state) {
  static std::map<std::int64_t, std::unique_ptr<FirstOrderDynamics>> cache;
  auto& pDynamics = cache[state.range(0)];
  if (!pDynamics) {
    pDynamics = makeForliDynamics();
    pDynamics->setDestinations(
        evenlySpacedStreets(*pDynamics, static_cast<std::size_t>(state.range(0))));
    // The first call drops the unreachable destinations: keep it out of the timing
    pDynamics->updatePaths();
  }
  for (auto _ : state) {
    pDynamics->updatePaths();
  }
  state.counters["itineraries"] = static_cast<double>(pDynamics->itineraries().size());
}

static void BM_FirstOrderDynamics_AddAgents_RandomODs(benchmark::State& state) {
  static auto pDynamics = [] {
    auto pDynamics = makeODDynamics();
    // Agents that are not inserted into the network are dropped at the next call, so
    // that agents do not pile up across iterations
    pDynamics->killStagnantAgents();
    return pDynamics;
  }();
  auto const nAgents{static_cast<std::size_t>(state.range(0))};
  for (auto _ : state) {
    pDynamics->addAgents(nAgents, AgentInsertionMethod::RANDOM_ODS);
  }
  state.SetItemsProcessed(state.iterations() * state.range(0));
}

static void BM_FirstOrderDynamics_Evolve_ODs(benchmark::State& state) {
  static ScenarioCache cache;
  auto& scenario = cachedScenario(cache, state.range(0), [&state] {
    return makeODScenario(static_cast<std::size_t>(state.range(0)));
  });
  for (auto _ : state) {
    scenario.step();
  }
  setAgentCounters(state, scenario);
}

static void BM_FirstOrderDynamics_Evolve_Random(benchmark::State& state) {
  static ScenarioCache cache;
  auto& scenario = cachedScenario(cache, state.range(0), [&state] {
    return makeRandomScenario(static_cast<std::size_t>(state.range(0)));
  });
  for (auto _ : state) {
    scenario.step();
  }
  setAgentCounters(state, scenario);
}

static void BM_FirstOrderDynamics_Evolve_TransitionMatrix(benchmark::State& state) {
  static ScenarioCache cache;
  auto& scenario = cachedScenario(cache, state.range(0), [&state] {
    auto scenario = makeRandomScenario(static_cast<std::size_t>(state.range(0)));
    // Uniform transition matrix over each street's possible next streets, with a 10%
    // probability of ending the trip
    auto const& graph = scenario.pDynamics->graph();
    std::unordered_map<dsf::Id, std::unordered_map<dsf::Id, double>> transitionMatrix;
    for (auto const& [streetId, pStreet] : graph.edges()) {
      auto const& nextStreets = graph.node(pStreet->target()).outgoingEdges();
      if (nextStreets.empty()) {
        continue;
      }
      auto& row = transitionMatrix[streetId];
      for (auto const& nextStreetId : nextStreets) {
        row[nextStreetId] = 0.9 / static_cast<double>(nextStreets.size());
      }
    }
    scenario.pDynamics->setTransitionMatrix(transitionMatrix);
    return scenario;
  });
  for (auto _ : state) {
    scenario.step();
  }
  setAgentCounters(state, scenario);
}

// Same as Evolve_ODs/5000, which uses the default LINEAR speed function
static void BM_FirstOrderDynamics_Evolve_ConstantSpeed(benchmark::State& state) {
  static ScenarioCache cache;
  auto& scenario = cachedScenario(cache, state.range(0), [&state] {
    auto scenario = makeODScenario(static_cast<std::size_t>(state.range(0)));
    scenario.pDynamics->setSpeedFunction(dsf::SpeedFunction::CONSTANT);
    return scenario;
  });
  for (auto _ : state) {
    scenario.step();
  }
  setAgentCounters(state, scenario);
}

// Same as Evolve_ODs/5000, collecting the step data. Agent data is left out, since
// Street::acquireAgentData() enables it globally, which would affect the other
// benchmarks of this executable.
static void BM_FirstOrderDynamics_Evolve_DataCollection(benchmark::State& state) {
  static ScenarioCache cache;
  auto& scenario = cachedScenario(cache, state.range(0), [&state] {
    return makeODScenario(static_cast<std::size_t>(state.range(0)));
  });
  dsf::mobility::StepDataRequest const request{
      .saveAverageStats = true, .saveStreetData = true, .saveTravelData = true};
  for (auto _ : state) {
    scenario.step(request);
  }
  setAgentCounters(state, scenario);
}

BENCHMARK(BM_FirstOrderDynamics_Empty_Evolve);
BENCHMARK(BM_FirstOrderDynamics_UpdatePaths)
    ->ArgName("destinations")
    ->Arg(10)
    ->Arg(100)
    ->Unit(benchmark::kMillisecond);
BENCHMARK(BM_FirstOrderDynamics_AddAgents_RandomODs)
    ->ArgName("agents")
    ->Arg(1000)
    ->Arg(10000)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_FirstOrderDynamics_Evolve_ODs)
    ->ArgName("agents")
    ->Arg(1000)
    ->Arg(5000)
    ->Arg(10000)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_FirstOrderDynamics_Evolve_Random)
    ->ArgName("agents")
    ->Arg(5000)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_FirstOrderDynamics_Evolve_TransitionMatrix)
    ->ArgName("agents")
    ->Arg(5000)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_FirstOrderDynamics_Evolve_ConstantSpeed)
    ->ArgName("agents")
    ->Arg(5000)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_FirstOrderDynamics_Evolve_DataCollection)
    ->ArgName("agents")
    ->Arg(5000)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
