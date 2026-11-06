#include "dsf/mobility/Agent.hpp"

#include <benchmark/benchmark.h>
#include <memory>
#include <numeric>
#include <vector>

static void BM_Agent_ConstructionWithDestination(benchmark::State& state) {
  std::time_t spawnTime = 0;
  for (auto _ : state) {
    dsf::mobility::Agent agent(0, spawnTime++, dsf::Id{1}, 0);
    benchmark::DoNotOptimize(agent);
  }
}

static void BM_Agent_ConstructionRandom(benchmark::State& state) {
  std::time_t spawnTime = 0;
  for (auto _ : state) {
    dsf::mobility::Agent agent(0, spawnTime++);
    benchmark::DoNotOptimize(agent);
  }
}

static void BM_Agent_SetSrcNodeId(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  for (auto _ : state) {
    agent.setSrcNodeId(5);
  }
}

static void BM_Agent_SetStreetId(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  for (auto _ : state) {
    agent.setStreetId(10);
  }
}

static void BM_Agent_SetNextStreetId(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  for (auto _ : state) {
    agent.setNextStreetId(15);
  }
}

static void BM_Agent_SetSpeed(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  for (auto _ : state) {
    agent.setSpeed(50.0);
  }
}

static void BM_Agent_SetFreeTime(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  std::time_t freeTime = 100;
  for (auto _ : state) {
    agent.setFreeTime(freeTime++);
  }
}

static void BM_Agent_IncrementDistance(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  for (auto _ : state) {
    agent.incrementDistance(10.0);
  }
}

static void BM_Agent_Reset(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  agent.setSpeed(50.0);
  agent.setStreetId(10);
  std::time_t spawnTime = 1000;
  for (auto _ : state) {
    agent.reset(spawnTime++);
  }
}

// Getter benchmarks - these are inline so very fast
static void BM_Agent_Getters(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  agent.setSpeed(50.0);
  agent.setStreetId(10);
  for (auto _ : state) {
    auto spawnTime = agent.spawnTime();
    auto freeTime = agent.freeTime();
    auto id = agent.id();
    auto streetId = agent.streetId();
    auto srcNodeId = agent.srcNodeId();
    auto nextStreetId = agent.nextStreetId();
    auto speed = agent.speed();
    auto distance = agent.distance();
    auto isRandom = agent.isRandom();
    benchmark::DoNotOptimize(spawnTime);
    benchmark::DoNotOptimize(freeTime);
    benchmark::DoNotOptimize(id);
    benchmark::DoNotOptimize(streetId);
    benchmark::DoNotOptimize(srcNodeId);
    benchmark::DoNotOptimize(nextStreetId);
    benchmark::DoNotOptimize(speed);
    benchmark::DoNotOptimize(distance);
    benchmark::DoNotOptimize(isRandom);
  }
}

static void BM_Agent_Destination(benchmark::State& state) {
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  for (auto _ : state) {
    benchmark::DoNotOptimize(agent.destination());
  }
}

// Plan a route of state.range(0) streets, then take them one by one, as an agent does
static void BM_Agent_Path(benchmark::State& state) {
  std::vector<dsf::Id> route(static_cast<std::size_t>(state.range(0)));
  std::iota(route.begin(), route.end(), dsf::Id{2});
  dsf::mobility::Agent agent(0, 0, dsf::Id{1}, 0);
  for (auto _ : state) {
    agent.setPath(route);
    while (agent.nextPlannedStreetId().has_value()) {
      agent.advancePath();
    }
    benchmark::DoNotOptimize(agent.destination());
  }
}

BENCHMARK(BM_Agent_ConstructionWithDestination);
BENCHMARK(BM_Agent_ConstructionRandom);
BENCHMARK(BM_Agent_SetSrcNodeId);
BENCHMARK(BM_Agent_SetStreetId);
BENCHMARK(BM_Agent_SetNextStreetId);
BENCHMARK(BM_Agent_SetSpeed);
BENCHMARK(BM_Agent_SetFreeTime);
BENCHMARK(BM_Agent_IncrementDistance);
BENCHMARK(BM_Agent_Reset);
BENCHMARK(BM_Agent_Getters);
BENCHMARK(BM_Agent_Destination);
BENCHMARK(BM_Agent_Path)->Arg(10)->Arg(50)->Arg(200);

BENCHMARK_MAIN();