#include "dsf/mobility/TrafficLight.hpp"
#include "dsf/geometry/Point.hpp"

#include <benchmark/benchmark.h>
#include <memory>

static void BM_TrafficLight_Construction(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::TrafficLight tl(0);
    benchmark::DoNotOptimize(tl);
  }
}

static void BM_TrafficLight_ConstructionWithPoint(benchmark::State& state) {
  dsf::geometry::Point point{0.0, 0.0};
  for (auto _ : state) {
    dsf::mobility::TrafficLight tl(0, point);
    benchmark::DoNotOptimize(tl);
  }
}

static void BM_TrafficLight_OperatorIncrement(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0);
  // ensure at least one phase so operator++ cycles
  tl.addPhase(dsf::mobility::TrafficLightPhase{1});
  for (auto _ : state) {
    ++tl;
    benchmark::DoNotOptimize(tl);
  }
}

static void BM_TrafficLight_SetCycle(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0);
  dsf::mobility::TrafficLightPhase phase(30);
  phase.addGreen(1, dsf::Direction::STRAIGHT);
  for (auto _ : state) {
    // replace phases to simulate setting a cycle
    tl.setPhases({phase});
  }
}

static void BM_TrafficLight_SetComplementaryCycle(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::TrafficLight tl(0);
    dsf::mobility::TrafficLightPhase p1(30);
    p1.addGreen(1, dsf::Direction::STRAIGHT);
    dsf::mobility::TrafficLightPhase p2(30);
    p2.addGreen(2, dsf::Direction::LEFT);
    tl.setPhases({p1, p2});
  }
}

static void BM_TrafficLight_IsGreen(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0);
  dsf::mobility::TrafficLightPhase cycle(30);
  cycle.addGreen(1, dsf::Direction::STRAIGHT);
  tl.setPhases({cycle});
  for (auto _ : state) {
    bool green = tl.isGreen(1, dsf::Direction::STRAIGHT);
    benchmark::DoNotOptimize(green);
  }
}

static void BM_TrafficLight_MeanGreenTime(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0);
  dsf::mobility::TrafficLightPhase cycle1(30);
  cycle1.addGreen(1, dsf::Direction::STRAIGHT);
  dsf::mobility::TrafficLightPhase cycle2(20);
  cycle2.addGreen(2, dsf::Direction::LEFT);
  tl.setPhases({cycle1, cycle2});
  for (auto _ : state) {
    double mean = tl.meanGreenTime(false);
    benchmark::DoNotOptimize(mean);
  }
}

static void BM_TrafficLight_ResetCycles(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0);
  dsf::mobility::TrafficLightPhase cycle(30);
  cycle.addGreen(1, dsf::Direction::STRAIGHT);
  tl.setPhases({cycle});
  for (auto _ : state) {
    tl.reset();
  }
}

static void BM_TrafficLight_IncreasePhases(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0);
  dsf::mobility::TrafficLightPhase cycle(30);
  cycle.addGreen(1, dsf::Direction::STRAIGHT);
  for (auto _ : state) {
    // simulate increasing phases by appending phases
    tl.addPhase(cycle);
  }
}

BENCHMARK(BM_TrafficLight_Construction);
BENCHMARK(BM_TrafficLight_ConstructionWithPoint);
BENCHMARK(BM_TrafficLight_OperatorIncrement);
BENCHMARK(BM_TrafficLight_SetCycle);
BENCHMARK(BM_TrafficLight_SetComplementaryCycle);
BENCHMARK(BM_TrafficLight_IsGreen);
BENCHMARK(BM_TrafficLight_MeanGreenTime);
BENCHMARK(BM_TrafficLight_ResetCycles);
BENCHMARK(BM_TrafficLight_IncreasePhases);

BENCHMARK_MAIN();