#include "dsf/mobility/Roundabout.hpp"
#include "dsf/geometry/Point.hpp"

#include <benchmark/benchmark.h>
#include <memory>

static void BM_Roundabout_Construction(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::Roundabout roundabout(0);
    benchmark::DoNotOptimize(roundabout);
  }
}

static void BM_Roundabout_ConstructionWithPoint(benchmark::State& state) {
  dsf::geometry::Point point{0.0, 0.0};
  for (auto _ : state) {
    dsf::mobility::Roundabout roundabout(0, point);
    benchmark::DoNotOptimize(roundabout);
  }
}

static void BM_Roundabout_Enqueue(benchmark::State& state) {
  dsf::mobility::Roundabout roundabout(0);
  roundabout.setCapacity(1000);
  std::time_t spawnTime = 0;
  std::optional<dsf::Id> const destination{1};
  for (auto _ : state) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, destination, 0);
    roundabout.enqueue(std::move(agent));
  }
}

static void BM_Roundabout_Dequeue(benchmark::State& state) {
  std::time_t spawnTime = 0;
  for (auto _ : state) {
    dsf::mobility::Roundabout roundabout(0);
    roundabout.setCapacity(100);
    auto agent = std::make_unique<dsf::mobility::Agent>(
        0, spawnTime++, dsf::Id{1}, 0);
    roundabout.enqueue(std::move(agent));
    auto dequeued = roundabout.dequeue();
    benchmark::DoNotOptimize(dequeued);
  }
}

static void BM_Roundabout_Density(benchmark::State& state) {
  dsf::mobility::Roundabout roundabout(0);
  roundabout.setCapacity(1000);
  std::time_t spawnTime = 0;
  std::optional<dsf::Id> const destination{1};
  for (int i = 0; i < 100; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, destination, 0);
    roundabout.enqueue(std::move(agent));
  }
  for (auto _ : state) {
    double d = roundabout.density();
    benchmark::DoNotOptimize(d);
  }
}

static void BM_Roundabout_IsFull(benchmark::State& state) {
  dsf::mobility::Roundabout roundabout(0);
  roundabout.setCapacity(1000);
  std::time_t spawnTime = 0;
  std::optional<dsf::Id> const destination{1};
  for (int i = 0; i < 100; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, destination, 0);
    roundabout.enqueue(std::move(agent));
  }
  for (auto _ : state) {
    bool full = roundabout.isFull();
    benchmark::DoNotOptimize(full);
  }
}

BENCHMARK(BM_Roundabout_Construction);
BENCHMARK(BM_Roundabout_ConstructionWithPoint);
BENCHMARK(BM_Roundabout_Enqueue);
BENCHMARK(BM_Roundabout_Dequeue);
BENCHMARK(BM_Roundabout_Density);
BENCHMARK(BM_Roundabout_IsFull);

BENCHMARK_MAIN();