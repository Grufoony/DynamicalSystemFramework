#include "dsf/mobility/Intersection.hpp"
#include "dsf/geometry/Point.hpp"

#include <benchmark/benchmark.h>
#include <memory>

static void BM_Intersection_Construction(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::Intersection intersection(0);
    benchmark::DoNotOptimize(intersection);
  }
}

static void BM_Intersection_ConstructionWithPoint(benchmark::State& state) {
  dsf::geometry::Point point{0.0, 0.0};
  for (auto _ : state) {
    dsf::mobility::Intersection intersection(0, point);
    benchmark::DoNotOptimize(intersection);
  }
}

static void BM_Intersection_AddAgentWithAngle(benchmark::State& state) {
  std::time_t spawnTime = 0;
  for (auto _ : state) {
    dsf::mobility::Intersection intersection(0);
    intersection.setCapacity(100);
    auto agent = std::make_unique<dsf::mobility::Agent>(
        0, spawnTime++, std::make_shared<dsf::mobility::Itinerary>(1, 1), 0);
    intersection.addAgent(0.0, std::move(agent));
  }
}

static void BM_Intersection_AddAgentWithoutAngle(benchmark::State& state) {
  std::time_t spawnTime = 0;
  for (auto _ : state) {
    dsf::mobility::Intersection intersection(0);
    intersection.setCapacity(100);
    auto agent = std::make_unique<dsf::mobility::Agent>(
        0, spawnTime++, std::make_shared<dsf::mobility::Itinerary>(1, 1), 0);
    intersection.addAgent(std::move(agent));
  }
}

static void BM_Intersection_nAgents(benchmark::State& state) {
  dsf::mobility::Intersection intersection(0);
  intersection.setCapacity(1000);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 100; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    intersection.addAgent(std::move(agent));
  }
  for (auto _ : state) {
    auto n = intersection.nAgents();
    benchmark::DoNotOptimize(n);
  }
}

static void BM_Intersection_Density(benchmark::State& state) {
  dsf::mobility::Intersection intersection(0);
  intersection.setCapacity(1000);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 100; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    intersection.addAgent(std::move(agent));
  }
  for (auto _ : state) {
    double d = intersection.density();
    benchmark::DoNotOptimize(d);
  }
}

static void BM_Intersection_IsFull(benchmark::State& state) {
  dsf::mobility::Intersection intersection(0);
  intersection.setCapacity(1000);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 100; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    intersection.addAgent(std::move(agent));
  }
  for (auto _ : state) {
    bool full = intersection.isFull();
    benchmark::DoNotOptimize(full);
  }
}

static void BM_Intersection_SetStreetPriorities(benchmark::State& state) {
  dsf::mobility::Intersection intersection(0);
  std::set<dsf::Id> priorities = {1, 2, 3};
  for (auto _ : state) {
    intersection.setStreetPriorities(priorities);
  }
}

static void BM_Intersection_AddStreetPriority(benchmark::State& state) {
  dsf::mobility::Intersection intersection(0);
  // Need to add ingoing edges first
  dsf::mobility::Intersection source1{1};
  dsf::mobility::Intersection source2{2};
  dsf::mobility::Street street1{1, {&source1, &intersection}};
  dsf::mobility::Street street2{2, {&source2, &intersection}};
  intersection.addIngoingEdge(&street1);
  intersection.addIngoingEdge(&street2);
  for (auto _ : state) {
    intersection.addStreetPriority(1);
    intersection.addStreetPriority(2);
  }
}

BENCHMARK(BM_Intersection_Construction);
BENCHMARK(BM_Intersection_ConstructionWithPoint);
BENCHMARK(BM_Intersection_AddAgentWithAngle);
BENCHMARK(BM_Intersection_AddAgentWithoutAngle);
BENCHMARK(BM_Intersection_nAgents);
BENCHMARK(BM_Intersection_Density);
BENCHMARK(BM_Intersection_IsFull);
BENCHMARK(BM_Intersection_SetStreetPriorities);
BENCHMARK(BM_Intersection_AddStreetPriority);

BENCHMARK_MAIN();