#include "dsf/mobility/Street.hpp"

#include <benchmark/benchmark.h>
#include <memory>

static void BM_Street_Construction(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::Street street(
        0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, std::nullopt, 1.0);
    benchmark::DoNotOptimize(street);
  }
}

static void BM_Street_AddAgent(benchmark::State& state) {
  dsf::mobility::Street street(0,
                               {0, 1},
                               100.0,
                               13.8888888889,
                               2,
                               "test",
                               {},
                               100,  // capacity
                               1.0);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (auto _ : state) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
  }
}

static void BM_Street_Enqueue(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
  }
  size_t queueId = 0;
  for (auto _ : state) {
    if (!street.movingAgents().empty()) {
      street.enqueue(queueId);
      queueId = (queueId + 1) % 2;
    }
  }
}

static void BM_Street_Dequeue(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
    street.enqueue(0);
  }
  size_t index = 0;
  for (auto _ : state) {
    if (!street.queue(index).empty()) {
      auto agent = street.dequeue(index, spawnTime++);
      benchmark::DoNotOptimize(agent);
    }
  }
}

static void BM_Street_nAgents(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
    if (i % 2 == 0)
      street.enqueue(0);
  }
  for (auto _ : state) {
    int n = street.nAgents();
    benchmark::DoNotOptimize(n);
  }
}

static void BM_Street_Density(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
    if (i % 2 == 0)
      street.enqueue(0);
  }
  for (auto _ : state) {
    double d = street.density<false>();
    benchmark::DoNotOptimize(d);
  }
}

static void BM_Street_nMovingAgents(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
  }
  for (auto _ : state) {
    int n = street.nMovingAgents();
    benchmark::DoNotOptimize(n);
  }
}

static void BM_Street_nExitingAgents(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
    street.enqueue(0);
  }
  for (auto _ : state) {
    double n = street.nExitingAgents(dsf::Direction::ANY, false);
    benchmark::DoNotOptimize(n);
  }
}

static void BM_Street_SetLaneMapping(benchmark::State& state) {
  dsf::mobility::Street street(
      0, {0, 1}, 100.0, 13.8888888889, 3, "test", {}, std::nullopt, 1.0);
  std::vector<dsf::Direction> laneMapping = {
      dsf::Direction::RIGHTANDSTRAIGHT, dsf::Direction::STRAIGHT, dsf::Direction::LEFT};
  for (auto _ : state) {
    street.setLaneMapping(laneMapping);
  }
}

static void BM_CoilStreet_AddAgent(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  street.enableCounter();
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (auto _ : state) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
  }
}

static void BM_CoilStreet_MeanFlow(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  street.enableCounter();
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
    street.enqueue(0);
    if (i % 2 == 0) {
      auto dequeued = street.dequeue(0, spawnTime++);
    }
  }
  for (auto _ : state) {
    auto flow = street.counts();
    benchmark::DoNotOptimize(flow);
  }
}

static void BM_CoilStreet_Dequeue(benchmark::State& state) {
  dsf::mobility::Street street(0, {0, 1}, 100.0, 13.8888888889, 2, "test", {}, 100, 1.0);
  street.enableCounter();
  std::time_t spawnTime = 0;
  auto pItinerary = std::make_shared<dsf::mobility::Itinerary>(1, 1);
  for (int i = 0; i < 50; ++i) {
    auto agent = std::make_unique<dsf::mobility::Agent>(0, spawnTime++, pItinerary, 0);
    street.addAgent(std::move(agent), spawnTime);
    street.enqueue(0);
  }
  size_t index = 0;
  for (auto _ : state) {
    if (!street.queue(index).empty()) {
      auto agent = street.dequeue(index, spawnTime++);
      benchmark::DoNotOptimize(agent);
    }
  }
}

BENCHMARK(BM_Street_Construction);
BENCHMARK(BM_Street_AddAgent);
BENCHMARK(BM_Street_Enqueue);
BENCHMARK(BM_Street_Dequeue);
BENCHMARK(BM_Street_nAgents);
BENCHMARK(BM_Street_Density);
BENCHMARK(BM_Street_nMovingAgents);
BENCHMARK(BM_Street_nExitingAgents);
BENCHMARK(BM_Street_SetLaneMapping);
BENCHMARK(BM_CoilStreet_AddAgent);
BENCHMARK(BM_CoilStreet_MeanFlow);
BENCHMARK(BM_CoilStreet_Dequeue);

BENCHMARK_MAIN();
