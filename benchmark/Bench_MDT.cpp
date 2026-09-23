#include "dsf/mdt/TrajectoryCollection.hpp"

#include <random>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <benchmark/benchmark.h>
#include <spdlog/spdlog.h>

using Dataframe = std::unordered_map<
    std::string,
    std::variant<std::vector<dsf::Id>, std::vector<std::time_t>, std::vector<double>>>;

[[maybe_unused]] static const bool SPDLOG_SILENCED = [] {
  spdlog::set_level(spdlog::level::off);
  return true;
}();

static constexpr std::size_t POINTS_PER_USER{500};

// Build a synthetic dataframe of nUsers users with POINTS_PER_USER GPS points each,
// sampled every minute around Forlì. Each user alternates stops (points jittered
// within a few meters) and trips (points moving at ~30 km/h), so that filtering finds
// both stop clusters and moving points.
static Dataframe makeDataframe(std::size_t nUsers) {
  std::mt19937 generator{42};
  std::uniform_real_distribution<double> lonDist{11.95, 12.10}, latDist{44.18, 44.26};
  std::normal_distribution<double> jitterDist{0., 2e-5};
  std::uniform_real_distribution<double> directionDist{-1., 1.};
  std::vector<dsf::Id> uids;
  std::vector<std::time_t> timestamps;
  std::vector<double> lats, lons;
  auto const nPoints{nUsers * POINTS_PER_USER};
  uids.reserve(nPoints);
  timestamps.reserve(nPoints);
  lats.reserve(nPoints);
  lons.reserve(nPoints);
  for (dsf::Id uid = 0; uid < nUsers; ++uid) {
    double lon{lonDist(generator)}, lat{latDist(generator)};
    double dLon{0.}, dLat{0.};
    for (std::size_t i = 0; i < POINTS_PER_USER; ++i) {
      // Switch between a 20-minute stop and a 20-minute trip
      if (i % 20 == 0) {
        bool const bMoving{(i / 20) % 2 == 1};
        // ~500 m per minute when moving
        dLon = bMoving ? 6e-3 * directionDist(generator) : 0.;
        dLat = bMoving ? 4.5e-3 * directionDist(generator) : 0.;
      }
      lon += dLon;
      lat += dLat;
      uids.push_back(uid);
      timestamps.push_back(static_cast<std::time_t>(1700000000 + 60 * i));
      lons.push_back(lon + jitterDist(generator));
      lats.push_back(lat + jitterDist(generator));
    }
  }
  return Dataframe{{"uid", std::move(uids)},
                   {"timestamp", std::move(timestamps)},
                   {"lat", std::move(lats)},
                   {"lon", std::move(lons)}};
}

static void BM_TrajectoryCollection_FromDataframe(benchmark::State& state) {
  auto const nUsers{static_cast<std::size_t>(state.range(0))};
  auto const dataframe = makeDataframe(nUsers);
  for (auto _ : state) {
    state.PauseTiming();
    auto copy = dataframe;
    state.ResumeTiming();
    dsf::mdt::TrajectoryCollection collection(std::move(copy));
    benchmark::DoNotOptimize(collection.trajectories().size());
  }
  state.SetItemsProcessed(state.iterations() * nUsers * POINTS_PER_USER);
}

static void BM_TrajectoryCollection_Filter(benchmark::State& state) {
  auto const nUsers{static_cast<std::size_t>(state.range(0))};
  dsf::mdt::TrajectoryCollection const collection(makeDataframe(nUsers));
  std::size_t nTrajectories{0};
  for (auto _ : state) {
    state.PauseTiming();
    auto copy = collection;
    state.ResumeTiming();
    copy.filter(0.1, 150., 2, 5);
    state.PauseTiming();
    // Sanity check: the synthetic stops must split users into several trajectories
    nTrajectories = 0;
    for (auto const& [uid, trajectories] : copy.trajectories()) {
      nTrajectories += trajectories.size();
    }
    state.ResumeTiming();
  }
  state.counters["trajectories"] = static_cast<double>(nTrajectories);
  state.SetItemsProcessed(state.iterations() * nUsers * POINTS_PER_USER);
}

BENCHMARK(BM_TrajectoryCollection_FromDataframe)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMillisecond);
BENCHMARK(BM_TrajectoryCollection_Filter)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
