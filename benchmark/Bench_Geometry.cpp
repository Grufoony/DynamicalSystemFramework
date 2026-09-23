#include "dsf/geometry/Point.hpp"
#include "dsf/geometry/PolyLine.hpp"

#include <format>
#include <random>
#include <string>
#include <vector>

#include <benchmark/benchmark.h>

// Build a WKT LINESTRING with nPoints points around Forlì, similar to the geometries
// found in the edges' CSV files
static std::string makeWKTLineString(std::size_t nPoints) {
  std::mt19937 generator{42};
  std::uniform_real_distribution<double> stepDist{-1e-4, 1e-4};
  double lon{12.0412345678901}, lat{44.2212345678901};
  std::string wkt{"LINESTRING ("};
  for (std::size_t i = 0; i < nPoints; ++i) {
    if (i > 0) {
      wkt += ", ";
    }
    wkt += std::format("{:.15f} {:.15f}", lon, lat);
    lon += stepDist(generator);
    lat += stepDist(generator);
  }
  wkt += ")";
  return wkt;
}

static void BM_Point_ParseWKT(benchmark::State& state) {
  std::string const wkt{"POINT (12.0412345678901 44.2212345678901)"};
  for (auto _ : state) {
    dsf::geometry::Point point(wkt);
    benchmark::DoNotOptimize(point);
  }
}

static void BM_Point_Haversine(benchmark::State& state) {
  std::mt19937 generator{42};
  std::uniform_real_distribution<double> lonDist{11.9, 12.2}, latDist{44.1, 44.3};
  std::vector<dsf::geometry::Point> points;
  points.reserve(1024);
  for (std::size_t i = 0; i < 1024; ++i) {
    points.emplace_back(lonDist(generator), latDist(generator));
  }
  std::size_t i{0};
  for (auto _ : state) {
    auto const& p1 = points[i % points.size()];
    auto const& p2 = points[(i + 1) % points.size()];
    benchmark::DoNotOptimize(dsf::geometry::haversine_km(p1, p2));
    ++i;
  }
}

static void BM_PolyLine_ParseWKT(benchmark::State& state) {
  auto const wkt = makeWKTLineString(static_cast<std::size_t>(state.range(0)));
  for (auto _ : state) {
    dsf::geometry::PolyLine polyline(wkt);
    benchmark::DoNotOptimize(polyline.data());
  }
  state.SetItemsProcessed(state.iterations() * state.range(0));
}

static void BM_PolyLine_FormatWKT(benchmark::State& state) {
  dsf::geometry::PolyLine const polyline(
      makeWKTLineString(static_cast<std::size_t>(state.range(0))));
  for (auto _ : state) {
    auto wkt = std::format("{}", polyline);
    benchmark::DoNotOptimize(wkt.data());
  }
  state.SetItemsProcessed(state.iterations() * state.range(0));
}

BENCHMARK(BM_Point_ParseWKT);
BENCHMARK(BM_Point_Haversine);
BENCHMARK(BM_PolyLine_ParseWKT)->Arg(2)->Arg(50);
BENCHMARK(BM_PolyLine_FormatWKT)->Arg(2)->Arg(50);

BENCHMARK_MAIN();
