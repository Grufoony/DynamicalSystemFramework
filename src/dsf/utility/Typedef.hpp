
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#ifndef __APPLE__
#include <execution>
#define DSF_EXECUTION std::execution::par_unseq,
#else
#define DSF_EXECUTION
#endif
#include <format>
#include <string_view>
#include <unordered_map>

namespace dsf {

  using Id = uint64_t;
  using Delay = uint16_t;

  enum class PathWeight : uint8_t { CUSTOM = 0, LENGTH = 1, TRAVELTIME = 2 };
  enum class SpeedFunction : uint8_t { CUSTOM = 0, LINEAR = 1 };
  enum Direction : uint8_t {
    RIGHT = 0,     // delta < 0
    STRAIGHT = 1,  // delta == 0
    LEFT = 2,      // delta > 0
    UTURN = 3,     // std::abs(delta) > std::numbers::pi
    RIGHTANDSTRAIGHT = 4,
    LEFTANDSTRAIGHT = 5,
    ANY = 6
  };
  constexpr std::array<std::string_view, 7> directionToString{
      "RIGHT", "STRAIGHT", "LEFT", "UTURN", "RIGHT&STRAIGHT", "LEFT&STRAIGHT", "ANY"};
  enum class TrafficLightOptimization : uint8_t { SINGLE_TAIL = 0, DOUBLE_TAIL = 1 };
  enum train_t : uint8_t {
    BUS = 0,           // Autobus
    SFM = 1,           // Servizio Ferroviario Metropolitano
    R = 2,             // Regionale
    RV = 3,            // Regionale Veloce
    IC = 4,            // InterCity (Notte)
    FRECCIA = 5,       // Frecciabianca / Frecciargento
    FRECCIAROSSA = 6,  // Frecciarossa
    ES = 7,            // Eurostar
  };
  enum class FileExt : std::size_t { CSV, JSON, GEOJSON };
  std::unordered_map<std::string, FileExt> const fileExtMap{
      {"csv", FileExt::CSV}, {"json", FileExt::JSON}, {"geojson", FileExt::GEOJSON}};

};  // namespace dsf

template <>
struct std::formatter<dsf::Direction> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(dsf::Direction const& direction, FormatContext& ctx) {
    return std::format_to(
        ctx.out(), "{}", dsf::directionToString[static_cast<size_t>(direction)]);
  }
};
