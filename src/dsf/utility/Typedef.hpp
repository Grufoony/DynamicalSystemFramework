
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
#include <optional>
#include <string_view>

namespace dsf {

  using Id = uint64_t;
  using Delay = uint16_t;

  enum class SpeedFunction : uint8_t { CUSTOM = 0, LINEAR = 1, CONSTANT = 2 };
  enum Direction : uint8_t {
    RIGHT = 0,  // delta < 0
    RIGHTANDSTRAIGHT = 1,
    STRAIGHT = 2,  // delta == 0
    ANY = 3,
    LEFTANDSTRAIGHT = 4,
    LEFT = 5,  // delta > 0
    UTURN = 6  // std::abs(delta) > std::numbers::pi
  };
  inline constexpr std::array<std::string_view, 7> directionToString{
      "RIGHT", "RIGHT&STRAIGHT", "STRAIGHT", "ANY", "LEFT&STRAIGHT", "LEFT", "UTURN"};
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
  /// @brief Get the FileExt matching a file extension (without the leading dot)
  /// @param ext The file extension, e.g. "csv"
  /// @return std::optional<FileExt> The matching FileExt, or std::nullopt if unsupported
  constexpr std::optional<FileExt> fileExtFromString(std::string_view ext) noexcept {
    if (ext == "csv") {
      return FileExt::CSV;
    }
    if (ext == "json") {
      return FileExt::JSON;
    }
    if (ext == "geojson") {
      return FileExt::GEOJSON;
    }
    return std::nullopt;
  }

  /// @brief Conversion factor from m/s to km/h
  inline constexpr double MS_TO_KMH = 3.6;

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
