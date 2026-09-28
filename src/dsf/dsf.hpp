#ifndef dsf_hpp
#define dsf_hpp

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

inline constexpr std::uint8_t DSF_VERSION_MAJOR = 7;
inline constexpr std::uint8_t DSF_VERSION_MINOR = 3;
inline constexpr std::uint8_t DSF_VERSION_PATCH = 2;

namespace dsf::detail {
  constexpr std::size_t digitCount(std::uint8_t value) noexcept {
    return value >= 100 ? 3 : (value >= 10 ? 2 : 1);
  }

  /// @brief Write the decimal digits of value starting at buffer[pos], advancing pos
  template <std::size_t N>
  constexpr void writeDigits(std::array<char, N>& buffer,
                             std::size_t& pos,
                             std::uint8_t value) noexcept {
    auto const nDigits = digitCount(value);
    for (std::size_t i = nDigits; i > 0; --i) {
      buffer[pos + i - 1] = static_cast<char>('0' + value % 10);
      value /= 10;
    }
    pos += nDigits;
  }

  /// @brief Build the "major.minor.patch" string at compile time, null-terminated
  template <std::uint8_t Major, std::uint8_t Minor, std::uint8_t Patch>
  consteval auto makeVersionString() {
    std::array<char, digitCount(Major) + digitCount(Minor) + digitCount(Patch) + 3>
        buffer{};
    std::size_t pos{0};
    writeDigits(buffer, pos, Major);
    buffer[pos++] = '.';
    writeDigits(buffer, pos, Minor);
    buffer[pos++] = '.';
    writeDigits(buffer, pos, Patch);
    return buffer;
  }

  inline constexpr auto VERSION_BUFFER =
      makeVersionString<DSF_VERSION_MAJOR, DSF_VERSION_MINOR, DSF_VERSION_PATCH>();
}  // namespace dsf::detail

inline constexpr std::string_view DSF_VERSION{dsf::detail::VERSION_BUFFER.data(),
                                              dsf::detail::VERSION_BUFFER.size() - 1};

namespace dsf {
  /// @brief Returns the version of the DSF library
  /// @return The version of the DSF library
  constexpr std::string_view version() noexcept { return DSF_VERSION; };

  /// @brief Set up logging to a specified file
  /// @param path The path to the log file
  inline void log_to_file(std::string const& path) {
    try {
      spdlog::info("Logging to file: {}", path);
      auto file_logger = spdlog::basic_logger_mt("dsf_file_logger", path);
      spdlog::set_default_logger(file_logger);
    } catch (const spdlog::spdlog_ex& ex) {
      spdlog::error("Log initialization failed: {}", ex.what());
    }
  };
}  // namespace dsf

#include "base/Edge.hpp"
#include "mobility/Agent.hpp"
#include "mobility/FirstOrderDynamics.hpp"
#include "mobility/TrafficSimulator.hpp"
#include "mobility/Intersection.hpp"
#include "mobility/Itinerary.hpp"
#include "mobility/RoadNetwork.hpp"
#include "mobility/Roundabout.hpp"
#include "mobility/Street.hpp"
#include "mobility/TrafficLight.hpp"
#include "mdt/TrajectoryCollection.hpp"
#include "utility/TypeTraits/is_node.hpp"
#include "utility/TypeTraits/is_street.hpp"
#include "utility/TypeTraits/is_numeric.hpp"

#endif
