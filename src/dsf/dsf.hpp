#ifndef dsf_hpp
#define dsf_hpp

#include <cstdint>
#include <format>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

static constexpr uint8_t DSF_VERSION_MAJOR = 5;
static constexpr uint8_t DSF_VERSION_MINOR = 8;
static constexpr uint8_t DSF_VERSION_PATCH = 3;

static auto const DSF_VERSION =
    std::format("{}.{}.{}", DSF_VERSION_MAJOR, DSF_VERSION_MINOR, DSF_VERSION_PATCH);

namespace dsf {
  /// @brief Returns the version of the DSF library
  /// @return The version of the DSF library
  auto const& version() { return DSF_VERSION; };

  /// @brief Set up logging to a specified file
  /// @param path The path to the log file
  void log_to_file(std::string const& path) {
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
