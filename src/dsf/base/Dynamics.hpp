/// @file       /src/dsf/headers/Dynamics.hpp
/// @brief      Defines the Dynamics class.
///
/// @details    This file contains the definition of the Dynamics class.
///             The Dynamics class represents the dynamics of the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include "Network.hpp"
#include "../utility/Measurement.hpp"
#include "../utility/Typedef.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <concepts>
#include <vector>
#include <memory>
#include <random>
#include <span>
#include <numeric>
#include <unordered_map>
#include <cmath>
#include <cassert>
#include <format>
#include <thread>
#include <exception>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iomanip>
#ifdef __APPLE__
#include <sstream>
#endif

#include <spdlog/spdlog.h>
#include <tbb/tbb.h>
#include <SQLiteCpp/SQLiteCpp.h>

namespace dsf {
  /// @brief The Dynamics class represents the dynamics of the network.
  /// @tparam network_t The type of the network
  template <typename network_t>
  class Dynamics {
  private:
    std::unique_ptr<network_t> m_graph;
    Id m_id;
    std::string m_name = "unnamed simulation";
    std::time_t m_timeStep = 0;
    std::unique_ptr<tbb::global_control> m_globalControl;

  protected:
    tbb::task_arena m_taskArena;
    std::mt19937_64 m_generator;

  protected:
    inline void m_evolve() { ++m_timeStep; };
    /// @brief Get a safe name string for filenames (spaces replaced by underscores)
    /// @return std::string, The safe name string
    inline auto m_safeName() const {
      std::string safeName = m_name;
      std::replace(safeName.begin(), safeName.end(), ' ', '_');
      return safeName;
    }

  public:
    /// @brief Construct a new Dynamics object
    /// @param graph The graph representing the network
    /// @param seed The seed for the random number generator (default is std::nullopt)
    Dynamics(network_t&& graph, std::optional<unsigned int> seed = std::nullopt);

    /// @brief Set the name of the simulation
    /// @param name The name of the simulation
    inline void setName(const std::string& name) { m_name = name; };
    /// @brief Set the maximum number of threads to use for parallel execution
    /// @param concurrency The maximum number of threads to use for parallel execution
    void setConcurrency(std::size_t const concurrency);
    /// @brief Get the current concurrency (number of threads configured in the task arena)
    /// @return std::size_t, The current concurrency
    inline auto concurrency() const {
      return static_cast<std::size_t>(m_taskArena.max_concurrency());
    }
    

    /// @brief Get the graph
    /// @return const network_t&, The graph
    inline auto const& graph() const { return *m_graph; };
    /// @brief Get the graph (mutable)
    /// @return network_t&, The graph
    inline auto& graph() { return *m_graph; };
    /// @brief Get the id of the simulation
    /// @return const Id&, The id of the simulation
    inline auto const& id() const { return m_id; };
    /// @brief Get the name of the simulation
    /// @return const std::string&, The name of the simulation
    inline auto const& name() const { return m_name; };
    /// @brief Get the current simulation time-step
    /// @return std::time_t, The current simulation time-step
    inline auto time_step() const { return m_timeStep; }
  };

  template <typename network_t>
  Dynamics<network_t>::Dynamics(network_t&& graph, std::optional<unsigned int> seed)
      : m_graph{std::make_unique<network_t>(std::move(graph))},
        m_generator{std::random_device{}()} {
    if (seed.has_value()) {
      m_generator.seed(*seed);
    }
    m_taskArena.initialize(static_cast<std::size_t>(tbb::info::default_concurrency()));
    // Take the current time and set id as YYYYMMDDHHMMSS
    auto const now = std::chrono::system_clock::now();
#ifdef __APPLE__
    std::time_t const t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y%m%d%H%M%S");
    m_id = std::stoull(oss.str());
#else
    m_id = std::stoull(std::format(
        "{:%Y%m%d%H%M%S}",
        std::chrono::floor<std::chrono::seconds>(
            std::chrono::current_zone()->to_local(std::chrono::system_clock::from_time_t(
                std::chrono::system_clock::to_time_t(now))))));
#endif
  }

  template <typename network_t>
  void Dynamics<network_t>::setConcurrency(std::size_t const concurrency) {
    m_taskArena.terminate();
    spdlog::info("Setting concurrency to {} threads.", concurrency);
    auto const maxConcurrency =
        static_cast<std::size_t>(tbb::info::default_concurrency());
    auto actualConcurrency = concurrency;
    if (concurrency == 0 || concurrency > maxConcurrency) {
      actualConcurrency = maxConcurrency;
      spdlog::warn(
          "Requested concurrency ({}) is invalid. Using maximum available concurrency "
          "({}).",
          concurrency,
          maxConcurrency);
    }
    // Limit the TBB global thread pool so the OS spawns only actualConcurrency
    // worker threads, rather than the full hardware_concurrency() count.
    m_globalControl.reset();
    m_globalControl = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, actualConcurrency);
    m_taskArena.initialize(static_cast<int>(actualConcurrency));
  }
};  // namespace dsf