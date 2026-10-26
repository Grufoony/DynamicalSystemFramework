/// @file       /src/dsf/headers/Dynamics.hpp
/// @brief      Defines the Dynamics class.
///
/// @details    This file contains the definition of the Dynamics class.
///             The Dynamics class represents the dynamics of the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include <atomic>
#include <cstdint>
#include <exception>
#include <format>
#include <memory>
#include <random>
#include <string>

#include "Network.hpp"
#include "../utility/Measurement.hpp"
#include "../utility/Typedef.hpp"

#include <spdlog/spdlog.h>
#include <tbb/tbb.h>

namespace dsf {
  /// @brief The Dynamics class represents the dynamics of the network.
  /// @tparam network_t The type of the network
  template <typename network_t>
  class Dynamics {
  protected:
    std::unique_ptr<network_t> m_graph;

  private:
    std::time_t m_timeStep = 0;
    // Base seed of the per-thread engines, and number of engines created so far
    std::uint64_t m_threadSeed;
    std::atomic<std::uint64_t> m_nThreadGenerators{0};

  protected:
    tbb::task_arena m_taskArena;
    std::mt19937_64 m_generator;
    // One engine per thread, for the parallel kernels: each is created (and seeded) the first
    // time a thread asks for it, so there is no locking nor seeding cost afterwards
    tbb::enumerable_thread_specific<std::mt19937_64> m_threadGenerators;

  protected:
    inline void m_evolve() { ++m_timeStep; };

  public:
    /// @brief Construct a new Dynamics object
    /// @param graph The graph representing the network
    /// @param seed The seed for the random number generator (default is std::nullopt)
    Dynamics(network_t&& graph, std::optional<std::uint64_t> seed = std::nullopt);

    /// @brief Set the seed for the random number generators
    /// @param seed The seed to set
    /// @details With a concurrency of 1 the simulation is reproducible. With more threads it is
    /// not, as which thread (thus which engine) processes each node depends on scheduling
    void setSeed(std::uint64_t const seed);
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
    /// @brief Get the current simulation time-step
    /// @return std::time_t, The current simulation time-step
    inline auto time_step() const { return m_timeStep; }
  };

  template <typename network_t>
  Dynamics<network_t>::Dynamics(network_t&& graph, std::optional<std::uint64_t> seed)
      : m_graph{std::make_unique<network_t>(std::move(graph))},
        m_threadSeed{seed.value_or(std::random_device{}())},
        m_generator{m_threadSeed},
        m_threadGenerators{[this] {
          return std::mt19937_64{
              m_threadSeed + 1 +
              m_nThreadGenerators.fetch_add(1, std::memory_order_relaxed)};
        }} {
    m_taskArena.initialize(static_cast<std::size_t>(tbb::info::default_concurrency()));
  }

  template <typename network_t>
  void Dynamics<network_t>::setSeed(std::uint64_t const seed) {
    m_generator.seed(seed);
    m_threadSeed = seed;
    m_threadGenerators.clear();
    m_nThreadGenerators = 0;
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
    m_taskArena.initialize(static_cast<int>(actualConcurrency));
  }
};  // namespace dsf