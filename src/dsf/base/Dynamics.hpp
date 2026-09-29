/// @file       /src/dsf/headers/Dynamics.hpp
/// @brief      Defines the Dynamics class.
///
/// @details    This file contains the definition of the Dynamics class.
///             The Dynamics class represents the dynamics of the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include <algorithm>
#include <cmath>
#include <exception>
#include <format>
#include <memory>
#include <random>
#include <stdexcept>
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
    double m_dt{1.};  // Seconds per time step
    std::unique_ptr<tbb::global_control> m_globalControl;

  protected:
    tbb::task_arena m_taskArena;
    std::mt19937_64 m_generator;

  protected:
    inline void m_evolve() { ++m_timeStep; };

  public:
    /// @brief Construct a new Dynamics object
    /// @param graph The graph representing the network
    /// @param seed The seed for the random number generator (default is std::nullopt)
    Dynamics(network_t&& graph, std::optional<unsigned int> seed = std::nullopt);

    /// @brief Set the seed for the random number generator
    /// @param seed The seed to set
    inline void setSeed(unsigned int const seed) noexcept { m_generator.seed(seed); }
    /// @brief Set the maximum number of threads to use for parallel execution
    /// @param concurrency The maximum number of threads to use for parallel execution
    void setConcurrency(std::size_t const concurrency);
    /// @brief Set the duration of a simulation time step, in seconds
    /// @param dt The duration of a time step in seconds. It must be either an integer (n) or the inverse of an integer (1/n), so that seconds and time steps are always related by an integer factor.
    /// @throw std::invalid_argument If dt is not positive or is neither n nor 1/n
    void setDt(double const dt);
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
    /// @brief Get the duration of a simulation time step, in seconds
    /// @return double, The duration of a time step in seconds
    inline auto dt() const { return m_dt; }
    /// @brief Convert a duration in seconds into a number of time steps
    /// @param seconds The duration in seconds
    /// @return std::time_t, The number of time steps
    /// @throw std::invalid_argument If seconds is not a multiple of dt
    std::time_t secondsToTimeSteps(double const seconds) const;
    /// @brief Convert a number of time steps into a duration in seconds
    /// @param timeSteps The number of time steps
    /// @return double, The duration in seconds
    inline double timeStepsToSeconds(std::time_t const timeSteps) const {
      return static_cast<double>(timeSteps) * m_dt;
    }
  };

  template <typename network_t>
  Dynamics<network_t>::Dynamics(network_t&& graph, std::optional<unsigned int> seed)
      : m_graph{std::make_unique<network_t>(std::move(graph))},
        m_generator{std::random_device{}()} {
    if (seed.has_value()) {
      m_generator.seed(*seed);
    }
    m_taskArena.initialize(static_cast<std::size_t>(tbb::info::default_concurrency()));
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

  template <typename network_t>
  void Dynamics<network_t>::setDt(double const dt) {
    constexpr double tolerance{1e-9};
    auto const isInteger = [](double const value) {
      return std::abs(value - std::round(value)) < tolerance * std::max(1., value);
    };
    if (!(dt > 0.) || !(isInteger(dt) || isInteger(1. / dt))) {
      throw std::invalid_argument(std::format(
          "The time step duration dt ({}) must be a positive integer or the inverse of "
          "a positive integer.",
          dt));
    }
    m_dt = dt >= 1. ? std::round(dt) : 1. / std::round(1. / dt);
  }

  template <typename network_t>
  std::time_t Dynamics<network_t>::secondsToTimeSteps(double const seconds) const {
    constexpr double tolerance{1e-9};
    auto const timeSteps{seconds / m_dt};
    auto const rounded{std::round(timeSteps)};
    if (std::abs(timeSteps - rounded) > tolerance * std::max(1., std::abs(timeSteps))) {
      throw std::invalid_argument(std::format(
          "The duration {} s is not a multiple of the time step duration dt ({} s).",
          seconds,
          m_dt));
    }
    return static_cast<std::time_t>(rounded);
  }
};  // namespace dsf