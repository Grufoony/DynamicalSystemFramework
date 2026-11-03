/// @file       /src/dsf/utility/FlowAccumulator.hpp
/// @brief      Defines the FlowAccumulator class.

#pragma once

#include <algorithm>
#include <cassert>
#include <ctime>
#include <optional>

namespace dsf {
  /// @brief A deterministic flow gate
  /// @details Each time step adds the capacity (agents per time step) without exceeding it,
  ///   and each agent passing through consume()s one unit, so that the value can become
  ///   negative. Agents may pass while canPass(). Hence an idle gate lets a lone agent pass
  ///   at once, while a platoon is released at exactly `capacity` agents per time step, at
  ///   most ceil(capacity) in a single step.
  ///   The gate is refilled lazily: refill() catches up on all the time steps since the
  ///   previous call, so it only needs calling when an agent tries to pass.
  class FlowAccumulator {
  private:
    // Absorbs the floating-point error of summing fractional capacities: a value that
    // should be exactly 0 (no passage yet) may come out as a tiny positive residue
    static constexpr double TOLERANCE{1e-9};
    double m_value{0.};
    std::optional<std::time_t> m_lastRefill;  // Empty until the first refill

  public:
    /// @brief Bring the accumulator up to date with a time step
    /// @details Adds the capacity once per time step since the previous refill, without
    ///   exceeding it: refilling once after k idle time steps is the same as refilling at
    ///   each of them. The first refill fills the gate, a second one in the same time step
    ///   has no effect.
    /// @param capacity The number of agents per time step allowed through, positive. If
    ///   it changed since the previous refill, it applies to the whole interval.
    /// @param timeStep The current time step
    inline void refill(double const capacity, std::time_t const timeStep) noexcept {
      if (!m_lastRefill.has_value()) [[unlikely]] {
        m_value = capacity;
      } else {
        assert(timeStep >= *m_lastRefill);
        auto const nSteps{static_cast<double>(timeStep - *m_lastRefill)};
        m_value = std::min(m_value + nSteps * capacity, capacity);
      }
      m_lastRefill = timeStep;
    }
    /// @brief Consume one unit, as an agent passes through
    inline void consume() noexcept { m_value -= 1.; }
    /// @brief Check whether an agent can pass through
    /// @return bool True if the accumulated value is positive
    inline bool canPass() const noexcept { return m_value > TOLERANCE; }
    /// @brief Get the accumulated value
    /// @return double The accumulated value, at most the last refilled capacity
    inline double value() const noexcept { return m_value; }
  };
}  // namespace dsf
