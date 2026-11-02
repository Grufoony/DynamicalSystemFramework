/// @file       /src/dsf/utility/FlowAccumulator.hpp
/// @brief      Defines the FlowAccumulator class.

#pragma once

#include <algorithm>

namespace dsf {
  /// @brief A deterministic flow gate
  /// @details Each time step refill() adds the capacity (agents per time step) without
  ///   exceeding it, and each agent passing through consume()s one unit, so that the value
  ///   can become negative. Agents may pass while canPass(). Hence an idle gate lets a lone
  ///   agent pass at once, while a platoon is released at exactly `capacity` agents per time
  ///   step, at most ceil(capacity) in a single step.
  class FlowAccumulator {
  private:
    // Absorbs the floating-point error of summing fractional capacities: a value that
    // should be exactly 0 (no passage yet) may come out as a tiny positive residue
    static constexpr double TOLERANCE{1e-9};
    double m_value{0.};

  public:
    /// @brief Refill the accumulator for a new time step
    /// @param capacity The number of agents per time step allowed through, positive
    inline void refill(double const capacity) noexcept {
      m_value = std::min(m_value + capacity, capacity);
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
