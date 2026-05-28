#include "TrafficLight.hpp"

#include <algorithm>
#include <format>
#include <numeric>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace dsf::mobility {

  // ══════════════════════════════════════════════════════════════════════════
  // Direction-fallback ladder (private, static)
  // ══════════════════════════════════════════════════════════════════════════

  bool TrafficLight::m_resolveDirection(std::unordered_set<Direction> const& dirSet,
                                        Direction direction) {
    // 1. Exact match.
    if (dirSet.contains(direction))
      return true;

    // 2. ANY covers every direction.
    if (dirSet.contains(Direction::ANY))
      return true;

    // 3. Compound-direction fallbacks.
    switch (direction) {
      case Direction::RIGHT:
        return dirSet.contains(Direction::RIGHTANDSTRAIGHT);

      case Direction::LEFT:
        return dirSet.contains(Direction::LEFTANDSTRAIGHT);

      case Direction::STRAIGHT:
        return dirSet.contains(Direction::RIGHTANDSTRAIGHT) ||
               dirSet.contains(Direction::LEFTANDSTRAIGHT);

      case Direction::UTURN:
        // U-turn treated similarly to left turn for signal purposes.
        return dirSet.contains(Direction::LEFT) ||
               dirSet.contains(Direction::LEFTANDSTRAIGHT);

      default:
        // Compound directions (RIGHTANDSTRAIGHT, LEFTANDSTRAIGHT) arrive here
        // only when there is no exact or ANY match — no further fallback.
        return false;
    }
  }

  TrafficLight& TrafficLight::operator++() {
    if (m_phases.empty())
      return *this;

    ++m_counter;
    if (m_counter >= m_phases[m_currentPhaseIndex].duration()) {
      m_counter = 0;
      m_currentPhaseIndex = (m_currentPhaseIndex + 1) % m_phases.size();
    }
    return *this;
  }

  void TrafficLight::addPhase(TrafficLightPhase phase) {
    if (phase.duration() == 0) {
      throw std::invalid_argument(
          std::format("TrafficLight {}: cannot add phase with zero duration.", m_id));
    }
    m_phases.push_back(phase);
    m_defaultPhases.push_back(std::move(phase));
    // Reset state machine so the index cannot go stale.
    m_currentPhaseIndex = 0;
    m_counter = 0;
  }
  void TrafficLight::setPhases(std::vector<TrafficLightPhase> phases) {
    for (auto const& phase : phases) {
      if (phase.duration() == 0) {
        throw std::invalid_argument(
            std::format("TrafficLight {}: cannot add phase with zero duration.", m_id));
      }
    }
    m_phases = phases;
    m_defaultPhases = std::move(phases);
    m_currentPhaseIndex = 0;
    m_counter = 0;
  }
  void TrafficLight::clearPhases() {
    m_phases.clear();
    m_defaultPhases.clear();
    m_snapshot.reset();
    m_currentPhaseIndex = 0;
    m_counter = 0;
  }

  void TrafficLight::snapshot() {
    m_snapshot = m_phases;
    spdlog::debug(
        "TrafficLight {}: snapshot saved ({} phases).", m_id, m_snapshot->size());
  }
  void TrafficLight::restore() {
    if (!m_snapshot.has_value()) {
      spdlog::warn("TrafficLight {}: restore() called but no snapshot exists — ignoring.",
                   m_id);
      return;
    }
    m_phases = *m_snapshot;
    m_currentPhaseIndex = 0;
    m_counter = 0;
    spdlog::debug(
        "TrafficLight {}: restored to snapshot ({} phases).", m_id, m_phases.size());
  }
  void TrafficLight::reset() {
    m_phases = m_defaultPhases;
    m_currentPhaseIndex = 0;
    m_counter = 0;
    spdlog::debug(
        "TrafficLight {}: reset to defaults ({} phases).", m_id, m_phases.size());
  }

  bool TrafficLight::isGreen(Id const streetId, Direction const direction) const {
    if (m_phases.empty()) {
      spdlog::trace("TrafficLight {}: no phases — returning m_allowFreeTurns={}.",
                    m_id,
                    m_allowFreeTurns);
      return m_allowFreeTurns;
    }

    auto const& activePhase = m_phases[m_currentPhaseIndex];
    auto const streetIt = activePhase.greenSet().find(streetId);

    if (streetIt == activePhase.greenSet().end()) {
      // Street is not listed in this phase at all.
      spdlog::trace(
          "TrafficLight {}: street {} absent from phase {} — returning "
          "m_allowFreeTurns={}.",
          m_id,
          streetId,
          m_currentPhaseIndex,
          m_allowFreeTurns);
      return m_allowFreeTurns;
    }

    bool const green = m_resolveDirection(streetIt->second, direction);
    spdlog::trace("TrafficLight {}: street {} dir {} → {}.",
                  m_id,
                  streetId,
                  dsf::directionToString.at(direction),
                  green ? "GREEN" : "RED");
    return green;
  }

  Delay TrafficLight::cycleTime() const {
    return std::accumulate(
        m_phases.cbegin(),
        m_phases.cend(),
        Delay{0},
        [](Delay sum, TrafficLightPhase const& p) { return sum + p.duration(); });
  }

  double TrafficLight::meanGreenTime(bool const priorityStreets) const {
    // Collect the set of unique streets that appear in any phase.
    std::unordered_set<Id> streets;
    for (auto const& phase : m_phases) {
      for (auto const& [streetId, _] : phase.greenSet()) {
        streets.emplace(streetId);
      }
    }

    double total{0.};
    std::size_t count{0};

    for (auto const streetId : streets) {
      bool const isPriority = m_streetPriorities.contains(streetId);
      if (isPriority != priorityStreets) {
        continue;
      }

      // Sum duration of every phase that has this street green.
      for (auto const& phase : m_phases) {
        if (phase.containsStreet(streetId)) {
          total += static_cast<double>(phase.duration());
        }
      }
      ++count;
    }

    return (count > 0) ? (total / static_cast<double>(count)) : 0.;
  }

  bool TrafficLight::isDefault() const { return m_phases == m_defaultPhases; }

  void TrafficLight::advanceBy(Delay offset) {
    if (m_phases.empty() || offset == 0) {
      return;
    }

    auto const total = cycleTime();
    if (total == 0)
      return;
    offset %= total;

    // Walk forward phase-by-phase consuming `offset` ticks.
    while (offset > 0) {
      auto const remaining = m_phases[m_currentPhaseIndex].duration() - m_counter;

      if (offset < remaining) {
        m_counter += offset;
        break;
      }
      // Consume the rest of this phase and move to the next.
      offset -= remaining;
      m_counter = 0;
      m_currentPhaseIndex = (m_currentPhaseIndex + 1) % m_phases.size();
    }

    spdlog::debug("TrafficLight {}: advanced by offset → phase {}, counter {}.",
                  m_id,
                  m_currentPhaseIndex,
                  m_counter);
  }

}  // namespace dsf::mobility
