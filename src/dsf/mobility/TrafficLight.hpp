/// @file TrafficLight.hpp
/// @brief Phase-based traffic light state machine.
///
/// Design overview
/// ───────────────
/// A TrafficLight cycles through an ordered sequence of TrafficLightPhase
/// objects.  Each phase has a fixed duration (in simulation ticks) and an
/// explicit *green set*: the collection of (streetId, Direction) pairs that
/// are green while that phase is active.  Every street/direction pair that
/// is absent from the green set is implicitly red — no arithmetic offsets or
/// wraparound logic is required.
///
/// State-machine transition (operator++):
///   m_counter is incremented every tick.
///   When m_counter reaches m_phases[m_currentPhaseIndex].duration() the
///   machine moves to the next phase (wrapping) and resets m_counter to 0.
///
/// Query (isGreen):
///   Looks up the active phase's green set and applies the standard
///   direction-fallback ladder (RIGHT → RIGHTANDSTRAIGHT → ANY, etc.).
///   If the street is absent entirely, returns m_allowFreeTurns.
///
/// Reset API:
///   snapshot()  — saves current phase sequence as a restore point.
///   restore()   — returns to the last snapshot.
///   reset()     — returns to construction-time defaults (set by the last
///                 setPhases() / addPhase() calls).
///
/// Optimiser interface:
///   phase(i).setDuration(d) — mutates a single phase duration in-place.
///   defaultPhases()         — read-only view of baseline durations.
///   advanceBy(offset)       — fast-forwards the state machine by `offset`
///                             ticks (used for green-wave synchronisation).

#pragma once

#include "Intersection.hpp"
#include "../utility/Typedef.hpp"

#include <format>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dsf::mobility {
  /// @brief One phase in a TrafficLight program.
  ///
  /// A phase owns:
  ///   - a duration  : how many ticks it stays active.
  ///   - a green set : unordered_map<streetId, unordered_set<Direction>>
  ///                   listing every (street, direction) that is green.
  ///
  /// Streets absent from the green set are implicitly red.
  /// Direction fallback resolution is applied at query time by TrafficLight,
  /// not here — containsGreen() performs an exact lookup only.
  class TrafficLightPhase {
  private:
    Delay m_duration;
    std::unordered_map<Id, std::unordered_set<Direction>> m_greenSet;

  public:
    /// @brief Construct a phase with the given duration and an empty green set.
    /// @param duration Duration of the phase in ticks.
    explicit TrafficLightPhase(Delay const duration) : m_duration{duration} {}
    /// @brief Construct a phase with a duration and a pre-built green set.
    /// @param duration Duration of the phase in ticks.
    /// @param greenSet Unordered map of streetId to the set of green directions for that street.
    TrafficLightPhase(Delay const duration,
                      std::unordered_map<Id, std::unordered_set<Direction>> greenSet)
        : m_duration{duration}, m_greenSet{std::move(greenSet)} {}

    /// @brief Mark a specific direction on a street as green.
    /// @param streetId Street identifier.
    /// @param direction Direction to mark as green.
    inline void addGreen(Id const streetId, Direction const direction) {
      m_greenSet[streetId].insert(direction);
    }
    /// @brief Mark several directions on a street as green.
    /// @param streetId Street identifier.
    /// @param directions List of directions to mark as green for the given street.
    inline void addGreen(Id const streetId, std::initializer_list<Direction> directions) {
      for (auto const dir : directions) {
        m_greenSet[streetId].insert(dir);
      }
    }
    /// @brief Mark a street as green for ALL directions (inserts Direction::ANY).
    /// This is the typical entry produced by auto-deduction.
    /// @param streetId Street identifier.
    inline void addGreen(Id const streetId) {
      m_greenSet[streetId].insert(Direction::ANY);
    }

    /// @brief Check if a street is present in the green set, regardless of direction.
    /// @return True if the street is present in the green set, regardless of direction.
    inline auto containsStreet(Id const streetId) const {
      return m_greenSet.contains(streetId);
    }
    /// @brief Exact lookup — does NOT apply the direction-fallback ladder.
    /// Use TrafficLight::isGreen() for the full resolved query.
    /// @param streetId Street identifier.
    /// @param direction Direction of the movement.
    /// @return True if the (streetId, direction) pair is present in the green set.
    inline auto containsGreen(Id const streetId, Direction const direction) const {
      auto const it = m_greenSet.find(streetId);
      return it != m_greenSet.end() && it->second.contains(direction);
    }

    /// @brief Get the phase duration in ticks.
    /// @return Duration of the phase in ticks.
    inline auto duration() const { return m_duration; }
    /// @brief Set the phase duration (used by the optimiser — does not affect
    /// the green set or the running state machine counter).
    /// @param duration New duration for the phase in ticks.
    inline void setDuration(Delay const duration) { m_duration = duration; }
    /// @brief Get a read-only view of the green set.
    /// @return Read-only view of the green set.
    inline auto const& greenSet() const { return m_greenSet; }

    bool operator==(TrafficLightPhase const& other) const {
      return m_duration == other.m_duration && m_greenSet == other.m_greenSet;
    }
    bool operator!=(TrafficLightPhase const& other) const { return !(*this == other); }
  };

  class TrafficLight final : public Intersection {
  private:
    std::vector<TrafficLightPhase> m_phases;         // Live phase sequence.
    std::vector<TrafficLightPhase> m_defaultPhases;  // Set by setPhases/addPhase.
    std::vector<TrafficLightPhase> m_snapshot;       // Saved by snapshot().

    std::size_t m_currentPhaseIndex{0};
    Delay m_counter{0};  // Ticks elapsed within the current phase.

    static bool m_allowFreeTurns;

    /// @brief Apply the direction-fallback ladder to a set of directions.
    /// Ladder: exact match → ANY → compound (RIGHTANDSTRAIGHT/LEFTANDSTRAIGHT).
    static bool m_resolveDirection(std::unordered_set<Direction> const& dirSet,
                                   Direction direction);

  public:
    /// @brief Construct a new Traffic Light object
    /// @param id The node's id
    explicit TrafficLight(Id const id) : Intersection{id} {}
    /// @brief Construct a new Traffic Light object with a location
    /// @param id The node's id
    /// @param point The location of the traffic light
    TrafficLight(Id const id, geometry::Point point)
        : Intersection{id, std::move(point)} {}
    /// @brief Construct a new Traffic Light object from a road junction
    /// @param node The road junction to convert into a traffic light
    explicit TrafficLight(RoadJunction const& node) : Intersection{node} {}
    ~TrafficLight() = default;

    /// @brief Advance by one simulation tick.
    /// When the counter reaches the current phase's duration the machine
    /// moves to the next phase (wrapping) and resets the counter to 0.
    TrafficLight& operator++();

    /// @brief When true (default), a street absent from the active phase's
    /// green set is treated as green (free turn).
    static void setAllowFreeTurns(bool const allow);

    // ── Phase management ──────────────────────────────────────────────────

    /// @brief Append one phase to the sequence and update stored defaults.
    /// Resets the state machine to phase 0, counter 0.
    void addPhase(TrafficLightPhase phase);
    /// @brief Replace the entire phase sequence and update stored defaults.
    /// Resets the state machine to phase 0, counter 0.
    void setPhases(std::vector<TrafficLightPhase> phases);
    /// @brief Remove all phases and clear all stored defaults/snapshots.
    void clearPhases();

    /// @brief Save the current phase sequence as a restore point.
    /// Does NOT reset the running counter or current phase index.
    void snapshot();
    /// @brief Restore the phase sequence to the last snapshot().
    /// Resets m_currentPhaseIndex and m_counter to 0.
    /// Logs a warning and is a no-op if no snapshot exists.
    void restore();
    /// @brief Restore the phase sequence to construction-time defaults
    /// (the state after the last setPhases() / addPhase() chain).
    /// Resets m_currentPhaseIndex and m_counter to 0.
    void reset();

    /// @brief Returns true if the light is currently green for
    /// (streetId, direction), applying the fallback ladder:
    ///   exact direction → ANY → compound (RIGHTANDSTRAIGHT/LEFTANDSTRAIGHT).
    /// Returns m_allowFreeTurns when streetId is absent from the active phase.
    /// @param streetId Street identifier.
    /// @param direction Direction of the movement.
    /// @return True if the light is currently green for the given street and direction.
    bool isGreen(Id const streetId, Direction const direction) const;

    // ── Timing ────────────────────────────────────────────────────────────

    /// @brief Total cycle time = sum of all phase durations.
    Delay cycleTime() const;

    /// @brief Mean green time (ticks) across priority or non-priority streets.
    /// A street's green time = sum of durations of phases that list it.
    double meanGreenTime(bool priorityStreets) const;

    /// @brief True if the live phase sequence equals the stored defaults.
    bool isDefault() const;

    // ── Green-wave helper ─────────────────────────────────────────────────

    /// @brief Fast-forward the state machine by `offset` ticks.
    /// offset is taken modulo cycleTime() so large values are safe.
    /// Used by the DOUBLE_TAIL optimiser.
    void advanceBy(Delay offset);

    // ── Accessors ─────────────────────────────────────────────────────────

    inline Delay counter() const { return m_counter; }
    inline std::size_t currentPhaseIndex() const { return m_currentPhaseIndex; }

    /// @brief Read-only view of the live phase sequence.
    inline std::vector<TrafficLightPhase> const& phases() const { return m_phases; }

    /// @brief Read-only view of the default (baseline) phase sequence.
    /// Used by the optimiser to access baseline durations without mutating
    /// the live sequence.
    inline std::vector<TrafficLightPhase> const& defaultPhases() const {
      return m_defaultPhases;
    }

    /// @brief Mutable access to one live phase (intended for optimiser only).
    inline TrafficLightPhase& phase(std::size_t index) { return m_phases.at(index); }

    constexpr bool isTrafficLight() const noexcept { return true; }
  };

}  // namespace dsf::mobility

// ── std::formatter specialisations ──────────────────────────────────────────

template <>
struct std::formatter<dsf::mobility::TrafficLightPhase> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(dsf::mobility::TrafficLightPhase const& p, FormatContext&& ctx) const {
    std::string body;
    for (auto const& [streetId, dirs] : p.greenSet()) {
      body += std::format("\tstreet {}:", streetId);
      for (auto const dir : dirs)
        body += std::format(" {}", dsf::directionToString.at(dir));
      body += "\n";
    }
    return std::format_to(
        ctx.out(), "Phase (duration: {} ticks)\n{}", p.duration(), body);
  }
};

template <>
struct std::formatter<dsf::mobility::TrafficLight> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(dsf::mobility::TrafficLight const& tl, FormatContext&& ctx) const {
    std::string phases;
    for (std::size_t i = 0; i < tl.phases().size(); ++i) {
      phases += std::format(
          "  [{}]{} {}", i, (i == tl.currentPhaseIndex()) ? '*' : ' ', tl.phases()[i]);
    }
    return std::format_to(ctx.out(),
                          "TrafficLight \"{}\" (id {}): cycle={} ticks, "
                          "phase {}/{}, counter={}\n{}",
                          tl.name(),
                          tl.id(),
                          tl.cycleTime(),
                          tl.currentPhaseIndex(),
                          tl.phases().size(),
                          tl.counter(),
                          phases);
  }
};
