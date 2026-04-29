#include "TrafficLight.hpp"

#include <format>
#include <numeric>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace dsf::mobility {
  bool TrafficLightCycle::isGreen(Delay const cycleTime, Delay const counter) const {
    auto const greenStart = m_phase % cycleTime;
    auto const greenEnd = (m_phase + m_greenTime) % cycleTime;

    if (greenStart < greenEnd) {
      // Normal case: green does not wrap around
      return (counter >= greenStart) && (counter < greenEnd);
    } else {
      // Wraparound case: green spans cycle boundary
      return (counter >= greenStart) || (counter < greenEnd);
    }
  }

  bool TrafficLight::m_allowFreeTurns{true};
  void TrafficLight::setAllowFreeTurns(bool allow) { m_allowFreeTurns = allow; }
  void TrafficLight::setCycle(Id const streetId,
                              Direction direction,
                              TrafficLightCycle const& cycle) {
    if ((cycle.greenTime() > m_cycleTime)) {
      throw std::invalid_argument(
          std::format("Green time ({}) must not exceed the cycle time ({}).",
                      cycle.greenTime(),
                      m_cycleTime));
    }
    if (!(cycle.phase() < m_cycleTime)) {
      throw std::invalid_argument(
          std::format("Phase ({}) must be less than the cycle time ({}).",
                      cycle.phase(),
                      m_cycleTime));
    }
    m_cycles[streetId].emplace(direction, cycle);
  }

  void TrafficLightCycle::reset() {
    m_greenTime = m_defaultValues.first;
    m_phase = m_defaultValues.second;
  }

  void TrafficLight::setComplementaryCycle(Id const streetId, Id const existingCycle) {
    if (m_cycles.contains(streetId)) {
      throw std::invalid_argument(std::format(
          "Street with id {} already has a cycle in traffic light with id {}.",
          streetId,
          m_id));
    }
    if (!m_cycles.contains(existingCycle)) {
      throw std::invalid_argument(std::format(
          "Existing cycle with id {} does not exist in traffic light with id {}.",
          existingCycle,
          m_id));
    }
    m_cycles.emplace(streetId, m_cycles.at(existingCycle));
    for (auto& [direction, cycle] : m_cycles.at(streetId)) {
      cycle = TrafficLightCycle(m_cycleTime - cycle.greenTime(),
                                cycle.phase() + m_cycleTime - cycle.greenTime());
    }
  }

  TrafficLight& TrafficLight::operator++() {
    m_counter = (m_counter + 1) % m_cycleTime;
    return *this;
  }

  double TrafficLight::meanGreenTime(bool priorityStreets) const {
    double meanTime{0.};
    size_t nCycles{0};
    for (auto const& [streetId, cycles] : m_cycles) {
      if ((priorityStreets && m_streetPriorities.contains(streetId)) ||
          (!priorityStreets && !m_streetPriorities.contains(streetId))) {
        meanTime +=
            std::transform_reduce(cycles.begin(),
                                  cycles.end(),
                                  0.0,                  // Initial value (double)
                                  std::plus<double>(),  // Reduction function (addition)
                                  [](const auto& pair) -> double {
                                    return static_cast<double>(pair.second.greenTime());
                                  });
        nCycles += cycles.size();
      }
    }
    return meanTime / nCycles;
  }

  void TrafficLight::increasePhases(Delay const phase) {
    for (auto& [streetId, cycles] : m_cycles) {
      for (auto& [direction, cycle] : cycles) {
        // Module new phase with cycleTime
        auto const newPhase{static_cast<Delay>((phase + cycle.phase()) % m_cycleTime)};
        cycle = TrafficLightCycle(cycle.greenTime(), newPhase);
      }
    }
  }

  bool TrafficLight::isDefault() const {
    for (auto const& [streetId, cycles] : m_cycles) {
      for (auto const& [direction, cycle] : cycles) {
        if (!cycle.isDefault()) {
          return false;
        }
      }
    }
    return true;
  }

  bool TrafficLight::isGreen(Id const streetId, Direction direction) const {
    if (!m_cycles.contains(streetId)) {
      throw std::invalid_argument(
          std::format("Street id {} is not valid for node {}.", streetId, id()));
    }
    if (!m_cycles.at(streetId).contains(direction)) {
      if (m_cycles.at(streetId).contains(Direction::ANY)) {
        direction = Direction::ANY;
      } else {
        switch (direction) {
          case Direction::RIGHT:
            if (m_cycles.at(streetId).contains(Direction::RIGHTANDSTRAIGHT)) {
              direction = Direction::RIGHTANDSTRAIGHT;
            } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
              direction = Direction::ANY;
            }
            break;
          case Direction::LEFT:
            if (m_cycles.at(streetId).contains(Direction::LEFTANDSTRAIGHT)) {
              direction = Direction::LEFTANDSTRAIGHT;
            } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
              direction = Direction::ANY;
            }
            break;
          case Direction::STRAIGHT:
            if (m_cycles.at(streetId).contains(Direction::RIGHTANDSTRAIGHT)) {
              direction = Direction::RIGHTANDSTRAIGHT;
            } else if (m_cycles.at(streetId).contains(Direction::LEFTANDSTRAIGHT)) {
              direction = Direction::LEFTANDSTRAIGHT;
            } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
              direction = Direction::ANY;
            }
            break;
          case Direction::UTURN:
            if (m_cycles.at(streetId).contains(Direction::LEFT)) {
              direction = Direction::LEFT;
            } else if (m_cycles.at(streetId).contains(Direction::LEFTANDSTRAIGHT)) {
              direction = Direction::LEFTANDSTRAIGHT;
            } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
              direction = Direction::ANY;
            }
            break;
          default:
            spdlog::debug(
                "Street {} has ...ANDSTRAIGHT phase but Traffic Light {} doesn't.",
                streetId,
                m_id);
        }
      }
    }
    if (!m_cycles.at(streetId).contains(direction)) {
      return m_allowFreeTurns;
    }
    return m_cycles.at(streetId).at(direction).isGreen(m_cycleTime, m_counter);
  }

  void TrafficLight::resetCycles() {
    m_defaultCycles.empty() ? m_defaultCycles = m_cycles : m_cycles = m_defaultCycles;
  }
}  // namespace dsf::mobility