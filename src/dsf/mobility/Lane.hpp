/// @file       /src/dsf/mobility/Lane.hpp
/// @brief      Defines the Lane struct.

#pragma once

#include "Agent.hpp"
#include "../utility/FlowAccumulator.hpp"
#include "../utility/queue.hpp"
#include "../utility/Typedef.hpp"

#include <memory>

namespace dsf::mobility {
  /// @brief A lane of a street: its exit queue, the turns it serves and its exit gate
  /// @details Plain data: the Street owning the lane keeps it consistent with its own state
  struct Lane {
    dsf::queue<std::unique_ptr<Agent>> queue;  ///< Agents waiting to leave the street
    Direction direction{Direction::ANY};       ///< The turns the lane serves
    FlowAccumulator flowAccumulator;  ///< Exit gate, refilled by the transport capacity
  };
}  // namespace dsf::mobility
