/// @file       /src/dsf/headers/Itinerary.hpp
/// @brief      Defines the Itinerary class.
///
/// @details    This file contains the definition of the Itinerary class.
///             An itinerary is a destination street, identified by its own id: agents
///             following it plan their route to it (see FirstOrderDynamics).

#pragma once

#include "../utility/Typedef.hpp"

namespace dsf::mobility {
  /// @brief The Itinerary class represents a destination of the agents.
  class Itinerary {
  private:
    Id m_id;
    Id m_destination;

  public:
    /// @brief Construct a new Itinerary object
    /// @param id The itinerary's id
    /// @param destination The id of the itinerary's destination street
    Itinerary(Id id, Id destination);

    // Allow move constructor and move assignment operator
    Itinerary(Itinerary&&) = default;
    Itinerary& operator=(Itinerary&&) = default;
    // Delete copy constructor and copy assignment operator
    Itinerary(const Itinerary&) = delete;
    Itinerary& operator=(const Itinerary&) = delete;

    /// @brief Get the itinerary's id
    /// @return Id, The itinerary's id
    inline auto id() const noexcept { return m_id; };
    /// @brief Get the itinerary's destination
    /// @return Id, The id of the itinerary's destination street
    inline auto destination() const noexcept { return m_destination; };
  };
};  // namespace dsf::mobility
