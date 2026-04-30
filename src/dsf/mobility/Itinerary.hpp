/// @file       /src/dsf/headers/Itinerary.hpp
/// @brief      Defines the Itinerary class.
///
/// @details    This file contains the definition of the Itinerary class.
///             The Itinerary class represents an itinerary in the network. It is templated
///             by the type of the itinerary's id, which must be an unsigned integral type.
///             An itinerary is defined by its id, its destination and the path to reach it.

#pragma once

#include "../base/PathCollection.hpp"
#include "../utility/Typedef.hpp"

#include <concepts>
#include <utility>
#include <string>
#include <format>
#include <memory>
#include <unordered_map>
#include <vector>

namespace dsf::mobility {
  /// @brief The Itinerary class represents an itinerary in the network.
  /// @tparam Id The type of the itinerary's id. It must be an unsigned integral type.
  class Itinerary {
  private:
    Id m_id;
    Id m_destination;
    PathCollection m_path;

  public:
    /// @brief Construct a new Itinerary object
    /// @param id The itinerary's id
    /// @param destination The itinerary's destination
    Itinerary(Id id, Id destination);

    // Allow move constructor and move assignment operator
    Itinerary(Itinerary&&) = default;
    Itinerary& operator=(Itinerary&&) = default;
    // Delete copy constructor and copy assignment operator
    Itinerary(const Itinerary&) = delete;
    Itinerary& operator=(const Itinerary&) = delete;

    void load(const std::string& fileName);

    /// @brief Set the itinerary's path
    /// @param pathCollection A dsf::mobility::PathCollection representing all equivalent paths to the destination
    void setPath(PathCollection pathCollection);

    /// @brief Get the itinerary's id
    /// @return Id, The itinerary's id
    inline auto id() const noexcept { return m_id; };
    /// @brief Get the itinerary's destination
    /// @return Id, The itinerary's destination
    inline auto destination() const noexcept { return m_destination; };
    /// @brief Get the itinerary's path
    /// @return PathCollection const&, The itinerary's path
    inline auto const& path() const noexcept { return m_path; };
    /// @brief Check if the itinerary's path is empty
    /// @return true if the itinerary's path is empty, false otherwise
    inline auto empty() const noexcept { return m_path.empty(); };
    /// @brief Save the itinerary to a binary file
    /// @param fileName The name of the file to save the itinerary to
    void save(const std::string& fileName) const;
  };
};  // namespace dsf::mobility
