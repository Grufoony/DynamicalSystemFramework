#pragma once

#include "../base/Node.hpp"

#include <format>

namespace dsf::mobility {
  class RoadJunction : public Node {
    std::size_t m_capacity;
    double m_transportCapacity;

  public:
    explicit RoadJunction(Id id);
    RoadJunction(Id id, geometry::Point coords);
    RoadJunction(RoadJunction const& other);

    RoadJunction& operator=(RoadJunction const& other) {
      if (this != &other) {
        Node::operator=(other);
        m_capacity = other.m_capacity;
        m_transportCapacity = other.m_transportCapacity;
      }
      return *this;
    }

    /// @brief Set the junction's capacity
    /// @param capacity The junction's capacity
    virtual void setCapacity(std::size_t const capacity);
    /// @brief Set the junction's transport capacity
    /// @param capacity The junction's transport capacity, in agents per second
    void setTransportCapacity(double capacity);

    /// @brief Get the junction's capacity
    /// @return std::size_t The junction's capacity
    inline std::size_t capacity() const { return m_capacity; }
    /// @brief Get the junction's transport capacity
    /// @return double The junction's transport capacity, in agents per second
    inline double transportCapacity() const { return m_transportCapacity; }

    virtual inline double density() const { return 0.; }
    virtual inline bool isFull() const { return true; }

    virtual constexpr bool isIntersection() const noexcept { return false; }
    virtual constexpr bool isTrafficLight() const noexcept { return false; }
    virtual constexpr bool isRoundabout() const noexcept { return false; }
  };
}  // namespace dsf::mobility

// Specialization of std::formatter for dsf::mobility::RoadJunction
template <>
struct std::formatter<dsf::mobility::RoadJunction> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(dsf::mobility::RoadJunction const& junction, FormatContext&& ctx) const {
    auto out =
        std::format_to(ctx.out(),
                       "RoadJunction(id: {}, name: {}, capacity: {}, transportCapacity: "
                       "{}, coords: ",
                       junction.id(),
                       junction.name(),
                       junction.capacity(),
                       junction.transportCapacity());
    if (junction.geometry().has_value()) {
      out = std::format_to(out, "{})", *junction.geometry());
    } else {
      out = std::format_to(out, "N/A)");
    }
    return out;
  }
};