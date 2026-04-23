#pragma once

#include "../geometry/Point.hpp"
#include "../geometry/PolyLine.hpp"
#include "../utility/Typedef.hpp"

#include <format>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace dsf {
  class Edge {
  protected:
    geometry::PolyLine m_geometry;
    Id m_id;
    std::pair<Id, Id> m_nodePair;
    double m_angle;
    std::unordered_map<
        std::string,
        std::variant<std::monostate, bool, std::int64_t, double, std::string>>
        m_attributes;

    void m_setAngle(geometry::Point srcNodeCoordinates,
                    geometry::Point dstNodeCoordinates);

  public:
    /// @brief Construct a new Edge object
    /// @param id The edge's id
    /// @param nodePair The edge's node pair (u, v) with the edge u -> v
    /// @param geometry The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's
    /// geometry. Default is an empty vector.
    Edge(Id id, std::pair<Id, Id> nodePair, geometry::PolyLine geometry = {});
    Edge(Edge&&) = default;
    Edge(const Edge&) = delete;
    virtual ~Edge() = default;

    /// @brief Set the edge's geometry
    /// @param geometry dsf::geometry::PolyLine The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's geometry
    void setGeometry(geometry::PolyLine geometry);
    /// @brief Set an attribute for the edge
    /// @param name The attribute's name
    /// @param value The attribute's value
    void setAttribute(
        std::string const& name,
        std::variant<std::monostate, bool, std::int64_t, double, std::string> const&
            value);

    /// @brief Get the edge's id
    /// @return Id The edge's id
    inline auto id() const { return m_id; }
    /// @brief Get the edge's source node id
    /// @return Id The edge's source node id
    inline auto source() const { return m_nodePair.first; }
    /// @brief Get the edge's target node id
    /// @return Id The edge's target node id
    inline auto target() const { return m_nodePair.second; }
    /// @brief Get the edge's node pair
    /// @return std::pair<Id, Id> The edge's node pair, where the first element is the source node id and the second
    /// element is the target node id. The pair is (u, v) with the edge u -> v.
    inline auto const& nodePair() const { return m_nodePair; }
    /// @brief Get the edge's geometry
    /// @return dsf::geometry::PolyLine The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's geometry
    inline auto const& geometry() const { return m_geometry; }

    /// @brief Get the edge's angle, in radians, between the source and target nodes
    /// @return double The edge's angle, in radians
    inline auto angle() const { return m_angle; }
    /// @brief Get the edge's attributes
    /// @return std::unordered_map<std::string, std::variant<std::monostate, bool, std::int64_t, double, std::string>> The edge's attributes, where the key is the attribute's name and the value is the attribute's value
    inline auto const& attributes() const { return m_attributes; }
    /// @brief Get an attribute of the edge by name
    /// @tparam T The expected type of the attribute's value
    /// @param name The attribute's name
    /// @return std::optional<T> The attribute's value if it exists and can be cast to the expected type, std::nullopt otherwise
    template <typename T>
    inline std::optional<T> getAttribute(std::string const& name) const {
      auto it = m_attributes.find(name);
      if (it == m_attributes.end()) {
        return std::nullopt;
      }
      const T* value = std::get_if<T>(&(it->second));
      return value ? std::optional<T>(*value) : std::nullopt;
    }

    virtual bool isFull() const = 0;

    double deltaAngle(double const previousEdgeAngle) const;
  };
};  // namespace dsf

template <>
struct std::formatter<dsf::Edge> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::Edge& edge, FormatContext&& ctx) const {
    std::string strAttributes;
    for (const auto& [key, value] : edge.attributes()) {
      const std::string strValue = std::visit(
          [](const auto& attributeValue) -> std::string {
            using AttributeType = std::decay_t<decltype(attributeValue)>;
            if constexpr (std::is_same_v<AttributeType, std::monostate>) {
              return "";
            } else {
              return std::format("{}", attributeValue);
            }
          },
          value);
      strAttributes += std::format(" ,{}={}", key, strValue);
    }
    return std::format_to(ctx.out(),
                          "Edge(id={}, source={}, target={}{})",
                          edge.id(),
                          edge.source(),
                          edge.target(),
                          strAttributes);
  }
};