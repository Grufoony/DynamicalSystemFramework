/// @file       /src/dsf/headers/Node.hpp
/// @brief      Defines the Node class.
///
/// @details    The Node class represents the concept of a node in the network.
///             It is a virtual class that needs to be implemented by derived classes.

#pragma once

#include "../geometry/Point.hpp"
#include "../utility/queue.hpp"
#include "../utility/Typedef.hpp"

#include <functional>
#include <utility>
#include <stdexcept>
#include <optional>
#include <map>
#include <format>
#include <cassert>
#include <string>
#include <unordered_map>
#include <variant>

namespace dsf {
  /// @brief The Node class represents the concept of a node in the network.
  class Node {
  protected:
    Id m_id;
    std::optional<geometry::Point> m_geometry;
    std::string m_name;
    std::vector<Id> m_ingoingEdges;
    std::vector<Id> m_outgoingEdges;
    std::unordered_map<
        std::string,
        std::variant<std::monostate, bool, std::int64_t, double, std::string>>
        m_attributes;

  public:
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    explicit Node(Id id) : m_id{id}, m_name{""} {}
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    /// @param point A geometry::Point containing the node's coordinates
    Node(Id id, geometry::Point point)
        : m_id{id}, m_geometry{std::move(point)}, m_name{""} {}

    Node(Node const& other)
        : m_id{other.m_id},
          m_geometry{other.m_geometry},
          m_name{other.m_name},
          m_ingoingEdges{other.m_ingoingEdges},
          m_outgoingEdges{other.m_outgoingEdges},
          m_attributes{other.m_attributes} {}
    virtual ~Node() = default;

    Node& operator=(Node const& other) {
      if (this != &other) {
        m_id = other.m_id;
        m_geometry = other.m_geometry;
        m_name = other.m_name;
        m_ingoingEdges = other.m_ingoingEdges;
        m_outgoingEdges = other.m_outgoingEdges;
        m_attributes = other.m_attributes;
      }
      return *this;
    }

    /// @brief Set the node's id
    /// @param id The node's id
    inline void setId(Id id) noexcept { m_id = id; }
    /// @brief Set the node's geometry
    /// @param point A geometry::Point containing the node's geometry
    inline void setGeometry(geometry::Point point) noexcept {
      m_geometry = std::move(point);
    }
    /// @brief Set the node's name
    /// @param name The node's name
    inline void setName(const std::string& name) noexcept { m_name = name; }
    /// @brief Add an ingoing edge to the node
    /// @param edgeId The edge's id
    /// @throws std::invalid_argument if the edge already exists in the ingoing edges
    inline void addIngoingEdge(Id edgeId) {
      if (std::find(m_ingoingEdges.cbegin(), m_ingoingEdges.cend(), edgeId) !=
          m_ingoingEdges.cend()) {
        throw std::invalid_argument(std::format(
            "Edge with id {} already exists in the incoming edges of node with id {}.",
            edgeId,
            m_id));
      }
      m_ingoingEdges.push_back(edgeId);
    }
    /// @brief Add an outgoing edge to the node
    /// @param edgeId The edge's id
    /// @throws std::invalid_argument if the edge already exists in the outgoing edges
    inline void addOutgoingEdge(Id edgeId) {
      if (std::find(m_outgoingEdges.cbegin(), m_outgoingEdges.cend(), edgeId) !=
          m_outgoingEdges.cend()) {
        throw std::invalid_argument(std::format(
            "Edge with id {} already exists in the outgoing edges of node with id {}.",
            edgeId,
            m_id));
      }
      m_outgoingEdges.push_back(edgeId);
    }
    /// @brief Set an attribute for the node
    /// @param name The attribute's name
    /// @param value The attribute's value
    inline void setAttribute(
        std::string const& name,
        std::variant<std::monostate, bool, std::int64_t, double, std::string> const&
            value) {
      m_attributes[name] = value;
    }

    /// @brief Get the node's id
    /// @return Id The node's id
    inline auto id() const { return m_id; }
    /// @brief Get the node's geometry
    /// @return std::optional<geometry::Point> A geometry::Point
    inline auto const& geometry() const noexcept { return m_geometry; }
    /// @brief Get the node's name
    /// @return std::string The node's name
    inline auto const& name() const noexcept { return m_name; }
    /// @brief Get the node's ingoing edges
    /// @return std::vector<Id> A vector of the node's ingoing edge ids
    inline auto const& ingoingEdges() const noexcept { return m_ingoingEdges; }
    /// @brief Get the node's outgoing edges
    /// @return std::vector<Id> A vector of the node's outgoing edge ids
    inline auto const& outgoingEdges() const noexcept { return m_outgoingEdges; }
    /// @brief Get the node's attributes
    /// @return std::unordered_map<std::string, std::variant<std::monostate, bool, std::int64_t, double, std::string>> A map of the node's attributes, where the key is the attribute's name and the value is the attribute's value
    inline auto const& attributes() const noexcept { return m_attributes; }
    /// @brief Get a specific attribute of the node
    /// @tparam T The type of the attribute's value
    /// @param name The attribute's name
    /// @return std::optional<T> The attribute's value if it exists and can be cast to the specified type, std::nullopt otherwise
    template <typename T>
    inline std::optional<T> getAttribute(std::string const& name) const {
      auto it = m_attributes.find(name);
      if (it == m_attributes.end()) {
        return std::nullopt;
      }
      const T* value = std::get_if<T>(&(it->second));
      return value ? std::optional<T>(*value) : std::nullopt;
    }
  };
};  // namespace dsf
