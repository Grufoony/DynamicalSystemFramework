/// @file src/dsf/headers/Roundabout.hpp
/// @brief Header file for the Roundabout class

/// @details This file contains the definition of the Roundabout class. The Roundabout class
///          represents a roundabout node in the road network. It is derived from the Node
///          class and has a queue of agents waiting to exit the roundabout.

#pragma once

#include "Agent.hpp"
#include "RoadJunction.hpp"
#include "../utility/queue.hpp"

#include <memory>

namespace dsf::mobility {
  /// @brief The Roundabout class represents a roundabout node in the network.
  class Roundabout : public RoadJunction {
  protected:
    dsf::queue<std::unique_ptr<Agent>> m_agents;

  public:
    /// @brief Construct a new Roundabout object
    /// @param id The node's id
    explicit Roundabout(Id id) : RoadJunction{id} {};
    /// @brief Construct a new Roundabout object
    /// @param id The node's id
    /// @param point A geometry::Point containing the node's coordinates
    Roundabout(Id id, geometry::Point point) : RoadJunction{id, point} {};
    /// @brief Construct a new Roundabout object
    /// @param node An Intersection object
    Roundabout(const RoadJunction& node);

    ~Roundabout() = default;

    /// @brief Put an agent in the node
    /// @param agentId The agent's id
    /// @throws std::runtime_error if the node is full
    void enqueue(std::unique_ptr<Agent> agentId);
    /// @brief Removes the first agent from the node
    /// @return Id The agent's id
    std::unique_ptr<Agent> dequeue();
    /// @brief Get the node's queue
    /// @return dsf::queue<Id> The node's queue
    dsf::queue<std::unique_ptr<Agent>> const& agents() const { return m_agents; }
    /// @brief Returns the node's density
    /// @return double The node's density
    double density() const override {
      return static_cast<double>(m_agents.size()) / this->capacity();
    }
    /// @brief Returns true if the node is full
    /// @return bool True if the node is full
    bool isFull() const override { return m_agents.size() >= this->capacity(); }
    /// @brief Returns true if the node is a roundabout
    /// @return bool True if the node is a roundabout
    constexpr bool isRoundabout() const noexcept final { return true; }
  };
}  // namespace dsf::mobility