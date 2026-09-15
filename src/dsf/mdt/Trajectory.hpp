#pragma once

#include "PointsCluster.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <type_traits>
#include <vector>

#include <spdlog/spdlog.h>

namespace dsf::mdt {
  class Trajectory {
  private:
    std::vector<dsf::mdt::PointsCluster> m_points;
    bool m_bSorted{true};

  public:
    Trajectory() = default;

    /// @brief Add a PointsCluster to the trajectory.
    /// @param cluster The PointsCluster to add.
    void addCluster(PointsCluster&& cluster);

    void addCluster(PointsCluster const& cluster);
    /// @brief Add a point with timestamp to the trajectory.
    /// @param timestamp The timestamp of the activity point.
    /// @param point The geometric point of the activity point.
    void addPoint(std::time_t timestamp, dsf::geometry::Point const& point);

    /// @brief Filter the trajectory to identify stop points based on clustering and speed criteria.
    /// @param cluster_radius_km The radius (in kilometers) to use for clustering points.
    /// @param max_speed_kph The max allowed speed (in km/h) to consider a cluster as a stop point.
    void filter(double const cluster_radius_km, double const max_speed_kph);
    /// @brief Sort the trajectory points by timestamp.
    void sort();
    /// @brief Get the number of points in the trajectory.
    /// @return The size of the trajectory.
    inline std::size_t size() const noexcept { return m_points.size(); }
    /// @brief Check if the trajectory is empty.
    /// @return True if the trajectory has no points, false otherwise.
    inline bool empty() const noexcept { return m_points.empty(); }
    /// @brief Get the underlying points map.
    /// @return A const reference to the map of points.
    inline auto const& points() const noexcept { return m_points; }
  };
}  // namespace dsf::mdt