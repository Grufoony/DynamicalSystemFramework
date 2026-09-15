#pragma once

#include "../geometry/Point.hpp"

#include <optional>
#include <vector>

namespace dsf::mdt {
  /// @brief Represents an activity point with a timestamp and a geometric point.
  struct ActivityPoint {
    std::time_t timestamp;
    dsf::geometry::Point point;
  };
  /// @brief Represents a cluster of activity points, providing main methods to manage and analyze them.
  class PointsCluster {
  private:
    mutable std::vector<ActivityPoint> m_points;
    mutable std::optional<dsf::geometry::Point> m_centroid;
    mutable bool m_bSorted{true};
    /// @brief Update the centroid of the cluster based on current activity points.
    /// The centroid is computed as the median of the x and y coordinates of the points.
    /// @throws std::runtime_error if the cluster is empty.
    void m_updateCentroid() const;

  public:
    /// @brief Default constructor for PointsCluster.
    PointsCluster() = default;
    /// @brief Copy constructor for PointsCluster.
    /// @param other The PointsCluster to copy from.
    PointsCluster(PointsCluster const& other) = default;
    /// @brief Copy assignment operator for PointsCluster.
    /// @param other The PointsCluster to copy from.
    /// @return Reference to this PointsCluster.
    PointsCluster& operator=(PointsCluster const& other) = default;
    /// @brief Add an activity point to the cluster.
    /// @param activityPoint The activity point to add.
    void addActivityPoint(ActivityPoint const& activityPoint);
    /// @brief Add a point with timestamp to the cluster.
    /// @param timestamp The timestamp of the activity point.
    /// @param point The geometric point of the activity point.
    void addPoint(std::time_t timestamp, dsf::geometry::Point const& point);
    /// @brief Sort the activity points in the cluster by timestamp.
    void sort() const;
    /// @brief Compute and return the centroid of the cluster.
    /// @return The centroid point of the cluster.
    dsf::geometry::Point centroid() const;
    /// @brief Get the number of activity points in the cluster.
    /// @return The size of the cluster.
    inline std::size_t size() const noexcept { return m_points.size(); }
    /// @brief Check if the cluster is empty.
    /// @return True if the cluster has no activity points, false otherwise.
    inline bool empty() const noexcept { return m_points.empty(); }
    /// @brief Get the underlying activity points.
    /// @return A const reference to the vector of activity points.
    inline std::vector<ActivityPoint> const& points() const noexcept { return m_points; }
    /// @brief Get the timestamp of the first activity point in the cluster.
    /// @return The timestamp of the first activity point.
    /// @throws std::runtime_error if the cluster is empty.
    std::time_t firstTimestamp() const;
    /// @brief Get the timestamp of the last activity point in the cluster.
    /// @return The timestamp of the last activity point.
    /// @throws std::runtime_error if the cluster is empty.
    std::time_t lastTimestamp() const;
    /// @brief Get the duration (in seconds) between the first and last activity points.
    /// @return The duration in seconds.
    /// @throws std::runtime_error if the cluster is empty.
    std::time_t duration() const;
  };
}  // namespace dsf::mdt