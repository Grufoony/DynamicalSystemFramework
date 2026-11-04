#pragma once

#include "Trajectory.hpp"
#include "../utility/Typedef.hpp"

#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <ankerl/unordered_dense.h>

namespace dsf::mdt {
  class TrajectoryCollection {
  private:
    ankerl::unordered_dense::map<Id, std::vector<Trajectory>> m_trajectories;

  public:
    /// @brief Construct a TrajectoryCollection from a dataframe.
    /// @param dataframe An unordered_map representing the dataframe with columns: 'uid',
    /// 'timestamp', 'lat', 'lon'.
    /// @param bbox Optional bounding box [minX, minY, maxX, maxY] to limit the area of interest. Default is empty (no bounding box).
    TrajectoryCollection(
        std::unordered_map<
            std::string,
            std::variant<std::vector<Id>, std::vector<std::time_t>, std::vector<double>>>&&
            dataframe,
        std::array<double, 4> const& bbox = {});
    /// @brief Construct a TrajectoryCollection, optionally importing from a CSV file.
    /// @param fileName The path to the CSV file.
    /// @param column_mapping A mapping of column names.
    /// @param sep The character used to separate values in the CSV file.
    /// @param bbox Optional bounding box [minX, minY, maxX, maxY] to limit the area of interest. Default is empty (no bounding box).
    TrajectoryCollection(
        std::string const& fileName = std::string(),
        std::unordered_map<std::string, std::string> const& column_mapping = {},
        char const sep = ';',
        std::array<double, 4> const& bbox = {});

    /// @brief Import trajectories from a CSV file.
    /// @param fileName The path to the CSV file. Accepts columns: 'uid', 'timestamp', 'lat', 'lon'.
    /// @param column_mapping A mapping of column names.
    /// @param sep The character used to separate values in the CSV file.
    /// @param bbox Optional bounding box [minX, minY, maxX, maxY] to limit the area of interest. Default is empty (no bounding box).
    void import(std::string const& fileName,
                std::unordered_map<std::string, std::string> const& column_mapping = {},
                char const sep = ';',
                std::array<double, 4> const& bbox = {});
    /// @brief Export clustered trajectories to a CSV file with columns 'uid', 'trajectory_id',
    /// 'lon', 'lat', 'timestamp_in', 'timestamp_out'.
    /// @param fileName The path to the output CSV file.
    /// @param sep The character used to separate values in the CSV file.
    void to_csv(std::string const& fileName, char const sep = ';') const;
    /// @brief Filter all point trajectories to identify stop points based on clustering and speed
    /// criteria.
    /// @param cluster_radius_km The radius (in kilometers) to use for clustering points.
    /// @param max_speed_kph The max allowed speed (in km/h) to consider a cluster as a stop point. Default is 150.0 km/h.
    /// @param min_points_per_trajectory The minimum number of points required for a trajectory to be considered valid. Default is 2.
    /// @param min_duration_min The minimum duration (in minutes) for a cluster to be considered a stop point.
    /// If stops are detected, trajectories may be split into multiple segments.
    void filter(double const cluster_radius_km,
                double const max_speed_kph = 150.0,
                std::size_t const min_points_per_trajectory = 2,
                std::optional<std::time_t> const min_duration_min = std::nullopt);
    /// @brief Get the underlying trajectories map.
    /// @return A const reference to the map of trajectories.
    inline auto const& trajectories() const noexcept { return m_trajectories; }
  };
}  // namespace dsf::mdt