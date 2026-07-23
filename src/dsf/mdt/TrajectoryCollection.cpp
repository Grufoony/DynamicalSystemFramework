#include "TrajectoryCollection.hpp"

#include <format>
#include <fstream>

#include <csv.hpp>
#include <spdlog/spdlog.h>

#include <tbb/parallel_for_each.h>
#include <tbb/concurrent_set.h>

static constexpr std::time_t SECONDS_IN_MINUTE = 60;
static constexpr std::time_t SECONDS_IN_HOUR = 3600;

namespace dsf::mdt {
  TrajectoryCollection::TrajectoryCollection(
      std::unordered_map<
          std::string,
          std::variant<std::vector<Id>, std::vector<std::time_t>, std::vector<double>>>&&
          dataframe,
      std::array<double, 4> const& bbox) {
    auto const& uids = std::get<std::vector<Id>>(dataframe.at("uid"));
    auto const& timestamps =
        std::get<std::vector<std::time_t>>(dataframe.at("timestamp"));
    auto const& lats = std::get<std::vector<double>>(dataframe.at("lat"));
    auto const& lons = std::get<std::vector<double>>(dataframe.at("lon"));

    auto const bbox_set =
        !(bbox[0] == 0.0 && bbox[1] == 0.0 && bbox[2] == 0.0 && bbox[3] == 0.0);

    for (std::size_t i = 0; i < uids.size(); ++i) {
      auto const point = dsf::geometry::Point(lons[i], lats[i]);
      // If bbox is the default-initialized array (all zeros) we treat it as unset.
      if (bbox_set && (point.x() < bbox[0] || point.x() > bbox[2] ||
                       point.y() < bbox[1] || point.y() > bbox[3])) {
        continue;
      }
      if (m_trajectories.find(uids[i]) == m_trajectories.end()) {
        m_trajectories[uids[i]] = std::vector<Trajectory>{};
        m_trajectories[uids[i]].emplace_back();
      }
      m_trajectories[uids[i]][0].addPoint(timestamps[i], point);
    }
  }

  TrajectoryCollection::TrajectoryCollection(
      std::string const& fileName,
      std::unordered_map<std::string, std::string> const& column_mapping,
      char const sep,
      std::array<double, 4> const& bbox) {
    if (!fileName.empty()) {
      this->import(fileName, column_mapping, sep, bbox);
    }
  }

  void TrajectoryCollection::import(
      std::string const& fileName,
      std::unordered_map<std::string, std::string> const& column_mapping,
      char const sep,
      std::array<double, 4> const& bbox) {
    csv::CSVFormat csvFormat;
    csvFormat.delimiter(sep);
    csv::CSVReader reader(fileName, csvFormat);

    std::unordered_map<std::string, std::string> column_names = {
        {"uid", "uid"}, {"timestamp", "timestamp"}, {"lat", "lat"}, {"lon", "lon"}};
    for (auto const& [key, value] : column_mapping) {
      if (!column_names.contains(key)) {
        spdlog::warn("Ignoring unknown column mapping key: {}", key);
        continue;
      }
      column_names[key] = value;
    }

    std::vector<Id> uids;
    std::vector<std::time_t> timestamps;
    std::vector<double> lats;
    std::vector<double> lons;
    for (auto& row : reader) {
      uids.push_back(static_cast<Id>(row[column_names.at("uid")].get<long long>()));
      timestamps.push_back(
          static_cast<std::time_t>(row[column_names.at("timestamp")].get<long long>()));
      lats.push_back(row[column_names.at("lat")].get<double>());
      lons.push_back(row[column_names.at("lon")].get<double>());
    }

    std::unordered_map<
        std::string,
        std::variant<std::vector<Id>, std::vector<std::time_t>, std::vector<double>>>
        dataframe;
    dataframe["uid"] = std::move(uids);
    dataframe["timestamp"] = std::move(timestamps);
    dataframe["lat"] = std::move(lats);
    dataframe["lon"] = std::move(lons);
    *this = TrajectoryCollection(std::move(dataframe), bbox);
  }

  void TrajectoryCollection::filter(double const cluster_radius_km,
                                    double const max_speed_kph,
                                    std::size_t const min_points_per_trajectory,
                                    std::optional<std::time_t> const min_duration_min) {
    // Collect IDs to remove in parallel
    tbb::concurrent_set<Id> to_remove;
    tbb::concurrent_set<Id> to_split;

    // Returns true if speed between two points is below max_speed_kph
    auto check_max_speed = [&max_speed_kph](PointsCluster const& currentCluster,
                                            PointsCluster const& previousCluster) {
      auto const distance_km = dsf::geometry::haversine_km(currentCluster.centroid(),
                                                           previousCluster.centroid());
      auto const current_time =
          (currentCluster.lastTimestamp() + currentCluster.firstTimestamp()) * 0.5;
      auto const previous_time =
          (previousCluster.lastTimestamp() + previousCluster.firstTimestamp()) * 0.5;
      if (current_time < previous_time) {
        // Should never happen if data is clean
        throw std::runtime_error(
            "Timestamps are not in increasing order within the trajectory.");
      }
      if (current_time == previous_time) {
        spdlog::debug(
            "Non-increasing timestamps detected. Skipping speed check for these points.");
        return true;
      }
      auto const speed_kph =
          (distance_km * SECONDS_IN_HOUR) / (current_time - previous_time);
      return speed_kph <= max_speed_kph;
    };
    // Returns true if cluster duration is below min_duration_min
    auto check_min_duration =
        [&min_duration_min](dsf::mdt::PointsCluster const& cluster) {
          if (!min_duration_min.has_value()) {
            return true;
          }
          return cluster.duration() < (*min_duration_min) * SECONDS_IN_MINUTE;
        };

    tbb::parallel_for_each(
        m_trajectories.begin(),
        m_trajectories.end(),
        [&to_remove,
         &to_split,
         &check_max_speed,
         &check_min_duration,
         min_points_per_trajectory,
         cluster_radius_km,
         max_speed_kph](auto& pair) {
          auto const& uid = pair.first;
          auto& trajectory =
              pair.second
                  [0];  // By now, each trajectory has only one segment as they were not split yet
          if (min_points_per_trajectory > 0 &&
              trajectory.size() < min_points_per_trajectory) {
            to_remove.insert(uid);
            return;
          }
          trajectory.filter(cluster_radius_km, max_speed_kph);
          if (min_points_per_trajectory > 0 &&
              trajectory.size() < min_points_per_trajectory) {
            to_remove.insert(uid);
            return;
          }
          auto const& points{trajectory.points()};
          auto const nPoints = points.size();
          for (std::size_t i = 0; i < nPoints;) {
            auto const& currentCluster = points[i];
            if (!check_min_duration(currentCluster)) {
              to_split.insert(uid);
              return;
            }
            if (++i < nPoints) {
              auto const& nextCluster = points[i];
              if (!check_max_speed(nextCluster, currentCluster)) {
                to_split.insert(uid);
                return;
              }
            }
          }
        });

    // Remove trajectories sequentially (fast for unordered_map)
    spdlog::info(
        "Removing {} ({:.2f}%) trajectories that do not meet the minimum points "
        "requirement after filtering.",
        to_remove.size(),
        (to_remove.size() * 100.0 / m_trajectories.size()));
    std::erase_if(m_trajectories, [&to_remove](auto const& pair) {
      return to_remove.contains(pair.first);
    });

    spdlog::info(
        "Splitting {} trajectories based on minimum duration or maximum speed "
        "requirements.",
        to_split.size());
    for (auto const& uid : to_split) {
      // Extract the trajectory
      if (!m_trajectories.contains(uid)) {
        continue;
      }
      auto& trajectories = m_trajectories.at(uid);
      auto originalTrajectory = std::move(trajectories[0]);
      trajectories.clear();

      Trajectory newTrajectory;
      auto const& points{originalTrajectory.points()};
      auto const nPoints = points.size();
      for (std::size_t i = 0; i < nPoints;) {
        auto const& currentCluster = points[i];
        newTrajectory.addCluster(currentCluster);

        bool bShouldSplit = false;

        if (++i < nPoints) {
          auto const& nextCluster = points[i];
          bShouldSplit = !check_max_speed(nextCluster, currentCluster);
        }
        if (!bShouldSplit) {
          bShouldSplit = !check_min_duration(currentCluster);
        }
        // If constraint violated (max speed or min duration) - finalize current trajectory and start a new one
        if (bShouldSplit && !newTrajectory.empty()) {
          if (newTrajectory.size() >= min_points_per_trajectory) {
            trajectories.emplace_back(std::move(newTrajectory));
          }
          newTrajectory = Trajectory();
          newTrajectory.addCluster(currentCluster);
        }
      }
      if (newTrajectory.size() >= min_points_per_trajectory) {
        trajectories.emplace_back(std::move(newTrajectory));
      }
    }
  }
  void TrajectoryCollection::to_csv(std::string const& fileName, char const sep) const {
    std::ofstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }

    auto const HEADER_LINE =
        std::format("uid{}trajectory_id{}lon{}lat{}timestamp_in{}timestamp_out\n",
                    sep,
                    sep,
                    sep,
                    sep,
                    sep);
    // Write CSV header
    file << HEADER_LINE;

    for (auto const& [uid, trajectories] : m_trajectories) {
      std::size_t trajIdx = 0;
      for (auto const& trajectory : trajectories) {
        for (auto const& cluster : trajectory.points()) {
          auto const centroid = cluster.centroid();
          file << uid << sep << trajIdx << sep << centroid.x() << sep << centroid.y()
               << sep << cluster.firstTimestamp() << sep << cluster.lastTimestamp()
               << "\n";
        }
        ++trajIdx;
      }
    }
  }
}  // namespace dsf::mdt