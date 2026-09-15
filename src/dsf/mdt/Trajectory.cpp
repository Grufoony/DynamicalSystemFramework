#include "Trajectory.hpp"

#include <tbb/parallel_sort.h>

namespace dsf::mdt {
  void Trajectory::addCluster(PointsCluster&& cluster) {
    m_points.push_back(std::move(cluster));
    m_bSorted = false;
  }
  void Trajectory::addCluster(PointsCluster const& cluster) {
    m_points.push_back(cluster);
    m_bSorted = false;
  }
  void Trajectory::addPoint(std::time_t timestamp, dsf::geometry::Point const& point) {
    // Create a new PointsCluster containing the single activity point and add it
    PointsCluster cluster;
    cluster.addPoint(timestamp, point);
    addCluster(std::move(cluster));
  }

  void Trajectory::filter(double const cluster_radius_km, double const max_speed_kph) {
    this->sort();
    auto rawPoints = std::move(m_points);
    m_points.clear();
    if (rawPoints.empty()) {
      return;
    }

    std::vector<PointsCluster> clusterCandidates;

    auto it = rawPoints.begin();
    PointsCluster currentCluster;
    currentCluster.addPoint(it->firstTimestamp(), it->centroid());
    ++it;
    for (; it != rawPoints.end(); ++it) {
      std::time_t timestamp = it->firstTimestamp();
      dsf::geometry::Point const& point = it->centroid();

      // Compute distance from current point to cluster centroid
      double const distance =
          dsf::geometry::haversine_km(currentCluster.centroid(), point);

      if (distance < cluster_radius_km) {
        // Add point to current cluster
        currentCluster.addPoint(timestamp, point);
      } else {
        // Distance exceeds threshold - finalize current cluster and start new one
        if (!currentCluster.empty()) {
          clusterCandidates.push_back(currentCluster);
        }

        // Start new cluster with current point
        currentCluster = PointsCluster();
        currentCluster.addPoint(timestamp, point);
      }
    }

    // Handle the last cluster
    if (!currentCluster.empty()) {
      clusterCandidates.push_back(currentCluster);
    }

    // Apply speed filtering: only keep clusters with average speed below maxSpeed
    // Speed is computed from first to last point in the cluster
    for (auto const& cluster : clusterCandidates) {
      if (cluster.size() < 2) {
        // Single point cluster - always consider it a stop point (speed = 0)
        m_points.push_back(cluster);
        continue;
      }
      cluster.sort();

      // Compute average speed for this cluster
      // Speed = distance / time (from first to last point in cluster)
      std::time_t const duration = cluster.duration();  // in seconds

      if (duration > 0) {
        // Calculate distance traveled from first to last point
        dsf::geometry::Point const firstPt = cluster.points().front().point;
        dsf::geometry::Point const lastPt = cluster.points().back().point;
        double const distanceTraveled =
            dsf::geometry::haversine_km(firstPt, lastPt);  // in kilometers

        // Compute average speed in km/h
        double const avgSpeedKPH =
            distanceTraveled / static_cast<double>(duration) * 3600.0;

        // Only add clusters where average speed is below maxSpeed threshold
        // These represent stop points or slow-moving areas
        if (avgSpeedKPH < max_speed_kph) {
          m_points.push_back(cluster);
        }
      } else {
        // Duration is 0 - all points have same timestamp, treat as stopped
        m_points.push_back(cluster);
      }
    }
  }

  void Trajectory::sort() {
    if (m_bSorted) {
      return;
    }
    tbb::parallel_sort(m_points.begin(),
                       m_points.end(),
                       [](PointsCluster const& a, PointsCluster const& b) {
                         return a.firstTimestamp() < b.firstTimestamp();
                       });
    m_bSorted = true;
  }
}  // namespace dsf::mdt