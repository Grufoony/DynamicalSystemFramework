#include "dsf/mdt/PointsCluster.hpp"

#include <boost/geometry/algorithms/equals.hpp>

#include <ctime>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf::mdt;
using namespace dsf::geometry;

/// @brief Boost.Geometry points have no operator==; equals() compares coordinates.
static bool same(Point const& lhs, Point const& rhs) {
  return boost::geometry::equals(lhs, rhs);
}

TEST_CASE("PointsCluster - Default constructor") {
  PointsCluster cluster;
  CHECK(cluster.empty());
  CHECK_EQ(cluster.size(), 0);
}

TEST_CASE("PointsCluster - addActivityPoint") {
  PointsCluster cluster;
  ActivityPoint ap{1000, Point(10.0, 20.0)};

  cluster.addActivityPoint(ap);

  CHECK_FALSE(cluster.empty());
  CHECK_EQ(cluster.size(), 1);
  CHECK_EQ(cluster.points().size(), 1);
  CHECK_EQ(cluster.points()[0].timestamp, 1000);
  CHECK(same(cluster.points()[0].point, Point(10.0, 20.0)));
}

TEST_CASE("PointsCluster - addPoint") {
  PointsCluster cluster;

  cluster.addPoint(2000, Point(15.0, 25.0));

  CHECK_EQ(cluster.size(), 1);
  CHECK_EQ(cluster.points()[0].timestamp, 2000);
  CHECK(same(cluster.points()[0].point, Point(15.0, 25.0)));
}

TEST_CASE("PointsCluster - addPoint multiple times") {
  PointsCluster cluster;

  cluster.addPoint(1000, Point(10.0, 20.0));
  cluster.addPoint(2000, Point(15.0, 25.0));
  cluster.addPoint(1500, Point(12.0, 22.0));

  CHECK_EQ(cluster.size(), 3);
}

TEST_CASE("PointsCluster - sort by timestamp") {
  PointsCluster cluster;

  // Add points in non-chronological order
  cluster.addPoint(3000, Point(10.0, 20.0));
  cluster.addPoint(1000, Point(15.0, 25.0));
  cluster.addPoint(2000, Point(12.0, 22.0));

  cluster.sort();

  auto const& points = cluster.points();
  CHECK_EQ(points[0].timestamp, 1000);
  CHECK_EQ(points[1].timestamp, 2000);
  CHECK_EQ(points[2].timestamp, 3000);
}

TEST_CASE("PointsCluster - sort is idempotent") {
  PointsCluster cluster;

  cluster.addPoint(3000, Point(10.0, 20.0));
  cluster.addPoint(1000, Point(15.0, 25.0));
  cluster.addPoint(2000, Point(12.0, 22.0));

  cluster.sort();
  cluster.sort();  // Sort again

  auto const& points = cluster.points();
  CHECK_EQ(points[0].timestamp, 1000);
  CHECK_EQ(points[1].timestamp, 2000);
  CHECK_EQ(points[2].timestamp, 3000);
}

TEST_CASE("PointsCluster - centroid with single point") {
  PointsCluster cluster;
  cluster.addPoint(1000, Point(10.0, 20.0));

  Point centroid = cluster.centroid();

  CHECK_EQ(centroid.x(), 10.0);
  CHECK_EQ(centroid.y(), 20.0);
}

TEST_CASE("PointsCluster - centroid with odd number of points") {
  PointsCluster cluster;

  // Three points: median should be the middle value
  cluster.addPoint(1000, Point(1.0, 10.0));
  cluster.addPoint(2000, Point(2.0, 20.0));
  cluster.addPoint(3000, Point(3.0, 30.0));

  Point centroid = cluster.centroid();

  // Median of [1.0, 2.0, 3.0] is 2.0
  // Median of [10.0, 20.0, 30.0] is 20.0
  CHECK_EQ(centroid.x(), 2.0);
  CHECK_EQ(centroid.y(), 20.0);
}

TEST_CASE("PointsCluster - centroid with even number of points") {
  PointsCluster cluster;

  // Four points: median should be average of two middle values
  cluster.addPoint(1000, Point(1.0, 10.0));
  cluster.addPoint(2000, Point(2.0, 20.0));
  cluster.addPoint(3000, Point(3.0, 30.0));
  cluster.addPoint(4000, Point(4.0, 40.0));

  Point centroid = cluster.centroid();

  // Median of [1.0, 2.0, 3.0, 4.0] is (2.0 + 3.0) / 2 = 2.5
  // Median of [10.0, 20.0, 30.0, 40.0] is (20.0 + 30.0) / 2 = 25.0
  CHECK_EQ(centroid.x(), 2.5);
  CHECK_EQ(centroid.y(), 25.0);
}

TEST_CASE("PointsCluster - centroid is cached") {
  PointsCluster cluster;

  cluster.addPoint(1000, Point(1.0, 10.0));
  cluster.addPoint(2000, Point(2.0, 20.0));
  cluster.addPoint(3000, Point(3.0, 30.0));

  Point centroid1 = cluster.centroid();
  Point centroid2 = cluster.centroid();

  // Should return the same centroid
  CHECK(same(centroid1, centroid2));
}

TEST_CASE("PointsCluster - centroid is reset when adding points") {
  PointsCluster cluster;

  cluster.addPoint(1000, Point(1.0, 10.0));
  cluster.addPoint(2000, Point(2.0, 20.0));

  Point centroid1 = cluster.centroid();

  // Add another point
  cluster.addPoint(3000, Point(3.0, 30.0));

  Point centroid2 = cluster.centroid();

  // Centroid should be different now
  CHECK_FALSE(same(centroid1, centroid2));
}

TEST_CASE("PointsCluster - centroid with unsorted points") {
  PointsCluster cluster;

  // Add points in non-chronological order
  cluster.addPoint(3000, Point(3.0, 30.0));
  cluster.addPoint(1000, Point(1.0, 10.0));
  cluster.addPoint(2000, Point(2.0, 20.0));

  Point centroid = cluster.centroid();

  // Centroid computation should sort the points first
  CHECK_EQ(centroid.x(), 2.0);
  CHECK_EQ(centroid.y(), 20.0);
}

TEST_CASE("PointsCluster - centroid throws on empty cluster") {
  PointsCluster cluster;

  CHECK_THROWS_AS(cluster.centroid(), std::runtime_error);
  CHECK_THROWS_WITH(cluster.centroid(),
                    "Cannot compute centroid of an empty PointsCluster.");
}

TEST_CASE("PointsCluster - firstTimestamp") {
  PointsCluster cluster;

  cluster.addPoint(3000, Point(10.0, 20.0));
  cluster.addPoint(1000, Point(15.0, 25.0));
  cluster.addPoint(2000, Point(12.0, 22.0));

  CHECK_EQ(cluster.firstTimestamp(), 1000);
}

TEST_CASE("PointsCluster - firstTimestamp throws on empty cluster") {
  PointsCluster cluster;

  CHECK_THROWS_AS(cluster.firstTimestamp(), std::runtime_error);
  CHECK_THROWS_WITH(cluster.firstTimestamp(),
                    "PointsCluster is empty, no first timestamp available.");
}

TEST_CASE("PointsCluster - lastTimestamp") {
  PointsCluster cluster;

  cluster.addPoint(3000, Point(10.0, 20.0));
  cluster.addPoint(1000, Point(15.0, 25.0));
  cluster.addPoint(2000, Point(12.0, 22.0));

  CHECK_EQ(cluster.lastTimestamp(), 3000);
}

TEST_CASE("PointsCluster - lastTimestamp throws on empty cluster") {
  PointsCluster cluster;

  CHECK_THROWS_AS(cluster.lastTimestamp(), std::runtime_error);
  CHECK_THROWS_WITH(cluster.lastTimestamp(),
                    "PointsCluster is empty, no last timestamp available.");
}

TEST_CASE("PointsCluster - duration") {
  PointsCluster cluster;

  cluster.addPoint(1000, Point(10.0, 20.0));
  cluster.addPoint(3000, Point(15.0, 25.0));
  cluster.addPoint(2000, Point(12.0, 22.0));

  // Duration should be 3000 - 1000 = 2000 seconds
  CHECK_EQ(cluster.duration(), 2000);
}

TEST_CASE("PointsCluster - duration with single point") {
  PointsCluster cluster;

  cluster.addPoint(1000, Point(10.0, 20.0));

  // Duration should be 0 for a single point
  CHECK_EQ(cluster.duration(), 0);
}

TEST_CASE("PointsCluster - duration throws on empty cluster") {
  PointsCluster cluster;

  CHECK_THROWS_AS(cluster.duration(), std::runtime_error);
  CHECK_THROWS_WITH(cluster.duration(), "PointsCluster is empty, no duration available.");
}

TEST_CASE("PointsCluster - points accessor") {
  PointsCluster cluster;

  cluster.addPoint(1000, Point(10.0, 20.0));
  cluster.addPoint(2000, Point(15.0, 25.0));

  auto const& points = cluster.points();

  CHECK_EQ(points.size(), 2);
  // Note: points might not be sorted yet
}

TEST_CASE("PointsCluster - comprehensive workflow") {
  PointsCluster cluster;

  // Start with empty cluster
  CHECK(cluster.empty());

  // Add several points
  cluster.addPoint(5000, Point(10.0, 20.0));
  cluster.addPoint(2000, Point(12.0, 22.0));
  cluster.addPoint(8000, Point(14.0, 24.0));
  cluster.addPoint(3000, Point(11.0, 21.0));

  CHECK_EQ(cluster.size(), 4);
  CHECK_FALSE(cluster.empty());

  // Check timestamps work correctly (should auto-sort)
  CHECK_EQ(cluster.firstTimestamp(), 2000);
  CHECK_EQ(cluster.lastTimestamp(), 8000);
  CHECK_EQ(cluster.duration(), 6000);

  // Verify points are sorted after timestamp access
  cluster.sort();
  auto const& points = cluster.points();
  CHECK_EQ(points[0].timestamp, 2000);
  CHECK_EQ(points[1].timestamp, 3000);
  CHECK_EQ(points[2].timestamp, 5000);
  CHECK_EQ(points[3].timestamp, 8000);

  // Check centroid computation
  Point centroid = cluster.centroid();
  // Median of x: [10.0, 11.0, 12.0, 14.0] = (11.0 + 12.0) / 2 = 11.5
  // Median of y: [20.0, 21.0, 22.0, 24.0] = (21.0 + 22.0) / 2 = 21.5
  CHECK_EQ(centroid.x(), 11.5);
  CHECK_EQ(centroid.y(), 21.5);
}
