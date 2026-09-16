#include "dsf/geometry/Point.hpp"

#include <boost/geometry/algorithms/equals.hpp>

#include <format>
#include <numbers>
#include <stdexcept>
#include <string>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf::geometry;

/// @brief Boost.Geometry points have no operator==; equals() compares coordinates.
static bool same(Point const& lhs, Point const& rhs) {
  return boost::geometry::equals(lhs, rhs);
}

TEST_CASE("Point construction and equality") {
  SUBCASE("Double constructor") {
    Point p(1.5, -2.3);
    CHECK_EQ(p.x(), 1.5);
    CHECK_EQ(p.y(), -2.3);
  }
  SUBCASE("Equality") {
    Point p1(1.0, 2.0);
    Point p2(1.0, 2.0);
    Point p3(1.0, 2.0000001);
    CHECK(same(p1, p2));
    CHECK_FALSE(same(p1, p3));
  }
}

TEST_CASE("Point WKT parsing") {
  SUBCASE("Accepted forms") {
    CHECK(same(pointFromWkt("POINT(3.2 4.5)"), Point(3.2, 4.5)));
    CHECK(same(pointFromWkt("POINT (3.2 4.5)"), Point(3.2, 4.5)));
    CHECK(same(pointFromWkt("POINT(  -1.5   2.0  )"), Point(-1.5, 2.0)));
    // Exact string as produced by the repository's own node fixtures.
    CHECK(same(pointFromWkt("POINT (12.055230700000001 43.790377299999996)"),
               Point(12.055230700000001, 43.790377299999996)));
  }
  SUBCASE("Trailing coordinates are rejected") {
    // Previously this silently yielded (1, 2).
    CHECK_THROWS_AS(pointFromWkt("POINT(1 2 3)"), std::invalid_argument);
  }
  SUBCASE("Wrong or missing keyword is rejected") {
    // Previously the keyword was never validated.
    CHECK_THROWS_AS(pointFromWkt("FOO(1 2)"), std::invalid_argument);
    CHECK_THROWS_AS(pointFromWkt("LINESTRING(1 2)"), std::invalid_argument);
    CHECK_THROWS_AS(pointFromWkt("POINT 1 2"), std::invalid_argument);
    CHECK_THROWS_AS(pointFromWkt(""), std::invalid_argument);
  }
  SUBCASE("Malformed coordinates are rejected") {
    CHECK_THROWS_AS(pointFromWkt("POINT(1 2"), std::invalid_argument);
    CHECK_THROWS_AS(pointFromWkt("POINT(a b)"), std::invalid_argument);
    CHECK_THROWS_AS(pointFromWkt("POINT (1 2) trailing"), std::invalid_argument);
  }
  SUBCASE("Missing coordinates are rejected rather than zero-filled") {
    // boost::geometry::read_wkt substitutes zero for coordinates it cannot find,
    // so on its own it would read "POINT(1)" as (1, 0). The coordinate count check
    // in pointFromWkt rejects that instead of importing a point on the equator.
    CHECK_THROWS_AS(pointFromWkt("POINT(1)"), std::invalid_argument);
    CHECK_THROWS_AS(pointFromWkt("POINT()"), std::invalid_argument);
  }
}

TEST_CASE("Point formatter") {
  // Regression guard: the formatter used to emit "POINT (x, y)" with a comma,
  // which pointFromWkt cannot read back. Coordinates are space-separated WKT.
  SUBCASE("Space-separated, not comma-separated") {
    CHECK_EQ(std::format("{}", Point(12.5, 43.25)), "POINT (12.5 43.25)");
    CHECK_EQ(std::format("{}", Point(-0.099, -0.081)), "POINT (-0.099 -0.081)");
    CHECK_EQ(std::format("{}", Point(8.0, 45.0)), "POINT (8 45)");
  }
  SUBCASE("Full precision is preserved") {
    // boost::geometry::wkt() would truncate this to 6 significant digits.
    CHECK_EQ(std::format("{}", Point(12.055230700000001, 43.790377299999996)),
             "POINT (12.055230700000001 43.790377299999996)");
  }
}

TEST_CASE("Point WKT round-trip") {
  auto const cases = {Point(0.0, 0.0),
                      Point(8.0, 45.0),
                      Point(-0.099, -0.081),
                      Point(12.5, -43.25),
                      Point(12.055230700000001, 43.790377299999996),
                      Point(8.2304888, 45.7131377)};
  for (auto const& p : cases) {
    CAPTURE(std::format("{}", p));
    CHECK(same(p, pointFromWkt(std::format("{}", p))));
  }
}

TEST_CASE("haversine_km") {
  // Points are (longitude, latitude), in degrees.
  constexpr double ONE_DEGREE_KM = 6371.0 * std::numbers::pi / 180.0;  // 111.1949...
  constexpr double HALF_CIRCUMFERENCE_KM = 6371.0 * std::numbers::pi;  // 20015.0868...

  SUBCASE("Identical points") {
    CHECK_EQ(haversine_km(Point(8.0, 45.0), Point(8.0, 45.0)), 0.0);
  }
  SUBCASE("One degree along the equator") {
    CHECK(haversine_km(Point(0.0, 0.0), Point(1.0, 0.0)) ==
          doctest::Approx(ONE_DEGREE_KM).epsilon(1e-6));
  }
  SUBCASE("One degree of latitude") {
    CHECK(haversine_km(Point(0.0, 0.0), Point(0.0, 1.0)) ==
          doctest::Approx(ONE_DEGREE_KM).epsilon(1e-6));
  }
  SUBCASE("Antipodal points") {
    CHECK(haversine_km(Point(0.0, 0.0), Point(180.0, 0.0)) ==
          doctest::Approx(HALF_CIRCUMFERENCE_KM).epsilon(1e-6));
    CHECK(haversine_km(Point(0.0, -90.0), Point(0.0, 90.0)) ==
          doctest::Approx(HALF_CIRCUMFERENCE_KM).epsilon(1e-6));
  }
  SUBCASE("Symmetry") {
    Point const p1(12.0552307, 43.7903773);
    Point const p2(8.2304888, 45.7131377);
    CHECK(haversine_km(p1, p2) == doctest::Approx(haversine_km(p2, p1)));
  }
  SUBCASE("Short distances stay well-conditioned") {
    // Roughly 1 m apart at the equator; the great-circle formula must not
    // collapse to zero.
    Point const p1(0.0, 0.0);
    Point const p2(9.0e-6, 0.0);
    auto const d = haversine_km(p1, p2);
    CHECK(d > 0.0);
    CHECK(d == doctest::Approx(1.0e-3).epsilon(1e-2));
  }
}
