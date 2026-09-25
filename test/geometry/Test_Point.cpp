#include "dsf/geometry/Point.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <string>

using namespace dsf::geometry;

TEST_CASE("Point constructors and equality") {
  SUBCASE("Double constructor") {
    Point p(1.5, -2.3);
    CHECK_EQ(p.x(), 1.5);
    CHECK_EQ(p.y(), -2.3);
  }
  SUBCASE("String constructor WKT") {
    Point p("POINT(3.2 4.5)");
    CHECK_EQ(p.x(), 3.2);
    CHECK_EQ(p.y(), 4.5);
  }
  SUBCASE("Invalid string constructor throws") {
    CHECK_THROWS_AS(Point("POINT 3.2 4.5"), std::invalid_argument);
    CHECK_THROWS_AS(Point("POINT(3.2)"), std::invalid_argument);
    CHECK_THROWS_AS(Point("POINT(3.2 4.5)", "GeoJSON"), std::invalid_argument);
  }
  SUBCASE("Equality operator") {
    Point p1(1.0, 2.0);
    Point p2(1.0, 2.0);
    Point p3(1.0, 2.0000001);
    CHECK(p1 == p2);
    CHECK_FALSE(p1 == p3);
  }
}