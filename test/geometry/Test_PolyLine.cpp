#include "dsf/geometry/PolyLine.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <format>
#include <stdexcept>
#include <string>

using namespace dsf::geometry;

TEST_CASE("PolyLine constructors and parsing") {
  SUBCASE("Default constructor") {
    PolyLine pl;
    CHECK(pl.empty());
  }
  SUBCASE("Empty WKT LINESTRING") {
    auto emptyPoly = polyLineFromWkt("LINESTRING()");
    CHECK(emptyPoly.empty());
    CHECK(polyLineFromWkt("LINESTRING ( )").empty());
  }
  SUBCASE("Initializer list constructor") {
    PolyLine pl{Point(1, 2), Point(3, 4)};
    CHECK_EQ(pl.size(), 2);
    CHECK_EQ(pl[0].x(), 1);
    CHECK_EQ(pl[0].y(), 2);
    CHECK_EQ(pl[1].x(), 3);
    CHECK_EQ(pl[1].y(), 4);
  }
  SUBCASE("WKT LINESTRING constructor") {
    auto pl = polyLineFromWkt("LINESTRING(1 2, 3 4, 5 6)");
    CHECK_EQ(pl.size(), 3);
    CHECK_EQ(pl[0].x(), 1);
    CHECK_EQ(pl[0].y(), 2);
    CHECK_EQ(pl[1].x(), 3);
    CHECK_EQ(pl[1].y(), 4);
    CHECK_EQ(pl[2].x(), 5);
    CHECK_EQ(pl[2].y(), 6);
  }
  SUBCASE("WKT LINESTRING with extra spaces") {
    auto pl = polyLineFromWkt("LINESTRING( 1 2 , 3 4 , 5 6 )");
    CHECK_EQ(pl.size(), 3);
    CHECK_EQ(pl[0].x(), 1);
    CHECK_EQ(pl[0].y(), 2);
    CHECK_EQ(pl[1].x(), 3);
    CHECK_EQ(pl[1].y(), 4);
    CHECK_EQ(pl[2].x(), 5);
    CHECK_EQ(pl[2].y(), 6);
  }
  SUBCASE("Invalid WKT format throws") {
    CHECK_THROWS_AS(polyLineFromWkt("LINESTRING(1 2, 3"), std::invalid_argument);
    CHECK_THROWS_AS(polyLineFromWkt("LINESTRING(a b, c d)"), std::invalid_argument);
    CHECK_THROWS_AS(polyLineFromWkt("LINESTRING (1 2, 3 4) trailing"),
                    std::invalid_argument);
  }
  SUBCASE("Missing coordinates are rejected rather than zero-filled") {
    // boost::geometry::read_wkt substitutes zero for coordinates it cannot find, so
    // on its own it would read "LINESTRING(1,2,3,4)" as the four points (1 0),
    // (2 0), (3 0) and (4 0). The coordinate count check in polyLineFromWkt rejects
    // that instead of importing a line along the equator.
    CHECK_THROWS_AS(polyLineFromWkt("LINESTRING(1,2,3,4)"), std::invalid_argument);
    CHECK_THROWS_AS(polyLineFromWkt("LINESTRING(1 2, 3)"), std::invalid_argument);
  }
  SUBCASE("Wrong or missing keyword is rejected") {
    CHECK_THROWS_AS(polyLineFromWkt("FOO(1 2, 3 4)"), std::invalid_argument);
    CHECK_THROWS_AS(polyLineFromWkt("POINT(1 2)"), std::invalid_argument);
    CHECK_THROWS_AS(polyLineFromWkt(""), std::invalid_argument);
  }
  SUBCASE("Comma separators are optional") {
    // Deliberate behaviour change: the previous hand-written parser rejected a
    // point group without a separating comma, whereas boost::geometry::read_wkt
    // treats commas as optional. "LINESTRING(1 2 3 4)" is therefore read as the
    // two points (1 2) and (3 4) instead of throwing.
    auto const pl = polyLineFromWkt("LINESTRING(1 2 3 4)");
    CHECK_EQ(pl.size(), 2);
    CHECK_EQ(pl[0].x(), 1);
    CHECK_EQ(pl[0].y(), 2);
    CHECK_EQ(pl[1].x(), 3);
    CHECK_EQ(pl[1].y(), 4);
  }
}

TEST_CASE("PolyLine formatter") {
  SUBCASE("Well-formed WKT") {
    CHECK_EQ(std::format("{}", PolyLine{Point(1, 2), Point(3, 4)}),
             "LINESTRING (1 2, 3 4)");
    CHECK_EQ(std::format("{}", PolyLine{}), "LINESTRING ()");
  }
  SUBCASE("Full precision is preserved") {
    // boost::geometry::wkt() would truncate these to 6 significant digits.
    CHECK_EQ(std::format("{}",
                         PolyLine{Point(12.055230700000001, 43.790377299999996),
                                  Point(8.2304888, 45.7131377)}),
             "LINESTRING (12.055230700000001 43.790377299999996, "
             "8.2304888 45.7131377)");
  }
}

TEST_CASE("PolyLine WKT round-trip") {
  PolyLine const pl{Point(12.055230700000001, 43.790377299999996),
                    Point(8.2304888, 45.7131377),
                    Point(-0.099, -0.081)};
  auto const parsed = polyLineFromWkt(std::format("{}", pl));
  REQUIRE_EQ(parsed.size(), pl.size());
  for (std::size_t i = 0; i < pl.size(); ++i) {
    CHECK_EQ(parsed[i].x(), pl[i].x());
    CHECK_EQ(parsed[i].y(), pl[i].y());
  }
}
