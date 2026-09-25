#include "dsf/geometry/PolyLine.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <string>

using namespace dsf::geometry;

TEST_CASE("PolyLine constructors and parsing") {
  SUBCASE("Default constructor") {
    PolyLine pl;
    CHECK(pl.empty());
  }
  SUBCASE("Empty WKT LINESTRING") {
    auto emptyPoly = PolyLine("LINESTRING()");
    CHECK(emptyPoly.empty());
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
    PolyLine pl("LINESTRING(1 2, 3 4, 5 6)");
    CHECK_EQ(pl.size(), 3);
    CHECK_EQ(pl[0].x(), 1);
    CHECK_EQ(pl[0].y(), 2);
    CHECK_EQ(pl[1].x(), 3);
    CHECK_EQ(pl[1].y(), 4);
    CHECK_EQ(pl[2].x(), 5);
    CHECK_EQ(pl[2].y(), 6);
  }
  SUBCASE("WKT LINESTRING with extra spaces") {
    PolyLine pl("LINESTRING( 1 2 , 3 4 , 5 6 )");
    CHECK_EQ(pl.size(), 3);
    CHECK_EQ(pl[0].x(), 1);
    CHECK_EQ(pl[0].y(), 2);
    CHECK_EQ(pl[1].x(), 3);
    CHECK_EQ(pl[1].y(), 4);
    CHECK_EQ(pl[2].x(), 5);
    CHECK_EQ(pl[2].y(), 6);
  }
  SUBCASE("Invalid WKT format throws") {
    CHECK_THROWS_AS(PolyLine("LINESTRING(1,2,3,4)"), std::invalid_argument);
    CHECK_THROWS_AS(PolyLine("LINESTRING(1 2 3 4)"), std::invalid_argument);
    CHECK_THROWS_AS(PolyLine("LINESTRING 1 2, 3 4"), std::invalid_argument);
    CHECK_THROWS_AS(PolyLine(std::string{"LINESTRING(1 2, 3 4)"}, "GeoJSON"),
                    std::invalid_argument);
  }
}