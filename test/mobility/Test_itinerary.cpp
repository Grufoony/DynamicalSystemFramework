#include <cstdint>

#include "dsf/mobility/Itinerary.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf;
using namespace dsf::mobility;

TEST_CASE("Itinerary") {
  SUBCASE("Constructors") {
    dsf::Id itineraryId{0};
    dsf::Id destinationId{2};
    Itinerary itinerary{itineraryId, destinationId};
    CHECK_EQ(itinerary.id(), itineraryId);
    CHECK_EQ(itinerary.destination(), destinationId);
  }

  SUBCASE("Set and get path") {
    Itinerary itinerary{1, 42};
    PathCollection path = {{1, {2, 3}}, {2, {4}}, {3, {5, 6, 7}}};
    itinerary.setNodePath(path);
    auto const& result = itinerary.nodePath();
    CHECK_EQ(result.size(), path.size());
    for (const auto& [k, v] : path) {
      CHECK(result.count(k) == 1);
      CHECK(result.at(k) == v);
    }
  }

  SUBCASE("Save and load itinerary") {
    Itinerary itinerary{7, 99};
    PathCollection nodePath = {{7, {8, 9}}, {8, {10}}, {9, {11, 12}}};
    itinerary.setNodePath(nodePath);
    const std::string filename = "test_itinerary.bin";
    itinerary.save(filename);

    Itinerary loaded{7, 0};  // destination will be overwritten by load
    loaded.load(filename);
    CHECK_EQ(loaded.destination(), 99);
    auto const& loadedPath = loaded.nodePath();
    CHECK_EQ(loadedPath.size(), nodePath.size());
    for (const auto& [k, v] : nodePath) {
      CHECK(loadedPath.count(k) == 1);
      CHECK(loadedPath.at(k) == v);
    }
    // Clean up
    std::remove(filename.c_str());
  }

  SUBCASE("Load from non-existent file throws") {
    Itinerary itinerary{1, 2};
    CHECK_THROWS_AS(itinerary.load("nonexistent_file.bin"), std::runtime_error);
  }

  SUBCASE("Empty path") {
    Itinerary itinerary{123, 456};
    itinerary.setNodePath({});
    CHECK(itinerary.nodePath().empty());
    const std::string filename = "test_empty_path.bin";
    itinerary.save(filename);
    Itinerary loaded{123, 0};
    loaded.load(filename);
    CHECK(loaded.nodePath().empty());
    std::remove(filename.c_str());
  }

  SUBCASE("Large path") {
    Itinerary itinerary{100, 200};
    PathCollection path;
    for (dsf::Id i = 0; i < 100; ++i) {
      std::vector<dsf::Id> v;
      for (dsf::Id j = 0; j < 10; ++j)
        v.push_back(i * 10 + j);
      path[i] = v;
    }
    itinerary.setNodePath(path);
    CHECK_EQ(itinerary.nodePath().size(), 100);
    for (dsf::Id i = 0; i < 100; ++i) {
      CHECK(itinerary.nodePath().count(i) == 1);
      CHECK(itinerary.nodePath().at(i).size() == 10);
    }
    const std::string filename = "test_large_path.bin";
    itinerary.save(filename);
    Itinerary loaded{100, 0};
    loaded.load(filename);
    CHECK_EQ(loaded.nodePath().size(), 100);
    for (dsf::Id i = 0; i < 100; ++i) {
      CHECK(loaded.nodePath().count(i) == 1);
      CHECK(loaded.nodePath().at(i).size() == 10);
      CHECK(loaded.nodePath().at(i) == path[i]);
    }
    std::remove(filename.c_str());
  }
}
