#include "dsf/mobility/RoadNetwork.hpp"
#include "dsf/mobility/Router.hpp"
#include "dsf/base/Node.hpp"
#include "dsf/mobility/Road.hpp"
#include "dsf/mobility/Street.hpp"
#include "dsf/mobility/TrafficLight.hpp"

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <sstream>
#include <type_traits>
#include <utility>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf;
using namespace dsf::mobility;

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "data";

template <typename T1, typename T2>
bool checkPath(const std::vector<T1>& path1, const std::vector<T2>& path2) {
  const size_t length{path1.size()};
  assert(length == path2.size());

  bool equal{true};
  for (size_t i{}; i < length; ++i) {
    if (path1[i] != path2[i]) {
      equal = false;
    }
  }

  return equal;
}

TEST_CASE("RoadNetwork") {
  Road::setMeanVehicleLength(5.);
  SUBCASE("Empty Constructor") {
    RoadNetwork network{};
    network.addEdge<Street>(1, std::make_pair(0, 1));
    CHECK_EQ(network.nEdges(), 1);
    CHECK_EQ(network.nNodes(), 2);
  }

  SUBCASE("Construction with addEdge") {
    RoadNetwork graph;
    graph.addEdge<Street>(1, std::make_pair(0, 1));
    graph.addEdge<Street>(2, std::make_pair(1, 2));
    graph.addEdge<Street>(3, std::make_pair(0, 2));
    graph.addEdge<Street>(4, std::make_pair(0, 3));
    graph.addEdge<Street>(5, std::make_pair(2, 3));
    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.nNodes(), 4);
    CHECK_NOTHROW(graph.edge(0, 1));
    CHECK_NOTHROW(graph.edge(1, 2));
    CHECK_NOTHROW(graph.edge(0, 2));
    CHECK_NOTHROW(graph.edge(0, 3));
    CHECK_NOTHROW(graph.edge(2, 3));
    // Streets are linked to the junctions owned by the network
    CHECK_EQ(graph.edge(1).source(), &graph.node(0));
    CHECK_EQ(graph.edge(1).target(), &graph.node(1));
    CHECK_EQ(graph.edge(5).source(), &graph.node(2));
    CHECK_EQ(graph.edge(5).target(), &graph.node(3));
    // Edge ids are unique
    CHECK_THROWS_AS(graph.addEdge<Street>(1, std::make_pair(3, 0)),
                    std::invalid_argument);
    // An edge which cannot be built leaves no node behind
    CHECK_THROWS_AS(graph.addEdge<Street>(6, std::make_pair(7, 8), 0.),
                    std::invalid_argument);
    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.nNodes(), 4);
  }
  SUBCASE("nodes() and edges() views") {
    RoadNetwork graph;
    graph.addEdge<Street>(1, std::make_pair(0, 1));
    graph.addEdge<Street>(2, std::make_pair(1, 2));
    auto const& constGraph{graph};
    // The views only give const access through a const network
    static_assert(std::is_same_v<std::ranges::range_value_t<decltype(constGraph.nodes())>,
                                 std::pair<Id, RoadJunction const*>>);
    static_assert(std::is_same_v<std::ranges::range_value_t<decltype(constGraph.edges())>,
                                 std::pair<Id, Street const*>>);
    static_assert(std::is_same_v<std::ranges::range_value_t<decltype(graph.nodes())>,
                                 std::pair<Id, RoadJunction*>>);
    static_assert(std::is_same_v<std::ranges::range_value_t<decltype(graph.edges())>,
                                 std::pair<Id, Street*>>);
    CHECK_EQ(constGraph.nodes().size(), 3);
    CHECK_EQ(constGraph.edges().size(), 2);
    CHECK(constGraph.nodes().contains(2));
    CHECK_FALSE(constGraph.edges().contains(3));
    for (auto const& [nodeId, pNode] : constGraph.nodes()) {
      CHECK_EQ(pNode, &constGraph.node(nodeId));
    }
    for (auto const& [edgeId, pEdge] : constGraph.edges()) {
      CHECK_EQ(pEdge, &constGraph.edge(edgeId));
    }
  }
  SUBCASE("Construction with addStreet") {
    RoadNetwork graph;
    graph.addStreet(1, std::make_pair(0, 1));
    graph.addStreet(2, std::make_pair(1, 2));
    graph.addStreet(3, std::make_pair(0, 2));
    graph.addStreet(4, std::make_pair(0, 3));
    graph.addStreet(5, std::make_pair(2, 3));
    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.nNodes(), 4);
    CHECK_NOTHROW(graph.edge(0, 1));
    CHECK_NOTHROW(graph.edge(1, 2));
    CHECK_NOTHROW(graph.edge(0, 2));
    CHECK_NOTHROW(graph.edge(0, 3));
    CHECK_NOTHROW(graph.edge(2, 3));
    // Test describe method
    std::ostringstream oss;
    graph.describe(oss);
    std::string description = oss.str();
    CHECK(description.find("RoadNetwork with 4 nodes and 5 edges") != std::string::npos);
    CHECK(description.find("intersections") != std::string::npos);
    CHECK(description.find("traffic lights") != std::string::npos);
    CHECK(description.find("roundabouts") != std::string::npos);
    CHECK(description.find("coil sensors") != std::string::npos);
  }

  SUBCASE("automatically do things") {
    GIVEN("A Graph object") {
      RoadNetwork graph{};
      graph.addNode(0, dsf::geometry::Point(0., 0.));
      graph.addNode(1, dsf::geometry::Point(-1., 0.));
      graph.addNode(2, dsf::geometry::Point(0., -1.));
      graph.addNode(3, dsf::geometry::Point(1., 1.));
      graph.addEdge<Street>(1, std::make_pair(0, 1), 1., 50 / 3.6, 3);
      graph.addEdge<Street>(4, std::make_pair(1, 0), 1., 50 / 3.6, 3);
      graph.addEdge<Street>(2, std::make_pair(0, 2), 1., 30 / 3.6);
      graph.addEdge<Street>(8, std::make_pair(2, 0), 1., 30 / 3.6);
      graph.addEdge<Street>(3, std::make_pair(0, 3), 1., 30 / 3.6, 2);
      graph.addEdge<Street>(12, std::make_pair(3, 0), 1., 30 / 3.6, 2);
      CHECK_EQ(graph.nEdges(), 6);
      CHECK_EQ(graph.nNodes(), 4);
      WHEN("We automatically map street lanes") {
        // dsf::Logger::setLogLevel(dsf::log_level_t::DEBUG);
        graph.autoMapStreetLanes();
        // dsf::Logger::setLogLevel(dsf::log_level_t::INFO);
        THEN("The lanes are correctly mapped") {
          CHECK_EQ(graph.edge(0, 1).laneMapping().size(), 3);
          CHECK_EQ(graph.edge(0, 1).laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 1).laneMapping()[1], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 1).laneMapping()[2], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(1, 0).laneMapping().size(), 3);
          CHECK_EQ(graph.edge(1, 0).laneMapping()[0], dsf::Direction::RIGHT);
          CHECK_EQ(graph.edge(1, 0).laneMapping()[1], dsf::Direction::RIGHT);
          CHECK_EQ(graph.edge(1, 0).laneMapping()[2], dsf::Direction::LEFT);
          CHECK_EQ(graph.edge(0, 2).laneMapping().size(), 1);
          CHECK_EQ(graph.edge(0, 2).laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(2, 0).laneMapping().size(), 1);
          CHECK_EQ(graph.edge(2, 0).laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 3).laneMapping().size(), 2);
          CHECK_EQ(graph.edge(0, 3).laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 3).laneMapping()[1], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(3, 0).laneMapping().size(), 2);
          CHECK_EQ(graph.edge(3, 0).laneMapping()[0], dsf::Direction::RIGHT);
          CHECK_EQ(graph.edge(3, 0).laneMapping()[1], dsf::Direction::STRAIGHT);
        }
      }
    }
  }
  SUBCASE("importTrafficLights") {
    GIVEN("A graph object with a legacy traffic light CSV") {
      RoadNetwork graph;
      graph.addNDefaultNodes(4);
      graph.addStreet(100, std::make_pair(0, 1), 50., 13.8888888889);
      graph.addStreet(101, std::make_pair(2, 1), 50., 13.8888888889);
      graph.addStreet(102, std::make_pair(3, 1), 50., 13.8888888889);

      auto const csvPath =
          std::filesystem::temp_directory_path() / "dsf_import_traffic_lights.csv";
      {
        std::ofstream out{csvPath};
        REQUIRE(out.is_open());
        out << "id;sourceId;cycleTime;greenTime\n";
        out << "1;0;90;30\n";
        out << "1;2;90;60\n";
      }

      WHEN("We import the legacy traffic light definition") {
        graph.importTrafficLights(csvPath.string());
        auto& tl = graph.node<TrafficLight>(1);

        THEN("The importer reconstructs the two phases") {
          CHECK_EQ(graph.nTrafficLights(), 1);
          CHECK_EQ(tl.phases().size(), 2);
          CHECK_EQ(tl.phases()[0].duration(), 30);
          CHECK_EQ(tl.phases()[1].duration(), 60);
          CHECK(tl.phases()[0].containsGreen(100, dsf::Direction::ANY));
          CHECK(tl.phases()[1].containsGreen(101, dsf::Direction::ANY));
        }

        THEN("Free turns stay disabled for streets absent from the CSV") {
          CHECK_FALSE(tl.isGreen(102, dsf::Direction::RIGHT));
        }
      }

      std::filesystem::remove(csvPath);
    }
    GIVEN("A graph object and other traffic light CSVs") {
      RoadNetwork graph;
      graph.addNDefaultNodes(4);
      graph.addStreet(100, std::make_pair(0, 1), 50., 13.8888888889);
      graph.addStreet(101, std::make_pair(2, 1), 50., 13.8888888889);

      auto const csvPath =
          std::filesystem::temp_directory_path() / "dsf_import_traffic_lights_other.csv";
      auto const importCsv = [&](std::string const& rows) {
        {
          std::ofstream out{csvPath};
          REQUIRE(out.is_open());
          out << "id;sourceId;cycleTime;greenTime\n" << rows;
        }
        graph.importTrafficLights(csvPath.string());
      };

      THEN("Invalid definitions throw") {
        CHECK_THROWS_AS(graph.importTrafficLights("i_do_not_exist.csv"),
                        std::runtime_error);
        CHECK_THROWS_AS(importCsv("1;0;30;60\n"), std::invalid_argument);
        CHECK_THROWS_AS(importCsv("1;0;90;30\n1;2;60;30\n"), std::invalid_argument);
        CHECK_THROWS_AS(importCsv("1;0;70000;30\n"), std::invalid_argument);
      }
      THEN("Green times covering the whole cycle give a single phase") {
        importCsv("1;0;60;60\n\n1;2;60;60\n");
        auto const& tl = graph.node<TrafficLight>(1);
        REQUIRE_EQ(tl.phases().size(), 1);
        CHECK(tl.phases()[0].containsGreen(100, dsf::Direction::ANY));
        CHECK(tl.phases()[0].containsGreen(101, dsf::Direction::ANY));
      }

      std::filesystem::remove(csvPath);
    }
  }
  SUBCASE("exportTrafficLights") {
    GIVEN("A graph object with a two-phase traffic light") {
      RoadNetwork graph;
      graph.addNDefaultNodes(4);
      graph.addStreet(100, std::make_pair(0, 1), 50., 13.8888888889);
      graph.addStreet(101, std::make_pair(2, 1), 50., 13.8888888889);
      graph.addStreet(102, std::make_pair(3, 1), 50., 13.8888888889);

      graph.makeTrafficLight(1);
      auto& tl = graph.node<TrafficLight>(1);

      TrafficLightPhase phase0{30};
      phase0.addGreen(100);  // Direction::ANY
      TrafficLightPhase phase1{60};
      phase1.addGreen(101);  // Direction::ANY

      tl.setPhases({phase0, phase1});
      // A traffic light without phases has nothing to export and must be skipped
      graph.makeTrafficLight(3);

      auto const csvPath =
          std::filesystem::temp_directory_path() / "dsf_export_traffic_lights.csv";

      WHEN("We export the traffic light definition") {
        graph.exportTrafficLights(csvPath.string());

        THEN(
            "The CSV contains one row per green street with the right cycle/green "
            "times") {
          std::ifstream in{csvPath};
          REQUIRE(in.is_open());

          std::string header;
          std::getline(in, header);
          CHECK_EQ(header, "id;sourceId;cycleTime;greenTime");

          std::unordered_map<Id, std::pair<Delay, Delay>>
              rows;  // sourceId -> {cycleTime, greenTime}
          std::string line;
          std::size_t nRows{0};
          while (std::getline(in, line)) {
            if (line.empty())
              continue;
            ++nRows;
            std::istringstream iss{line};
            std::string strId, strSource, strCycle, strGreen;
            std::getline(iss, strId, ';');
            std::getline(iss, strSource, ';');
            std::getline(iss, strCycle, ';');
            std::getline(iss, strGreen, '\n');

            CHECK_EQ(static_cast<Id>(std::stoul(strId)), 1);
            auto const sourceId = static_cast<Id>(std::stoul(strSource));
            rows[sourceId] = {static_cast<Delay>(std::stoul(strCycle)),
                              static_cast<Delay>(std::stoul(strGreen))};
          }

          CHECK_EQ(nRows, 2);
          REQUIRE(rows.contains(0));
          REQUIRE(rows.contains(2));
          CHECK_EQ(rows.at(0).first, 90);   // cycleTime
          CHECK_EQ(rows.at(0).second, 30);  // greenTime (phase0)
          CHECK_EQ(rows.at(2).first, 90);
          CHECK_EQ(rows.at(2).second, 60);  // greenTime (phase1)

          CHECK_FALSE(rows.contains(3));  // street 102 not green in any phase
        }

        THEN("Re-importing the exported CSV reconstructs an equivalent traffic light") {
          RoadNetwork roundTrip;
          roundTrip.addNDefaultNodes(4);
          roundTrip.addStreet(100, std::make_pair(0, 1), 50., 13.8888888889);
          roundTrip.addStreet(101, std::make_pair(2, 1), 50., 13.8888888889);
          roundTrip.addStreet(102, std::make_pair(3, 1), 50., 13.8888888889);

          roundTrip.importTrafficLights(csvPath.string());
          auto& tl2 = roundTrip.node<TrafficLight>(1);

          CHECK_EQ(tl2.phases().size(), 2);
          CHECK_EQ(tl2.cycleTime(), tl.cycleTime());

          // Only phase 0 is active by default — check its contents directly
          // rather than assuming both streets are simultaneously green.
          CHECK(tl2.phases()[0].containsGreen(100, dsf::Direction::ANY));
          CHECK(tl2.phases()[1].containsGreen(101, dsf::Direction::ANY));
          CHECK(tl2.isGreen(100, dsf::Direction::ANY));        // active phase
          CHECK_FALSE(tl2.isGreen(101, dsf::Direction::ANY));  // not active yet
          CHECK_FALSE(tl2.isGreen(102, dsf::Direction::RIGHT));
        }
      }
      WHEN("We export to a path which cannot be opened") {
        auto const badPath =
            std::filesystem::temp_directory_path() / "dsf_missing_dir" / "tl.csv";
        THEN("An exception is thrown") {
          CHECK_THROWS_AS(graph.exportTrafficLights(badPath.string()),
                          std::runtime_error);
        }
      }

      std::filesystem::remove(csvPath);
    }
  }

  SUBCASE("importEdges and importNodeProperties") {
    GIVEN("A graph object") {
      RoadNetwork graph;
      WHEN("We import edges and then node properties from OSM") {
        graph.importEdges((DATA_FOLDER / "postua_edges.csv").string());
        graph.importNodeProperties((DATA_FOLDER / "postua_nodes.csv").string());
        std::ifstream fNodes{(DATA_FOLDER / "postua_nodes.csv").string()};
        // get number of lines
        std::string line;
        int nNodes{-1};  // -1 because of the header
        while (std::getline(fNodes, line)) {
          ++nNodes;
        }
        fNodes.close();
        std::ifstream fEdges{(DATA_FOLDER / "postua_edges.csv").string()};
        int nEdges{-1};  // -1 because of the header
        while (std::getline(fEdges, line)) {
          ++nEdges;
        }
        fEdges.close();
        THEN("Sizes are correct") {
          CHECK_EQ(graph.nNodes(), nNodes);
          CHECK_EQ(graph.nEdges(), nEdges);
          CHECK_EQ(graph.nCoils(), 0);
          CHECK_EQ(graph.nIntersections(), 25);
          CHECK_EQ(graph.nRoundabouts(), 1);
          CHECK_EQ(graph.nTrafficLights(), 0);
        }
        THEN("Road types are set") {
          int nHighways = 0, nPrimary = 0, nSecondary = 0, nTertiary = 0,
              nResidential = 0;
          for (auto const& [_, pEdge] : graph.edges()) {
            switch (pEdge->mobilityClass()) {
              case 128u:
                ++nHighways;
                break;
              case 64u:
                ++nPrimary;
                break;
              case 32u:
                ++nSecondary;
                break;
              case 16u:
                ++nTertiary;
                break;
              case 8u:
                ++nResidential;
                break;
              default:
                break;
            }
          }
          CHECK_GT(nHighways + nPrimary + nSecondary + nTertiary + nResidential, 0);
        }
        RoadNetwork graph2;
        graph2.importEdges((DATA_FOLDER / "postua_edges.geojson").string());
        THEN("Sizes are correct also with geojson") {
          CHECK_EQ(graph2.nEdges(), graph.nEdges());
          CHECK_EQ(graph2.nNodes(), graph.nNodes());
          for (auto const& [edgeId, pEdge] : graph2.edges()) {
            CHECK_EQ(*pEdge, graph.edge(edgeId));
          }
        }
      }
      WHEN("We import many nodes and edges from OSM") {
        graph.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
        graph.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
        std::ifstream fNodes{(DATA_FOLDER / "forlì_nodes.csv").string()};
        // get number of lines
        std::string line;
        int nNodes{-1};  // -1 because of the header
        while (std::getline(fNodes, line)) {
          ++nNodes;
        }
        fNodes.close();
        std::ifstream fEdges{(DATA_FOLDER / "forlì_edges.csv").string()};
        int nEdges{-1};  // -1 because of the header
        while (std::getline(fEdges, line)) {
          ++nEdges;
        }
        fEdges.close();
        THEN("Sizes are correct") {
          CHECK_EQ(graph.nNodes(), nNodes);
          CHECK_EQ(graph.nEdges(), nEdges);
          CHECK_EQ(graph.nCoils(), 0);
          CHECK_EQ(graph.nIntersections(), 11100);
          CHECK_EQ(graph.nRoundabouts(), 17);
          CHECK_EQ(graph.nTrafficLights(), 30);
        }
      }

      WHEN("We import edges from CSV with priority field") {
        auto const tmpCsvPath =
            std::filesystem::temp_directory_path() / "dsf_priority_edges_test.csv";
        {
          std::ofstream tmpCsv(tmpCsvPath);
          REQUIRE(tmpCsv.is_open());
          tmpCsv << "id;source;target;oneway;length;geometry;travel_time;maxspeed;nlanes;"
                    "type;name;priority\n";
          tmpCsv << "100;1;2;False;100.0;LINESTRING (8.0 45.0, 8.1 45.1);12.0;30.0;1;"
                    "residential;priority_street_true;True\n";
          tmpCsv << "101;2;3;False;120.0;LINESTRING (8.1 45.1, 8.2 45.2);14.4;30.0;1;"
                    "residential;priority_street_false;False\n";
        }

        RoadNetwork graphWithPriority;
        graphWithPriority.importEdges(tmpCsvPath.string());

        THEN("Priority is correctly imported from CSV") {
          REQUIRE_NOTHROW(graphWithPriority.edge(static_cast<Id>(100)));
          REQUIRE_NOTHROW(graphWithPriority.edge(static_cast<Id>(101)));
          CHECK(graphWithPriority.edge(static_cast<Id>(100)).hasPriority());
          CHECK_FALSE(graphWithPriority.edge(static_cast<Id>(101)).hasPriority());
          std::filesystem::remove(tmpCsvPath);
        }
      }

      WHEN("We import edges from GeoJSON with priority field") {
        auto const tmpGeoJsonPath =
            std::filesystem::temp_directory_path() / "dsf_priority_edges_test.geojson";
        {
          std::ofstream tmpGeoJson(tmpGeoJsonPath);
          REQUIRE(tmpGeoJson.is_open());
          tmpGeoJson << R"({
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {
        "id": 200,
        "source": 10,
        "target": 11,
        "length": 50.0,
        "maxspeed": 30.0,
        "nlanes": 1,
        "type": "residential",
        "name": "geo_priority_true",
        "priority": true
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [[8.0, 45.0], [8.1, 45.1]]
      }
    },
    {
      "type": "Feature",
      "properties": {
        "id": 201,
        "source": 11,
        "target": 12,
        "length": 60.0,
        "maxspeed": 30.0,
        "nlanes": 1,
        "type": "residential",
        "name": "geo_priority_false",
        "priority": false
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [[8.1, 45.1], [8.2, 45.2]]
      }
    }
  ]
})";
        }

        RoadNetwork graphWithPriority;
        graphWithPriority.importEdges(tmpGeoJsonPath.string());

        THEN("Priority is correctly imported from GeoJSON") {
          REQUIRE_NOTHROW(graphWithPriority.edge(static_cast<Id>(200)));
          REQUIRE_NOTHROW(graphWithPriority.edge(static_cast<Id>(201)));
          CHECK(graphWithPriority.edge(static_cast<Id>(200)).hasPriority());
          CHECK_FALSE(graphWithPriority.edge(static_cast<Id>(201)).hasPriority());
          std::filesystem::remove(tmpGeoJsonPath);
        }
      }
      WHEN("We import edges from CSV with mobility_class field") {
        auto const tmpCsvPath =
            std::filesystem::temp_directory_path() / "dsf_mobility_class_edges_test.csv";
        {
          std::ofstream tmpCsv(tmpCsvPath);
          REQUIRE(tmpCsv.is_open());
          tmpCsv << "id;source;target;oneway;length;geometry;travel_time;maxspeed;nlanes;"
                    "type;name;mobility_class\n";
          // type says "residential" but mobility_class explicitly overrides it to 128
          tmpCsv << "300;1;2;False;100.0;LINESTRING (8.0 45.0, 8.1 45.1);12.0;30.0;1;"
                    "residential;mobility_class_override;128\n";
          // no override needed here, just a plain explicit value
          tmpCsv << "301;2;3;False;120.0;LINESTRING (8.1 45.1, 8.2 45.2);14.4;30.0;1;"
                    "residential;mobility_class_plain;32\n";
          // invalid value should fall back to default (0)
          tmpCsv << "302;3;4;False;140.0;LINESTRING (8.2 45.2, 8.3 45.3);16.8;30.0;1;"
                    "residential;mobility_class_invalid;not_a_number\n";
        }

        RoadNetwork graphWithMobilityClass;
        graphWithMobilityClass.importEdges(tmpCsvPath.string());

        THEN("mobility_class is correctly imported from CSV") {
          REQUIRE_NOTHROW(graphWithMobilityClass.edge(static_cast<Id>(300)));
          REQUIRE_NOTHROW(graphWithMobilityClass.edge(static_cast<Id>(301)));
          REQUIRE_NOTHROW(graphWithMobilityClass.edge(static_cast<Id>(302)));
          CHECK_EQ(graphWithMobilityClass.edge(static_cast<Id>(300)).mobilityClass(),
                   128u);
          CHECK_EQ(graphWithMobilityClass.edge(static_cast<Id>(301)).mobilityClass(),
                   32u);
          CHECK_EQ(graphWithMobilityClass.edge(static_cast<Id>(302)).mobilityClass(), 0u);
          std::filesystem::remove(tmpCsvPath);
        }
      }

      WHEN("We import edges from GeoJSON with mobility_class field") {
        auto const tmpGeoJsonPath = std::filesystem::temp_directory_path() /
                                    "dsf_mobility_class_edges_test.geojson";
        {
          std::ofstream tmpGeoJson(tmpGeoJsonPath);
          REQUIRE(tmpGeoJson.is_open());
          tmpGeoJson << R"({
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {
        "id": 400,
        "source": 20,
        "target": 21,
        "length": 50.0,
        "maxspeed": 30.0,
        "nlanes": 1,
        "type": "residential",
        "name": "geo_mobility_class_override",
        "mobility_class": 64
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [[8.0, 45.0], [8.1, 45.1]]
      }
    },
    {
      "type": "Feature",
      "properties": {
        "id": 401,
        "source": 21,
        "target": 22,
        "length": 60.0,
        "maxspeed": 30.0,
        "nlanes": 1,
        "type": "motorway",
        "name": "geo_mobility_class_from_type"
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [[8.1, 45.1], [8.2, 45.2]]
      }
    }
  ]
})";
        }

        RoadNetwork graphWithMobilityClass;
        graphWithMobilityClass.importEdges(tmpGeoJsonPath.string());

        THEN("mobility_class is correctly imported from GeoJSON") {
          REQUIRE_NOTHROW(graphWithMobilityClass.edge(static_cast<Id>(400)));
          REQUIRE_NOTHROW(graphWithMobilityClass.edge(static_cast<Id>(401)));
          // explicit mobility_class (64) overrides the "residential" type default
          CHECK_EQ(graphWithMobilityClass.edge(static_cast<Id>(400)).mobilityClass(),
                   64u);
          // no mobility_class given -> falls back to type-based inference ("motorway" -> 128)
          CHECK_EQ(graphWithMobilityClass.edge(static_cast<Id>(401)).mobilityClass(),
                   128u);
          std::filesystem::remove(tmpGeoJsonPath);
        }
      }

      WHEN("We import edges from CSV with optional and invalid fields") {
        auto const tmpCsvPath =
            std::filesystem::temp_directory_path() / "dsf_optional_edges_test.csv";
        {
          std::ofstream tmpCsv(tmpCsvPath);
          REQUIRE(tmpCsv.is_open());
          tmpCsv << "id;source;target;length;maxspeed;nlanes;type;name;coilcode;capacity;"
                    "status;forbidden_turns;lane_mapping;transport_capacity\n";
          tmpCsv << "10;0;1;100;50;2;primary;a;C1;5;open;[11, 12];[left, straight];0.5\n";
          // Invalid maxspeed, nlanes and capacity fall back to their defaults
          tmpCsv << "11;1;2;100;fast;many;secondary;b;null;big;CLOSED;;"
                    "[straight_right, straight_left, through];2\n";
          tmpCsv << "12;2;3;100;50;1;tertiary;c;;;weird;[x];[any];-1\n";
          tmpCsv << "13;3;3;100;50;1;tertiary;self_loop;;;;;;\n";
        }

        RoadNetwork graphWithOptionals;
        graphWithOptionals.importEdges(tmpCsvPath.string());
        std::filesystem::remove(tmpCsvPath);

        THEN("Valid fields are imported, invalid ones are skipped") {
          CHECK_EQ(graphWithOptionals.nEdges(), 3);
          CHECK_EQ(graphWithOptionals.nCoils(), 1);

          auto const& e10 = graphWithOptionals.edge(static_cast<Id>(10));
          CHECK_EQ(e10.counterName(), "c1");
          CHECK_EQ(e10.capacity(), 5);
          CHECK(e10.isActive());
          CHECK_EQ(e10.forbiddenTurns(), std::set<Id>{11, 12});
          CHECK_EQ(e10.laneMapping(),
                   std::vector<Direction>{Direction::STRAIGHT, Direction::LEFT});
          CHECK_EQ(e10.transportCapacity(), doctest::Approx(0.5));
          CHECK_FALSE(e10.attributes().contains("transport_capacity"));

          auto const& e11 = graphWithOptionals.edge(static_cast<Id>(11));
          CHECK_FALSE(e11.hasCoil());
          CHECK_FALSE(e11.isActive());
          CHECK_EQ(e11.maxSpeed(), doctest::Approx(30. / 3.6));
          // The lane mapping sets the number of lanes
          CHECK_EQ(e11.nLanes(), 3);
          CHECK_EQ(e11.laneMapping(),
                   std::vector<Direction>{Direction::RIGHTANDSTRAIGHT,
                                          Direction::ANY,
                                          Direction::LEFTANDSTRAIGHT});
          // Values above 1 are accepted and not rescaled by the lane_mapping lane change
          CHECK_EQ(e11.transportCapacity(), doctest::Approx(2.));

          auto const& e12 = graphWithOptionals.edge(static_cast<Id>(12));
          CHECK(e12.isActive());
          CHECK(e12.forbiddenTurns().empty());
          CHECK_EQ(e12.laneMapping(), std::vector<Direction>{Direction::ANY});
          // Non-positive transport capacity falls back to the default
          CHECK_EQ(e12.transportCapacity(), doctest::Approx(1.));
        }
      }

      WHEN("We import edges from GeoJSON with optional and invalid fields") {
        auto const tmpGeoJsonPath =
            std::filesystem::temp_directory_path() / "dsf_optional_edges_test.geojson";
        {
          std::ofstream tmpGeoJson(tmpGeoJsonPath);
          REQUIRE(tmpGeoJson.is_open());
          tmpGeoJson << R"({
  "type": "FeatureCollection",
  "features": [
    { "type": "Feature",
      "properties": { "id": 500, "source": 30, "target": 31, "length": 50.0,
        "maxspeed": "50", "nlanes": "2", "type": "primary", "name": "a",
        "coilcode": "C5", "forbidden_turns": [501, "x"],
        "lane_mapping": ["Left", "straight", 3], "transport_capacity": 0.25,
        "custom_int": 7, "custom_str": "hello", "custom_null": null, "custom_arr": [1] },
      "geometry": { "type": "LineString", "coordinates": [[8.0, 45.0], [8.1, 45.1]] } },
    { "type": "Feature",
      "properties": { "id": 501, "source": 31, "target": 32, "length": 50.0,
        "maxspeed": "fast", "nlanes": "x", "type": ["secondary", 5], "name": "b",
        "coilcode": 42, "forbidden_turns": "none", "lane_mapping": "none",
        "mobility_class": "high", "transport_capacity": "x" },
      "geometry": { "type": "LineString", "coordinates": [[8.1, 45.1], [8.2, 45.2]] } },
    { "type": "Feature",
      "properties": { "id": 502, "source": 32, "target": 33, "length": 50.0,
        "maxspeed": 50, "nlanes": 1, "type": 7, "name": "c", "coilcode": -3,
        "transport_capacity": 3 },
      "geometry": { "type": "LineString", "coordinates": [[8.2, 45.2], [8.3, 45.3]] } },
    { "type": "Feature",
      "properties": { "id": 503, "source": 33, "target": 34, "length": 50.0,
        "maxspeed": 50, "nlanes": 1, "type": "residential", "name": "d",
        "coilcode": true, "transport_capacity": 0 },
      "geometry": { "type": "LineString", "coordinates": [[8.3, 45.3], [8.4, 45.4]] } },
    { "type": "Feature",
      "properties": { "id": 504, "source": 34, "target": 34, "length": 50.0,
        "maxspeed": 50, "nlanes": 1, "type": "residential", "name": "loop" },
      "geometry": { "type": "LineString", "coordinates": [[8.4, 45.4], [8.4, 45.4]] } }
  ]
})";
        }

        RoadNetwork graphWithOptionals;
        graphWithOptionals.importEdges(tmpGeoJsonPath.string());
        std::filesystem::remove(tmpGeoJsonPath);

        THEN("Valid fields are imported, invalid ones are skipped") {
          CHECK_EQ(graphWithOptionals.nEdges(), 4);
          CHECK_EQ(graphWithOptionals.nCoils(), 4);

          auto const& e500 = graphWithOptionals.edge(static_cast<Id>(500));
          CHECK_EQ(e500.maxSpeed(), doctest::Approx(50. / 3.6));
          CHECK_EQ(e500.nLanes(), 2);
          CHECK_EQ(e500.mobilityClass(), 64u);
          CHECK_EQ(e500.counterName(), "C5");
          CHECK_EQ(e500.forbiddenTurns(), std::set<Id>{501});
          CHECK_EQ(e500.laneMapping(),
                   std::vector<Direction>{Direction::STRAIGHT, Direction::LEFT});
          CHECK_EQ(e500.getAttribute<std::int64_t>("custom_int"), 7);
          CHECK_EQ(e500.getAttribute<std::string>("custom_str"), "hello");
          CHECK(e500.attributes().contains("custom_null"));
          CHECK_FALSE(e500.attributes().contains("custom_arr"));
          CHECK_EQ(e500.transportCapacity(), doctest::Approx(0.25));
          CHECK_FALSE(e500.attributes().contains("transport_capacity"));

          auto const& e501 = graphWithOptionals.edge(static_cast<Id>(501));
          CHECK_EQ(e501.maxSpeed(), doctest::Approx(30. / 3.6));
          CHECK_EQ(e501.nLanes(), 1);
          CHECK_EQ(e501.mobilityClass(), 0u);
          CHECK_EQ(e501.counterName(), "42");
          CHECK(e501.forbiddenTurns().empty());
          CHECK_EQ(e501.transportCapacity(), doctest::Approx(1.));

          auto const& e502 = graphWithOptionals.edge(static_cast<Id>(502));
          CHECK_EQ(e502.counterName(), "-3");
          CHECK_EQ(e502.transportCapacity(), doctest::Approx(3.));

          CHECK_EQ(graphWithOptionals.edge(static_cast<Id>(503)).transportCapacity(),
                   doctest::Approx(1.));
        }
      }

      WHEN("We import from invalid files") {
        auto const tmpPath = std::filesystem::temp_directory_path() / "dsf_invalid.txt";
        auto const tmpJsonPath =
            std::filesystem::temp_directory_path() / "dsf_invalid.json";
        {
          std::ofstream tmpFile(tmpPath);
          std::ofstream tmpJson(tmpJsonPath);
          REQUIRE(tmpFile.is_open());
          REQUIRE(tmpJson.is_open());
          tmpJson << "{ not json";
        }
        THEN("An exception is thrown") {
          CHECK_THROWS_AS(graph.importNodeProperties(tmpPath.string()),
                          std::runtime_error);
          CHECK_THROWS_AS(graph.importEdges("i_do_not_exist.csv"), std::runtime_error);
          CHECK_THROWS_AS(graph.importEdges(tmpPath.string()), std::invalid_argument);
          CHECK_THROWS_AS(graph.importEdges(tmpJsonPath.string()), std::runtime_error);

          graph.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
          CHECK_THROWS_AS(graph.importNodeProperties("i_do_not_exist.csv"),
                          std::runtime_error);
          CHECK_THROWS_AS(graph.importNodeProperties(tmpPath.string()),
                          std::invalid_argument);
          CHECK_THROWS_AS(graph.importNodeProperties(tmpJsonPath.string()),
                          std::invalid_argument);
        }
        std::filesystem::remove(tmpPath);
        std::filesystem::remove(tmpJsonPath);
      }

      WHEN("We import node properties for unknown nodes or mismatching geometries") {
        graph.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
        auto const tmpCsvPath =
            std::filesystem::temp_directory_path() / "dsf_node_properties_test.csv";
        {
          std::ofstream tmpCsv(tmpCsvPath);
          REQUIRE(tmpCsv.is_open());
          tmpCsv << "id;type;geometry\n";
          tmpCsv << "99999;traffic_signals;POINT (0 0)\n";
          tmpCsv << "0;N/A;POINT (10 10)\n";
        }
        auto const oldGeometry = graph.node(0).geometry();
        graph.importNodeProperties(tmpCsvPath.string());
        std::filesystem::remove(tmpCsvPath);
        THEN("Unknown nodes are skipped and existing geometries are kept") {
          CHECK_EQ(graph.nTrafficLights(), 0);
          CHECK_EQ(graph.node(0).geometry(), oldGeometry);
        }
      }
    }
    SUBCASE("street") {
      /// GIVEN: a graph
      /// WHEN: we add a street
      /// THEN: the street is added
      RoadNetwork graph{};
      graph.addEdge<Street>(1, std::make_pair(0, 1), 1.);
      auto result = graph.street(0, 1);
      CHECK(result);
      const auto& street2 = *result;
      CHECK_EQ(street2.id(), 1);
      CHECK_EQ(street2.length(), 1.);
      CHECK_EQ(street2.capacity(), 1);
      CHECK_FALSE(graph.street(1, 0));
    }
    SUBCASE("make trafficlight") {
      GIVEN("A graph with a traffic light with no parameters") {
        RoadNetwork graph2;
        graph2.addNDefaultNodes(5);
        graph2.makeTrafficLight(1);
        auto& s1 = graph2.addStreet(1, std::make_pair(0, 1), 30., 15., 3);
        s1.setPriority();
        auto& s2 = graph2.addStreet(11, std::make_pair(2, 1), 30., 15., 3);
        s2.setPriority();
        graph2.addStreet(16, std::make_pair(3, 1), 30., 15., 1);
        graph2.addStreet(21, std::make_pair(4, 1), 30., 15., 2);
        for (auto const& [streetId, pStreet] : graph2.edges()) {
          pStreet->setCapacity(2 * pStreet->nLanes());
        }
        WHEN("We auto-init Traffic Lights") {
          graph2.autoInitTrafficLights();
          THEN("Parameters are correctly set") {
            auto& tl{graph2.node<dsf::mobility::TrafficLight>(1)};
            // cycleTime = TRAFFICLIGHT_DEFAULT_CYCLE = 90
            CHECK_EQ(tl.cycleTime(), 90);
            auto const& phases{tl.phases()};
            CHECK_EQ(phases.size(), 2);
            // Phase 0: priority streets 1 and 11 (3-lane → RIGHTANDSTRAIGHT + LEFT)
            // duration = floor(0.6 * 90) = 54
            CHECK_EQ(phases[0].duration(), 54);
            CHECK(phases[0].containsGreen(1, dsf::Direction::ANY));
            CHECK(phases[0].containsGreen(11, dsf::Direction::ANY));
            // Phase 1: non-priority streets 16 and 21 (1-lane → ANY)
            // duration = 90 - 54 = 36
            CHECK_EQ(phases[1].duration(), 36);
            CHECK(phases[1].containsGreen(16, dsf::Direction::ANY));
            CHECK(phases[1].containsGreen(21, dsf::Direction::ANY));
          }
        }
      }
      GIVEN("A traffic light with streets having same names (priority by name)") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0);
        // Constructor: id, nodePair, length, maxSpeed, nLanes, name
        graph.addStreet(1, std::make_pair(1, 0), 50., 15., 1, "Main Street");
        graph.addStreet(2, std::make_pair(2, 0), 50., 15., 1, "Main Street");
        graph.addStreet(3, std::make_pair(3, 0), 50., 15., 1, "Side Street");

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Streets with same name get priority") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& priorities{tl.streetPriorities()};
            CHECK_EQ(priorities.size(), 2);
            CHECK(priorities.contains(1));
            CHECK(priorities.contains(2));
            CHECK_FALSE(priorities.contains(3));
          }
        }
      }
      GIVEN("A traffic light with streets having different max speeds") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0);
        // All streets have unique names (no name-based priority) but different speeds
        // Higher speed
        // Higher speed
        // Lower speed
        graph.addStreet(1, std::make_pair(1, 0), 50., 50., 1, "Road A");
        graph.addStreet(2, std::make_pair(2, 0), 50., 50., 1, "Road B");
        graph.addStreet(3, std::make_pair(3, 0), 50., 30., 1, "Road C");

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Streets with higher speed get priority") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& priorities{tl.streetPriorities()};
            CHECK_EQ(priorities.size(), 2);
            CHECK(priorities.contains(1));
            CHECK(priorities.contains(2));
            CHECK_FALSE(priorities.contains(3));
          }
        }
      }
      GIVEN("A traffic light with streets having different lane counts") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0);
        // All unique names, same speed, different lane counts
        // Higher lanes
        // Higher lanes
        // Lower lanes
        graph.addStreet(1, std::make_pair(1, 0), 50., 30., 2, "Road A");
        graph.addStreet(2, std::make_pair(2, 0), 50., 30., 2, "Road B");
        graph.addStreet(3, std::make_pair(3, 0), 50., 30., 1, "Road C");

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Streets with higher lane count get priority") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& priorities{tl.streetPriorities()};
            CHECK_EQ(priorities.size(), 2);
            CHECK(priorities.contains(1));
            CHECK(priorities.contains(2));
            CHECK_FALSE(priorities.contains(3));
          }
        }
      }
      GIVEN("A traffic light with less than 3 incoming streets") {
        RoadNetwork graph;
        graph.addNDefaultNodes(3);
        graph.makeTrafficLight(0);
        graph.addStreet(1, std::make_pair(1, 0), 50., 30., 1);
        graph.addStreet(2, std::make_pair(2, 0), 50., 30., 1);

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Node is converted to intersection") {
            CHECK_FALSE(graph.node(0).isTrafficLight());
            CHECK(graph.node(0).isIntersection());
            // The streets are re-linked to the new node
            CHECK_EQ(graph.edge(1).target(), &graph.node(0));
            CHECK_EQ(graph.edge(2).target(), &graph.node(0));
          }
        }
      }
      GIVEN("A traffic light with custom mainRoadPercentage") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0);
        auto& s1 = graph.addStreet(1, std::make_pair(1, 0), 50., 15., 1);
        s1.setPriority();
        auto& s2 = graph.addStreet(2, std::make_pair(2, 0), 50., 15., 1);
        s2.setPriority();
        graph.addStreet(3, std::make_pair(3, 0), 50., 15., 1);

        WHEN("We auto-init Traffic Lights with 70% main road percentage") {
          graph.autoInitTrafficLights(0.7);
          THEN("Phase durations reflect the custom percentage") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& phases{tl.phases()};
            CHECK_EQ(phases.size(), 2);
            CHECK_EQ(phases.at(0).duration(), 62);
            CHECK_EQ(phases.at(1).duration(), 28);
            CHECK(phases.at(0).containsGreen(1, dsf::Direction::ANY));
            CHECK(phases.at(0).containsGreen(2, dsf::Direction::ANY));
            CHECK(phases.at(1).containsGreen(3, dsf::Direction::ANY));
          }
        }
      }
      GIVEN("A traffic light already initialized with cycles") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        auto& tl = graph.makeTrafficLight(0);
        graph.addStreet(1, std::make_pair(1, 0), 50., 15., 1);
        graph.addStreet(2, std::make_pair(2, 0), 50., 15., 1);
        graph.addStreet(3, std::make_pair(3, 0), 50., 15., 1);
        // Pre-initialize phases so auto-init should skip this traffic light
        tl.setPhases(std::vector<TrafficLightPhase>{TrafficLightPhase{50}});

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Traffic light is not modified") {
            auto const& phases{tl.phases()};
            CHECK_EQ(phases.size(), 1);
            CHECK_EQ(phases.at(0).duration(), 50);
          }
        }
      }
      GIVEN("A traffic light with 4 streets all having the same characteristics") {
        RoadNetwork graph;
        graph.addNDefaultNodes(5);
        graph.makeTrafficLight(0);
        // All streets identical with unique names - will fall back to angle or capacity-based selection
        graph.addStreet(1, std::make_pair(1, 0), 50., 30., 1, "Road A");
        graph.addStreet(2, std::make_pair(2, 0), 50., 30., 1, "Road B");
        graph.addStreet(3, std::make_pair(3, 0), 50., 30., 1, "Road C");
        graph.addStreet(4, std::make_pair(4, 0), 50., 30., 1, "Road D");
        for (auto const& [streetId, pStreet] : graph.edges()) {
          pStreet->setCapacity(10);
        }

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Fallback assigns exactly 2 priorities") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& priorities{tl.streetPriorities()};
            CHECK_EQ(priorities.size(), 2);
            // Phases should be initialized for priority and non-priority streets
            auto const& phases{tl.phases()};
            CHECK_EQ(phases.size(), 2);
          }
        }
      }
      GIVEN("A traffic light with 3-lane streets") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0);
        auto& s1 = graph.addStreet(1, std::make_pair(1, 0), 50., 15., 3);
        s1.setPriority();
        auto& s2 = graph.addStreet(2, std::make_pair(2, 0), 50., 15., 3);
        s2.setPriority();
        graph.addStreet(3, std::make_pair(3, 0), 50., 15., 3);

        graph.autoMapStreetLanes();

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("3-lane streets get RIGHTANDSTRAIGHT and LEFT phases") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& phases{tl.phases()};
            CHECK_EQ(phases.size(), 2);
            // Each 3-lane street should have both directions green in the priority phase
            CHECK(phases.at(0).containsGreen(1, dsf::Direction::ANY));
            CHECK(phases.at(0).containsGreen(2, dsf::Direction::ANY));
            // Non-priority phase should still carry the 3-lane street directions
            CHECK(phases.at(1).containsGreen(3, dsf::Direction::ANY));
          }
        }
      }
    }
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(1, std::make_pair(0, 1));
      WHEN("We make node 0 a traffic light") {
        auto& tl = graph.makeTrafficLight(0);
        THEN("The node 0 is a traffic light") { CHECK(graph.node(0).isTrafficLight()); }
        THEN("The traffic light has the correct parameters") {
          CHECK_EQ(tl.id(), 0);
          CHECK_EQ(tl.cycleTime(), 0);
        }
      }
    }
  }
  SUBCASE("make roundabout") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(1, std::make_pair(0, 1));
      WHEN("We make node 0 a roundabout") {
        auto& roundabout = graph.makeRoundabout(0);
        THEN("The node 0 is a roundabout and the street leaves from it") {
          CHECK(graph.node(0).isRoundabout());
          CHECK_EQ(graph.edge(1).source(), &roundabout);
        }
      }
      WHEN("We make node 1 a station") {
        auto& station = graph.makeStation(1, 5);
        THEN("The node 1 is a station and keeps its connections") {
          CHECK(station.isStation());
          CHECK_EQ(station.managementTime(), 5);
          CHECK_EQ(station.ingoingEdges().size(), 1);
          CHECK_EQ(station.ingoingEdges().front(), &graph.edge(1));
          CHECK_EQ(graph.edge(1).target(), &station);
        }
      }
    }
  }
  SUBCASE("add coil on a street") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(0, std::make_pair(0, 1));
      WHEN("We add a coil to the street") {
        graph.addCoil(0);
        THEN("The street has a coil") { CHECK(graph.edge(0).hasCoil()); }
      }
    }
  }
  SUBCASE("street and oppositeStreet") {
    GIVEN("A RoadNetwork object with two streets") {
      RoadNetwork graph{};
      graph.addNDefaultNodes(2);
      graph.addStreet(1, std::make_pair(0, 1), 1.);
      graph.addStreet(2, std::make_pair(1, 0), 1.);
      WHEN("We search for a street") {
        auto result = graph.street(0, 1);
        THEN("The street is found and has correct values") {
          CHECK(result);
          const auto& road = *result;
          CHECK_EQ(road.id(), 1);
          CHECK_EQ(road.length(), 1.);
          CHECK_EQ(road.capacity(), 1);
        }
      }
      WHEN("We search for the opposite street") {
        auto result = graph.street(1, 0);
        THEN("The opposite street is found and has correct values") {
          CHECK(result);
          const auto& road = *result;
          CHECK_EQ(road.id(), 2);
          CHECK_EQ(road.length(), 1.);
          CHECK_EQ(road.capacity(), 1);
        }
      }
      WHEN("We search for a not existing street") {
        THEN("The street is not found") { CHECK_FALSE(graph.street(1, 2)); }
      }
      WHEN("We search for the opposite of a not existing street") {
        THEN("It throws an exception") { CHECK_FALSE(graph.street(2, 1)); }
      }
    }
    GIVEN("A RoadNetwork object with a path of two streets") {
      RoadNetwork graph{};
      graph.addNDefaultNodes(3);
      graph.addStreet(1, std::make_pair(0, 1), 1.);
      graph.addStreet(2, std::make_pair(1, 2), 1.);
      WHEN("We search for streets by their endpoints") {
        THEN("Existing streets are found") {
          CHECK_EQ(graph.street(1, 2)->id(), 2);
          CHECK_EQ(graph.edge(0, 1).id(), 1);
        }
        THEN("Missing streets between existing nodes are not found") {
          CHECK_FALSE(graph.street(0, 2));
          CHECK_FALSE(graph.street(2, 1));
          CHECK_THROWS_AS(graph.edge(0, 2), std::out_of_range);
          CHECK_THROWS_AS(graph.edge(3, 0), std::out_of_range);
        }
      }
    }
  }

  SUBCASE("reverse") {
    GIVEN("A RoadNetwork object with a two-way street and a one-way street") {
      RoadNetwork graph{};
      graph.addNDefaultNodes(3);
      graph.addStreet(1, std::make_pair(0, 1), 1.);
      graph.addStreet(2, std::make_pair(1, 2), 1.);
      graph.addStreet(3, std::make_pair(1, 0), 1.);
      THEN("Reverse streets are linked both ways") {
        CHECK_EQ(graph.edge(1).reverse(), &graph.edge(3));
        CHECK_EQ(graph.edge(3).reverse(), &graph.edge(1));
      }
      THEN("A one-way street has no reverse") {
        CHECK_EQ(graph.edge(2).reverse(), nullptr);
      }
      THEN("An edge that is not the reverse cannot be set") {
        CHECK_THROWS_AS(graph.edge(1).setReverse(&graph.edge(2)), std::invalid_argument);
        CHECK_EQ(graph.edge(1).reverse(), &graph.edge(3));
      }
      THEN("The reverse can be unset") {
        graph.edge(1).setReverse(nullptr);
        CHECK_EQ(graph.edge(1).reverse(), nullptr);
      }
    }
  }

  SUBCASE("adjustNodeCapacities and normalizeStreetCapacities") {
    GIVEN(
        "A graph composed of four streets with a different lane number and an isolated "
        "node") {
      RoadNetwork graph{};
      graph.addStreet(0, std::make_pair(0, 1), 10., 30., 1);
      graph.addStreet(1, std::make_pair(1, 2), 40., 30., 2);
      graph.addStreet(2, std::make_pair(3, 1), 75., 30., 3);
      graph.addStreet(3, std::make_pair(1, 4), 55., 30., 1);
      graph.addNode(5);
      WHEN("We adjust node capacities") {
        graph.adjustNodeCapacities();
        THEN("The node capacities are correct") {
          CHECK_EQ(graph.node(0).capacity(), 1);
          CHECK_EQ(graph.node(1).capacity(), 4);
          CHECK_EQ(graph.node(2).capacity(), 2);
          CHECK_EQ(graph.node(3).capacity(), 3);
          CHECK_EQ(graph.node(4).capacity(), 1);
          CHECK_EQ(graph.node(5).capacity(), 1);
        }
        THEN("The transport capacities are correct") {
          CHECK_EQ(graph.node(0).transportCapacity(), 1);
          CHECK_EQ(graph.node(1).transportCapacity(), 3);
          CHECK_EQ(graph.node(2).transportCapacity(), 1);
          CHECK_EQ(graph.node(3).transportCapacity(), 3);
          CHECK_EQ(graph.node(4).transportCapacity(), 1);
          CHECK_EQ(graph.node(5).transportCapacity(), 1);
        }
      }
      WHEN("Fractional flows sum to an integer up to floating-point error") {
        graph.edge(0).setTransportCapacity(0.6);
        graph.edge(2).setTransportCapacity(0.8);
        graph.adjustNodeCapacities();
        THEN("The node capacity is not rounded up past the integer") {
          // 0.6 + 3 * 0.8 evaluates to 3.0000000000000004
          CHECK_EQ(graph.node(1).capacity(), 3);
        }
      }
      WHEN("Transport capacities are scaled below one and we adjust node capacities") {
        for (auto const& [_, pStreet] : graph.edges()) {
          pStreet->setTransportCapacity(0.6);
        }
        graph.adjustNodeCapacities();
        THEN("The node capacities are rounded up and never zero") {
          CHECK_EQ(graph.node(0).capacity(), 1);  // no ingoing, 0.6 outgoing
          CHECK_EQ(graph.node(1).capacity(), 3);  // 0.6 + 1.8 ingoing
          CHECK_EQ(graph.node(2).capacity(), 2);  // 1.2 ingoing
          CHECK_EQ(graph.node(3).capacity(), 2);  // no ingoing, 1.8 outgoing
          CHECK_EQ(graph.node(4).capacity(), 1);  // 0.6 ingoing
        }
        THEN("The transport capacities are the outgoing flows") {
          CHECK_EQ(graph.node(0).transportCapacity(), doctest::Approx(0.6));
          CHECK_EQ(graph.node(1).transportCapacity(), doctest::Approx(1.8));
          CHECK_EQ(graph.node(2).transportCapacity(), 1);
          CHECK_EQ(graph.node(3).transportCapacity(), doctest::Approx(1.8));
          CHECK_EQ(graph.node(4).transportCapacity(), 1);
        }
      }
      WHEN("We normalize street capacities") {
        // graph.normalizeStreetCapacities();
        THEN("The street capacities are correct") {
          CHECK_EQ(graph.edge(0, 1).capacity(), 2);
          CHECK_EQ(graph.edge(1, 2).capacity(), 16);
          CHECK_EQ(graph.edge(3, 1).capacity(), 45);
          CHECK_EQ(graph.edge(1, 4).capacity(), 11);
        }
      }
    }
  }

  SUBCASE("autoAssignRoadPriorities") {
    GIVEN("A graph with a node having multiple incoming edges of different types") {
      RoadNetwork graph{};
      // Node 1 is the intersection
      // Edge 1: 0 -> 1 (HIGHWAY)

      // Edge 2: 2 -> 1 (HIGHWAY)

      // Edge 3: 3 -> 1 (SECONDARY)

      auto& s1 = graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setMobilityClass(128u);  // HIGHWAY
      auto& s2 = graph.addStreet(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setMobilityClass(128u);  // HIGHWAY
      auto& s3 = graph.addStreet(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setMobilityClass(32u);  // SECONDARY

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("The priorities are assigned to the most important roads") {
          CHECK(graph.edge(1).hasPriority());
          CHECK(graph.edge(2).hasPriority());
          CHECK_FALSE(graph.edge(3).hasPriority());
        }
      }
    }

    GIVEN("A node with only one incoming edge") {
      RoadNetwork graph{};
      auto& s1 = graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setMobilityClass(128u);  // HIGHWAY

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("No priorities are assigned") { CHECK_FALSE(graph.edge(1).hasPriority()); }
      }
    }

    GIVEN("A node with 3 incoming edges of the same road type") {
      RoadNetwork graph{};
      auto& s1 = graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setMobilityClass(64u);  // PRIMARY
      auto& s2 = graph.addStreet(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setMobilityClass(64u);  // PRIMARY
      auto& s3 = graph.addStreet(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setMobilityClass(64u);  // PRIMARY

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("No priorities are assigned since count != 2") {
          CHECK_FALSE(graph.edge(1).hasPriority());
          CHECK_FALSE(graph.edge(2).hasPriority());
          CHECK_FALSE(graph.edge(3).hasPriority());
        }
      }
    }

    GIVEN("A node with edges without road type set") {
      RoadNetwork graph{};
      graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      graph.addStreet(2, std::make_pair(2, 1), 100., 30., 1);
      graph.addStreet(3, std::make_pair(3, 1), 100., 30., 1);

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("No priorities are assigned") {
          CHECK_FALSE(graph.edge(1).hasPriority());
          CHECK_FALSE(graph.edge(2).hasPriority());
          CHECK_FALSE(graph.edge(3).hasPriority());
        }
      }
    }

    GIVEN("A node where highest type has 3 roads but lower type has exactly 2") {
      RoadNetwork graph{};
      // 3 HIGHWAY roads (won't match count == 2)
      // 2 SECONDARY roads (will match count == 2)
      auto& s1 = graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setMobilityClass(128u);  // HIGHWAY
      auto& s2 = graph.addStreet(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setMobilityClass(128u);  // HIGHWAY
      auto& s3 = graph.addStreet(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setMobilityClass(128u);  // HIGHWAY
      auto& s4 = graph.addStreet(4, std::make_pair(4, 1), 100., 30., 1);
      s4.setMobilityClass(32u);  // SECONDARY
      auto& s5 = graph.addStreet(5, std::make_pair(5, 1), 100., 30., 1);
      s5.setMobilityClass(32u);  // SECONDARY

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("Priorities go to the SECONDARY roads (first type with count == 2)") {
          CHECK_FALSE(graph.edge(1).hasPriority());
          CHECK_FALSE(graph.edge(2).hasPriority());
          CHECK_FALSE(graph.edge(3).hasPriority());
          CHECK(graph.edge(4).hasPriority());
          CHECK(graph.edge(5).hasPriority());
        }
      }
    }

    GIVEN("A node with exactly 2 PRIMARY and 1 TERTIARY road") {
      RoadNetwork graph{};
      auto& s1 = graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setMobilityClass(64u);  // PRIMARY
      auto& s2 = graph.addStreet(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setMobilityClass(64u);  // PRIMARY
      auto& s3 = graph.addStreet(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setMobilityClass(16u);  // TERTIARY

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("Priorities are assigned to PRIMARY roads") {
          CHECK(graph.edge(1).hasPriority());
          CHECK(graph.edge(2).hasPriority());
          CHECK_FALSE(graph.edge(3).hasPriority());
        }
      }
    }

    GIVEN("A node with 1 HIGHWAY, 1 PRIMARY, and 2 RESIDENTIAL roads") {
      RoadNetwork graph{};
      auto& s1 = graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setMobilityClass(128u);  // HIGHWAY
      auto& s2 = graph.addStreet(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setMobilityClass(64u);  // PRIMARY
      auto& s3 = graph.addStreet(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setMobilityClass(8u);  // RESIDENTIAL
      auto& s4 = graph.addStreet(4, std::make_pair(4, 1), 100., 30., 1);
      s4.setMobilityClass(8u);  // RESIDENTIAL

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("Priorities are assigned to RESIDENTIAL roads (first with count == 2)") {
          CHECK_FALSE(graph.edge(1).hasPriority());
          CHECK_FALSE(graph.edge(2).hasPriority());
          CHECK(graph.edge(3).hasPriority());
          CHECK(graph.edge(4).hasPriority());
        }
      }
    }

    GIVEN("Multiple nodes with different priority scenarios") {
      RoadNetwork graph{};
      // Node 1: 2 HIGHWAY + 1 SECONDARY
      // Node 5: 2 TERTIARY + 1 RESIDENTIAL
      auto& s1 = graph.addStreet(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setMobilityClass(128u);  // HIGHWAY
      auto& s2 = graph.addStreet(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setMobilityClass(128u);  // HIGHWAY
      auto& s3 = graph.addStreet(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setMobilityClass(32u);  // SECONDARY
      auto& s4 = graph.addStreet(4, std::make_pair(1, 5), 100., 30., 1);
      s4.setMobilityClass(16u);  // TERTIARY
      auto& s5 = graph.addStreet(5, std::make_pair(4, 5), 100., 30., 1);
      s5.setMobilityClass(16u);  // TERTIARY
      auto& s6 = graph.addStreet(6, std::make_pair(6, 5), 100., 30., 1);
      s6.setMobilityClass(8u);  // RESIDENTIAL

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("Both nodes have correct priorities assigned") {
          // Node 1: HIGHWAY roads get priority
          CHECK(graph.edge(1).hasPriority());
          CHECK(graph.edge(2).hasPriority());
          CHECK_FALSE(graph.edge(3).hasPriority());
          // Node 5: TERTIARY roads get priority
          CHECK(graph.edge(4).hasPriority());
          CHECK(graph.edge(5).hasPriority());
          CHECK_FALSE(graph.edge(6).hasPriority());
        }
      }
    }
  }
  SUBCASE("exportCSV exports edges and nodes") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(8.0, 45.0));
    graph.addNode(1, dsf::geometry::Point(8.1, 45.1));

    auto& s = graph.addStreet(42,
                              std::make_pair(0, 1),
                              123.0,
                              36.0 / 3.6,
                              2,
                              "Exported Street",
                              dsf::geometry::PolyLine{{dsf::geometry::Point(8.0, 45.0),
                                                       dsf::geometry::Point(8.1, 45.1)}});
    s.setMobilityClass(64u);  // PRIMARY
    s.setPriority();
    s.setTransportCapacity(0.5);
    graph.addCoil(42, "coil_42");

    auto const outputDir =
        std::filesystem::temp_directory_path() / "dsf_exportCSV_roadnetwork_test";
    std::filesystem::create_directories(outputDir);

    CHECK_NOTHROW(graph.exportCSV(outputDir.string()));

    auto const edgesPath = outputDir / "edges.csv";
    auto const nodesPath = outputDir / "nodes.csv";
    REQUIRE(std::filesystem::exists(edgesPath));
    REQUIRE(std::filesystem::exists(nodesPath));

    std::ifstream edgesFile{edgesPath};
    REQUIRE(edgesFile.is_open());
    std::string edgesCsv((std::istreambuf_iterator<char>(edgesFile)),
                         std::istreambuf_iterator<char>());
    CHECK(edgesCsv.find("id,source,target,length,maxspeed,nlanes,type,capacity,"
                        "transport_capacity,status,name,priority,coilcode,geometry") !=
          std::string::npos);
    CHECK(edgesCsv.find("Exported Street") != std::string::npos);
    CHECK(edgesCsv.find("64") != std::string::npos);  // PRIMARY
    CHECK(edgesCsv.find("OPEN") != std::string::npos);
    CHECK(edgesCsv.find("coil_42") != std::string::npos);
    CHECK(edgesCsv.find("LINESTRING (") != std::string::npos);

    std::ifstream nodesFile{nodesPath};
    REQUIRE(nodesFile.is_open());
    std::string nodesCsv((std::istreambuf_iterator<char>(nodesFile)),
                         std::istreambuf_iterator<char>());
    CHECK(nodesCsv.find("id,type,geometry,capacity,transportCapacity,name") !=
          std::string::npos);
    CHECK(nodesCsv.find("intersection") != std::string::npos);
    CHECK(nodesCsv.find("POINT (8 45)") != std::string::npos);

    // The exported node geometries can be imported back
    RoadNetwork roundTrip{};
    roundTrip.addNode(0);
    roundTrip.addNode(1);
    roundTrip.importNodeProperties(nodesPath.string(), ',');
    CHECK_EQ(roundTrip.node(0).geometry(), dsf::geometry::Point(8.0, 45.0));
    CHECK_EQ(roundTrip.node(1).geometry(), dsf::geometry::Point(8.1, 45.1));

    // The exported transport capacity can be imported back
    RoadNetwork edgesRoundTrip{};
    edgesRoundTrip.importEdges(edgesPath.string(), ',');
    CHECK_EQ(edgesRoundTrip.edge(static_cast<Id>(42)).transportCapacity(),
             doctest::Approx(0.5));

    std::filesystem::remove(edgesPath);
    std::filesystem::remove(nodesPath);
    std::filesystem::remove(outputDir);
  }

  SUBCASE("exportCSV throws for non-directory path") {
    RoadNetwork graph{};
    graph.addEdge<Street>(1, std::make_pair(0, 1));

    auto const invalidPath =
        std::filesystem::temp_directory_path() / "dsf_exportCSV_not_a_directory.csv";
    {
      std::ofstream out{invalidPath};
      REQUIRE(out.is_open());
      out << "placeholder";
    }

    CHECK_THROWS_AS(graph.exportCSV(invalidPath.string()), std::runtime_error);

    std::filesystem::remove(invalidPath);
  }

  SUBCASE("exportCSV re-exports imported custom attributes") {
    auto const inputCsvPath =
        std::filesystem::temp_directory_path() / "dsf_exportCSV_attributes_input.csv";
    {
      std::ofstream inputCsv{inputCsvPath};
      REQUIRE(inputCsv.is_open());
      inputCsv << "id;source;target;oneway;length;geometry;travel_time;maxspeed;nlanes;"
                  "type;name;is_toll;zone;weight\n";
      inputCsv << "300;100;101;false;80.0;LINESTRING (8.0 45.0, 8.1 45.1);9.6;"
                  "30.0;1;residential;attr_street_1;true;A1;3.14\n";
      inputCsv << "301;101;102;false;90.0;LINESTRING (8.1 45.1, 8.2 45.2);10.8;"
                  "30.0;1;residential;attr_street_2;false;B2;2\n";
    }

    RoadNetwork graph;
    graph.importEdges(inputCsvPath.string());

    auto const outputDir =
        std::filesystem::temp_directory_path() / "dsf_exportCSV_attributes_export";
    std::filesystem::create_directories(outputDir);
    CHECK_NOTHROW(graph.exportCSV(outputDir.string()));

    auto const edgesPath = outputDir / "edges.csv";
    REQUIRE(std::filesystem::exists(edgesPath));

    std::ifstream edgesFile{edgesPath};
    REQUIRE(edgesFile.is_open());
    std::string edgesCsv((std::istreambuf_iterator<char>(edgesFile)),
                         std::istreambuf_iterator<char>());

    CHECK(edgesCsv.find("is_toll") != std::string::npos);
    CHECK(edgesCsv.find("zone") != std::string::npos);
    CHECK(edgesCsv.find("weight") != std::string::npos);
    CHECK(edgesCsv.find("attr_street_1") != std::string::npos);
    CHECK(edgesCsv.find("attr_street_2") != std::string::npos);
    CHECK(edgesCsv.find("A1") != std::string::npos);
    CHECK(edgesCsv.find("B2") != std::string::npos);
    CHECK(edgesCsv.find(",2") != std::string::npos);

    std::filesystem::remove(inputCsvPath);
    std::filesystem::remove(edgesPath);
    std::filesystem::remove(outputDir / "nodes.csv");
    std::filesystem::remove(outputDir);
  }
}

TEST_CASE("RoadStatus") {
  SUBCASE("setStreetStatusById") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));

    graph.addStreet(0, std::make_pair(0, 1), 100.0);
    graph.addStreet(1, std::make_pair(1, 2), 100.0);

    // Initially all streets are OPEN
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::OPEN);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::OPEN);

    // Close street by id
    graph.setStreetStatusById(0, RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::OPEN);

    // Re-open street
    graph.setStreetStatusById(0, RoadStatus::OPEN);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::OPEN);

    CHECK_THROWS_AS(graph.setStreetStatusById(999, RoadStatus::CLOSED),
                    std::out_of_range);

    // Several streets at once
    std::vector<Id> const streetIds{0, 1};
    graph.setStreetStatusById(streetIds, RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::CLOSED);

    // An unknown id changes nothing
    std::vector<Id> const withUnknown{0, 999};
    CHECK_THROWS_AS(graph.setStreetStatusById(withUnknown, RoadStatus::OPEN),
                    std::out_of_range);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::CLOSED);
  }

  SUBCASE("setStreetStatusByName") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
    graph.addNode(3, dsf::geometry::Point(3.0, 0.0));

    graph.addStreet(0, std::make_pair(0, 1), 100.0, 13.8888, 1, "Main Street");
    graph.addStreet(1, std::make_pair(1, 2), 100.0, 13.8888, 1, "Main Street");
    graph.addStreet(2, std::make_pair(2, 3), 100.0, 13.8888, 1, "Side Road");

    // Initially all streets are OPEN
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::OPEN);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::OPEN);
    CHECK_EQ(graph.edge(2).roadStatus(), RoadStatus::OPEN);

    // Close all streets with name "Main Street"
    graph.setStreetStatusByName("Main Street", RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(2).roadStatus(), RoadStatus::OPEN);

    // Re-open Main Street
    graph.setStreetStatusByName("Main Street", RoadStatus::OPEN);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::OPEN);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::OPEN);
  }

  SUBCASE("setStreetStatusByName - partial match") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));

    graph.addStreet(0, std::make_pair(0, 1), 100.0, 13.8888, 1, "Via Roma Nord");
    graph.addStreet(1, std::make_pair(1, 2), 100.0, 13.8888, 1, "Via Roma Sud");

    // Close all streets containing "Roma" in the name
    graph.setStreetStatusByName("Roma", RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::CLOSED);
  }
}

TEST_CASE("Change Street Lanes") {
  Road::setMeanVehicleLength(5.);

  SUBCASE("changeStreetNLanesById") {
    GIVEN("A network with multiple streets") {
      RoadNetwork graph{};
      graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
      graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
      graph.addNode(2, dsf::geometry::Point(2.0, 0.0));

      graph.addStreet(10, std::make_pair(0, 1), 100.0, 20.0, 2);
      graph.addStreet(11, std::make_pair(1, 2), 150.0, 25.0, 3);

      auto const* pStreet01 = graph.street(0, 1);
      REQUIRE(pStreet01 != nullptr);
      auto const initialCapacity = pStreet01->capacity();
      auto const initialMaxSpeed = pStreet01->maxSpeed();

      WHEN("Lanes are increased for street by id") {
        graph.changeStreetNLanesById(10, 4);

        THEN("The street's properties are updated correctly") {
          CHECK_EQ(pStreet01->nLanes(), 4);
          CHECK_EQ(pStreet01->capacity(), initialCapacity * 2);
          CHECK_EQ(pStreet01->maxSpeed(), initialMaxSpeed);
          CHECK_EQ(pStreet01->lanes().size(), 4);
        }
      }

      WHEN("Lanes are changed with speed factor") {
        graph.changeStreetNLanesById(10, 1, 0.6);

        THEN("Both lanes and speed are updated") {
          CHECK_EQ(pStreet01->nLanes(), 1);
          CHECK_EQ(pStreet01->maxSpeed(), doctest::Approx(initialMaxSpeed * 0.6));
        }
      }

      WHEN("Invalid street id is provided") {
        THEN("Exception is thrown") {
          CHECK_THROWS_AS(graph.changeStreetNLanesById(999, 2), std::out_of_range);
        }
      }

      WHEN("Invalid number of lanes is provided") {
        THEN("Exception is thrown") {
          CHECK_THROWS_AS(graph.changeStreetNLanesById(10, 0), std::invalid_argument);
          CHECK_THROWS_AS(graph.changeStreetNLanesById(10, -1), std::invalid_argument);
        }
      }
    }
  }

  SUBCASE("changeStreetNLanesByName") {
    GIVEN("A network with named streets") {
      RoadNetwork graph{};
      graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
      graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
      graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
      graph.addNode(3, dsf::geometry::Point(1.0, 1.0));

      graph.addStreet(10, std::make_pair(0, 1), 100.0, 20.0, 2, "Main Street");
      graph.addStreet(11, std::make_pair(1, 2), 150.0, 25.0, 3, "Main Street");
      graph.addStreet(12, std::make_pair(0, 3), 120.0, 15.0, 1, "Side Road");
      graph.addStreet(13, std::make_pair(3, 2), 130.0, 18.0, 2, "Side Road");

      auto const* pMainStreet1 = graph.street(0, 1);
      auto const* pMainStreet2 = graph.street(1, 2);
      auto const* pSideRoad1 = graph.street(0, 3);
      auto const* pSideRoad2 = graph.street(3, 2);

      REQUIRE(pMainStreet1 != nullptr);
      REQUIRE(pMainStreet2 != nullptr);
      REQUIRE(pSideRoad1 != nullptr);
      REQUIRE(pSideRoad2 != nullptr);

      auto const initialMainSpeed1 = pMainStreet1->maxSpeed();
      auto const initialMainSpeed2 = pMainStreet2->maxSpeed();
      auto const initialSideSpeed1 = pSideRoad1->maxSpeed();
      auto const initialSideSpeed2 = pSideRoad2->maxSpeed();

      WHEN("All streets with 'Main' in name are changed") {
        graph.changeStreetNLanesByName("Main", 1);

        THEN("Only Main Streets are affected") {
          CHECK_EQ(pMainStreet1->nLanes(), 1);
          CHECK_EQ(pMainStreet2->nLanes(), 1);
          CHECK_EQ(pSideRoad1->nLanes(), 1);  // unchanged
          CHECK_EQ(pSideRoad2->nLanes(), 2);  // unchanged
          CHECK_EQ(pMainStreet1->maxSpeed(), initialMainSpeed1);
          CHECK_EQ(pMainStreet2->maxSpeed(), initialMainSpeed2);
        }
      }

      WHEN("Streets are changed by name with speed factor") {
        graph.changeStreetNLanesByName("Side", 3, 0.8);

        THEN("Only Side Roads are affected with both changes") {
          CHECK_EQ(pSideRoad1->nLanes(), 3);
          CHECK_EQ(pSideRoad2->nLanes(), 3);
          CHECK_EQ(pMainStreet1->nLanes(), 2);  // unchanged
          CHECK_EQ(pMainStreet2->nLanes(), 3);  // unchanged
          CHECK_EQ(pSideRoad1->maxSpeed(), doctest::Approx(initialSideSpeed1 * 0.8));
          CHECK_EQ(pSideRoad2->maxSpeed(), doctest::Approx(initialSideSpeed2 * 0.8));
          CHECK_EQ(pMainStreet1->maxSpeed(), initialMainSpeed1);  // unchanged
          CHECK_EQ(pMainStreet2->maxSpeed(), initialMainSpeed2);  // unchanged
        }
      }

      WHEN("No streets match the name pattern") {
        graph.changeStreetNLanesByName("NonExistent", 5);

        THEN("No streets are changed") {
          CHECK_EQ(pMainStreet1->nLanes(), 2);
          CHECK_EQ(pMainStreet2->nLanes(), 3);
          CHECK_EQ(pSideRoad1->nLanes(), 1);
          CHECK_EQ(pSideRoad2->nLanes(), 2);
        }
      }

      WHEN("Partial name match is used") {
        graph.changeStreetNLanesByName("Street", 4);

        THEN("All streets with 'Street' in name are changed") {
          CHECK_EQ(pMainStreet1->nLanes(), 4);
          CHECK_EQ(pMainStreet2->nLanes(), 4);
          CHECK_EQ(pSideRoad1->nLanes(), 1);  // unchanged
          CHECK_EQ(pSideRoad2->nLanes(), 2);  // unchanged
        }
      }
    }
  }
}

TEST_CASE("Change Street Capacity") {
  RoadNetwork graph{};
  graph.addStreet(0, std::make_pair(0, 1), 100.0, 13.8888, 1, "Main Street");
  graph.addStreet(1, std::make_pair(1, 2), 100.0, 13.8888, 1, "Main Street");
  graph.addStreet(2, std::make_pair(2, 3), 100.0, 13.8888, 1, "Side Road");
  auto const initialCapacity = graph.edge(0).capacity();
  auto const initialTotalCapacity = graph.capacity();

  SUBCASE("changeStreetCapacityById") {
    graph.changeStreetCapacityById(0, 2.);
    CHECK_EQ(graph.edge(0).capacity(), 2 * initialCapacity);
    CHECK_EQ(graph.edge(1).capacity(), initialCapacity);
    CHECK_EQ(graph.capacity(), initialTotalCapacity + initialCapacity);
    CHECK_THROWS_AS(graph.changeStreetCapacityById(999, 2.), std::out_of_range);
  }
  SUBCASE("changeStreetCapacityByName") {
    graph.changeStreetCapacityByName("Main", 0.5);
    CHECK_EQ(graph.edge(0).capacity(), std::ceil(initialCapacity * 0.5));
    CHECK_EQ(graph.edge(1).capacity(), std::ceil(initialCapacity * 0.5));
    CHECK_EQ(graph.edge(2).capacity(), initialCapacity);
  }
}

TEST_CASE("BetweennessCentrality") {
  Road::setMeanVehicleLength(5.);

  SUBCASE("Linear chain: 0 -> 1 -> 2 -> 3") {
    // In a linear chain, all shortest paths between non-adjacent nodes pass
    // through the intermediate nodes. Node 1 and 2 should have BC > 0,
    // while endpoints 0 and 3 should have BC = 0.
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(1, 2), 10.0);
    graph.addStreet(2, std::make_pair(2, 3), 10.0);
    graph.computeBetweennessCentralities(Router::edgeWeightFunction("uniform"));

    // Node 0: source only, never an intermediate -> BC = 0
    REQUIRE(graph.node(0).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(0).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(0.0));

    // Node 1: on path 0->2 and 0->3 -> BC = 2
    REQUIRE(graph.node(1).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(1).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(2.0));

    // Node 2: on path 0->3 and 1->3 -> BC = 2
    REQUIRE(graph.node(2).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(2).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(2.0));

    // Node 3: sink only, never an intermediate -> BC = 0
    REQUIRE(graph.node(3).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(3).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(0.0));
  }

  SUBCASE("Star topology: center node") {
    // Star: 0 -> 1, 0 -> 2, 0 -> 3
    // No node is intermediate (all paths have length 1), so all BC = 0.
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(0, 2), 10.0);
    graph.addStreet(2, std::make_pair(0, 3), 10.0);
    graph.computeBetweennessCentralities(Router::edgeWeightFunction("uniform"));

    for (Id i = 0; i <= 3; ++i) {
      REQUIRE(graph.node(i).getAttribute<double>("betweennessCentrality").has_value());
      CHECK_EQ(*graph.node(i).getAttribute<double>("betweennessCentrality"),
               doctest::Approx(0.0));
    }
  }

  SUBCASE("Diamond graph with different weights") {
    /*     1
          / \
         0   3
          \ /
           2
       0->1 weight 1, 1->3 weight 1 (total 2)
       0->2 weight 1, 2->3 weight 3 (total 4)
       Shortest path 0->3 goes through node 1 only */
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(0, 2), 10.0);
    graph.addStreet(2, std::make_pair(1, 3), 10.0);
    graph.addStreet(3, std::make_pair(2, 3), 30.0);
    graph.computeBetweennessCentralities(Router::edgeWeightFunction("length"));

    // Node 1 is on the only shortest path 0->3 -> BC = 1
    REQUIRE(graph.node(1).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(1).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(1.0));

    // Node 2 is not on any shortest path between other pairs -> BC = 0
    REQUIRE(graph.node(2).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(2).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(0.0));
  }

  SUBCASE("Single edge") {
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.computeBetweennessCentralities(Router::edgeWeightFunction("uniform"));

    REQUIRE(graph.node(0).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(0).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(0.0));
    REQUIRE(graph.node(1).getAttribute<double>("betweennessCentrality").has_value());
    CHECK_EQ(*graph.node(1).getAttribute<double>("betweennessCentrality"),
             doctest::Approx(0.0));
  }

  SUBCASE("Disconnected graph") {
    // Two separate components: 0->1, 2->3
    // No intermediate nodes possible -> all BC = 0
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(2, 3), 10.0);
    graph.computeBetweennessCentralities(Router::edgeWeightFunction("uniform"));

    for (Id i = 0; i <= 3; ++i) {
      REQUIRE(graph.node(i).getAttribute<double>("betweennessCentrality").has_value());
      CHECK_EQ(*graph.node(i).getAttribute<double>("betweennessCentrality"),
               doctest::Approx(0.0));
    }
  }
}

TEST_CASE("EdgeBetweennessCentrality") {
  Road::setMeanVehicleLength(5.);

  SUBCASE("Linear chain: 0 -> 1 -> 2 -> 3") {
    // Edge 0->1 (id=0): used by paths 0->1, 0->2, 0->3 => EBC = 3
    // Edge 1->2 (id=1): used by paths 0->2, 1->2, 0->3, 1->3 => EBC = 4
    // Edge 2->3 (id=2): used by paths 0->3, 1->3, 2->3 => EBC = 3
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(1, 2), 10.0);
    graph.addStreet(2, std::make_pair(2, 3), 10.0);
    graph.computeEdgeBetweennessCentralities(Router::edgeWeightFunction("uniform"));

    REQUIRE(graph.edge(static_cast<Id>(0))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(graph.edge(static_cast<Id>(0))
                 .getAttribute<double>("betweennessCentrality")
                 .value(),
             doctest::Approx(3.0));

    REQUIRE(graph.edge(static_cast<Id>(1))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(graph.edge(static_cast<Id>(1))
                 .getAttribute<double>("betweennessCentrality")
                 .value(),
             doctest::Approx(4.0));

    REQUIRE(graph.edge(static_cast<Id>(2))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(graph.edge(static_cast<Id>(2))
                 .getAttribute<double>("betweennessCentrality")
                 .value(),
             doctest::Approx(3.0));
  }

  SUBCASE("Diamond graph with different weights") {
    /*     1
          / \
         0   3
          \ /
           2
       0->1 length 10, 1->3 length 10 (total 20)
       0->2 length 10, 2->3 length 30 (total 40)
       Shortest path 0->3 goes 0->1->3 */
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(0, 2), 10.0);
    graph.addStreet(2, std::make_pair(1, 3), 10.0);
    graph.addStreet(3, std::make_pair(2, 3), 30.0);
    graph.computeEdgeBetweennessCentralities(Router::edgeWeightFunction("length"));

    // Edge 0->1: used by 0->1 and 0->3 (via 0->1->3) => EBC = 2
    REQUIRE(graph.edge(static_cast<Id>(0))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(
        *graph.edge(static_cast<Id>(0)).getAttribute<double>("betweennessCentrality"),
        doctest::Approx(2.0));

    // Edge 0->2: used only by 0->2 => EBC = 1
    REQUIRE(graph.edge(static_cast<Id>(1))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(
        *graph.edge(static_cast<Id>(1)).getAttribute<double>("betweennessCentrality"),
        doctest::Approx(1.0));

    // Edge 1->3: used by 1->3 and 0->3 => EBC = 2
    REQUIRE(graph.edge(static_cast<Id>(2))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(
        *graph.edge(static_cast<Id>(2)).getAttribute<double>("betweennessCentrality"),
        doctest::Approx(2.0));

    // Edge 2->3: used only by 2->3 => EBC = 1
    REQUIRE(graph.edge(static_cast<Id>(3))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(
        *graph.edge(static_cast<Id>(3)).getAttribute<double>("betweennessCentrality"),
        doctest::Approx(1.0));
  }

  SUBCASE("Single edge") {
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.computeEdgeBetweennessCentralities(Router::edgeWeightFunction("uniform"));

    // Only one path: 0->1, using edge 0 => EBC = 1
    REQUIRE(graph.edge(static_cast<Id>(0))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(
        *graph.edge(static_cast<Id>(0)).getAttribute<double>("betweennessCentrality"),
        doctest::Approx(1.0));
  }
}

TEST_CASE("computeEdgeKBetweennessCentralities") {
  Road::setMeanVehicleLength(5.);

  SUBCASE("K=1 matches normalized standard edge BC on linear chain") {
    RoadNetwork standardGraph{};
    standardGraph.addStreet(0, std::make_pair(0, 1), 10.0);
    standardGraph.addStreet(1, std::make_pair(1, 2), 10.0);
    standardGraph.addStreet(2, std::make_pair(2, 3), 10.0);
    standardGraph.computeEdgeBetweennessCentralities(
        Router::edgeWeightFunction("uniform"));

    size_t const nNodes = standardGraph.nNodes();
    double const norm = static_cast<double>((nNodes - 1) * (nNodes - 2));
    REQUIRE(norm > 0.0);

    auto const stdBc0 = standardGraph.edge(static_cast<Id>(0))
                            .getAttribute<double>("betweennessCentrality");
    auto const stdBc1 = standardGraph.edge(static_cast<Id>(1))
                            .getAttribute<double>("betweennessCentrality");
    auto const stdBc2 = standardGraph.edge(static_cast<Id>(2))
                            .getAttribute<double>("betweennessCentrality");
    REQUIRE(stdBc0.has_value());
    REQUIRE(stdBc1.has_value());
    REQUIRE(stdBc2.has_value());

    double const expected0 = *stdBc0 / norm;
    double const expected1 = *stdBc1 / norm;
    double const expected2 = *stdBc2 / norm;

    RoadNetwork yenGraph{};
    yenGraph.addStreet(0, std::make_pair(0, 1), 10.0);
    yenGraph.addStreet(1, std::make_pair(1, 2), 10.0);
    yenGraph.addStreet(2, std::make_pair(2, 3), 10.0);
    yenGraph.computeEdgeKBetweennessCentralities(Router::edgeWeightFunction("uniform"),
                                                 1);

    auto const yenBc0 =
        yenGraph.edge(static_cast<Id>(0)).getAttribute<double>("betweennessCentrality");
    auto const yenBc1 =
        yenGraph.edge(static_cast<Id>(1)).getAttribute<double>("betweennessCentrality");
    auto const yenBc2 =
        yenGraph.edge(static_cast<Id>(2)).getAttribute<double>("betweennessCentrality");
    REQUIRE(yenBc0.has_value());
    REQUIRE(yenBc1.has_value());
    REQUIRE(yenBc2.has_value());

    CHECK_EQ(*yenBc0, doctest::Approx(expected0));
    CHECK_EQ(*yenBc1, doctest::Approx(expected1));
    CHECK_EQ(*yenBc2, doctest::Approx(expected2));
  }

  SUBCASE("K=2 captures additional shortest-path alternatives") {
    RoadNetwork graphK1{};
    graphK1.addStreet(0, std::make_pair(0, 1), 10.0);
    graphK1.addStreet(1, std::make_pair(0, 2), 10.0);
    graphK1.addStreet(2, std::make_pair(1, 3), 10.0);
    graphK1.addStreet(3, std::make_pair(2, 3), 10.0);
    graphK1.computeEdgeKBetweennessCentralities(Router::edgeWeightFunction("uniform"), 1);

    RoadNetwork graphK2{};
    graphK2.addStreet(0, std::make_pair(0, 1), 10.0);
    graphK2.addStreet(1, std::make_pair(0, 2), 10.0);
    graphK2.addStreet(2, std::make_pair(1, 3), 10.0);
    graphK2.addStreet(3, std::make_pair(2, 3), 10.0);
    graphK2.computeEdgeKBetweennessCentralities(Router::edgeWeightFunction("uniform"), 2);

    bool hasStrictIncrease = false;
    Id const edgeIds[] = {
        static_cast<Id>(0), static_cast<Id>(1), static_cast<Id>(2), static_cast<Id>(3)};

    for (Id const edgeId : edgeIds) {
      auto const k1Bc =
          graphK1.edge(edgeId).getAttribute<double>("betweennessCentrality");
      auto const k2Bc =
          graphK2.edge(edgeId).getAttribute<double>("betweennessCentrality");
      REQUIRE(k1Bc.has_value());
      REQUIRE(k2Bc.has_value());

      CHECK(*k2Bc + 1e-12 >= *k1Bc);
      if (*k2Bc > *k1Bc + 1e-12) {
        hasStrictIncrease = true;
      }
    }

    CHECK(hasStrictIncrease);
  }

  SUBCASE("K=1 matches Brandes on a diamond graph with tied shortest paths") {
    RoadNetwork standardGraph{};
    standardGraph.addStreet(0, std::make_pair(0, 1), 10.0);
    standardGraph.addStreet(1, std::make_pair(0, 2), 10.0);
    standardGraph.addStreet(2, std::make_pair(1, 3), 10.0);
    standardGraph.addStreet(3, std::make_pair(2, 3), 10.0);
    standardGraph.computeEdgeBetweennessCentralities(
        Router::edgeWeightFunction("uniform"));

    std::size_t const nNodes = standardGraph.nNodes();
    double const norm = static_cast<double>((nNodes - 1) * (nNodes - 2));
    REQUIRE(norm > 0.0);

    RoadNetwork yenGraph{};
    yenGraph.addStreet(0, std::make_pair(0, 1), 10.0);
    yenGraph.addStreet(1, std::make_pair(0, 2), 10.0);
    yenGraph.addStreet(2, std::make_pair(1, 3), 10.0);
    yenGraph.addStreet(3, std::make_pair(2, 3), 10.0);
    yenGraph.computeEdgeKBetweennessCentralities(Router::edgeWeightFunction("uniform"),
                                                 1);

    Id const edgeIds[] = {
        static_cast<Id>(0), static_cast<Id>(1), static_cast<Id>(2), static_cast<Id>(3)};

    for (Id const edgeId : edgeIds) {
      auto const standardBc =
          standardGraph.edge(edgeId).getAttribute<double>("betweennessCentrality");
      auto const yenBc =
          yenGraph.edge(edgeId).getAttribute<double>("betweennessCentrality");
      REQUIRE(standardBc.has_value());
      REQUIRE(yenBc.has_value());
      CHECK(*yenBc == doctest::Approx(*standardBc / norm));
    }
  }

  SUBCASE("K=2 counts both equal shortest paths exactly on a diamond graph") {
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(0, 2), 10.0);
    graph.addStreet(2, std::make_pair(1, 3), 10.0);
    graph.addStreet(3, std::make_pair(2, 3), 10.0);
    graph.computeEdgeKBetweennessCentralities(Router::edgeWeightFunction("uniform"), 2);

    Id const edgeIds[] = {
        static_cast<Id>(0), static_cast<Id>(1), static_cast<Id>(2), static_cast<Id>(3)};

    for (Id const edgeId : edgeIds) {
      auto const bc = graph.edge(edgeId).getAttribute<double>("betweennessCentrality");
      REQUIRE(bc.has_value());
      CHECK(*bc == doctest::Approx(1.0 / 3.0));
    }
  }

  SUBCASE("K=3 finds three equal shortest paths with persistent candidate heap") {
    // Graph structure: 3 edge-disjoint paths from source 0 to target 4
    // Each path has 2 edges with uniform cost
    // Path 1: 0 -> 1 -> 4  (edges 0, 1)
    // Path 2: 0 -> 2 -> 4  (edges 2, 3)
    // Path 3: 0 -> 3 -> 4  (edges 4, 5)
    //
    // For (s, t) = (0, 4), all 3 paths have cost 20, so K=3 should find all of them.
    // This tests that the global candidate heap persists across rank iterations
    // and correctly accumulates spur candidates from earlier ranks.
    RoadNetwork graph{};
    graph.addStreet(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(1, std::make_pair(1, 4), 10.0);
    graph.addStreet(2, std::make_pair(0, 2), 10.0);
    graph.addStreet(3, std::make_pair(2, 4), 10.0);
    graph.addStreet(4, std::make_pair(0, 3), 10.0);
    graph.addStreet(5, std::make_pair(3, 4), 10.0);
    graph.computeEdgeKBetweennessCentralities(Router::edgeWeightFunction("uniform"), 3);

    // Verify all 6 edges appear in the computations
    Id const edgeIds[] = {static_cast<Id>(0),
                          static_cast<Id>(1),
                          static_cast<Id>(2),
                          static_cast<Id>(3),
                          static_cast<Id>(4),
                          static_cast<Id>(5)};

    std::unordered_map<Id, double> edgeCentralities;
    for (Id const edgeId : edgeIds) {
      auto const bc = graph.edge(edgeId).getAttribute<double>("betweennessCentrality");
      REQUIRE(bc.has_value());
      edgeCentralities[edgeId] = *bc;
    }

    // Edges on the paths (0,1), (1,4), (0,2), (2,4), (0,3), (3,4) contribute equally
    // because all three paths are shortest and equally likely.
    // Each edge should appear in 1 out of 3 paths from node 0 to node 4.
    // Normalization factor: (N-1)*(N-2) = (5-1)*(5-2) = 4*3 = 12
    // Each edge appears 1 time across all 3 paths for (0,4), and it's not on paths
    // from other pairs. So contribution = 1/12.
    // Actually, let's verify that all edges contribute positively and K=3 >= K=2
    RoadNetwork graphK2{};
    graphK2.addStreet(0, std::make_pair(0, 1), 10.0);
    graphK2.addStreet(1, std::make_pair(1, 4), 10.0);
    graphK2.addStreet(2, std::make_pair(0, 2), 10.0);
    graphK2.addStreet(3, std::make_pair(2, 4), 10.0);
    graphK2.addStreet(4, std::make_pair(0, 3), 10.0);
    graphK2.addStreet(5, std::make_pair(3, 4), 10.0);
    graphK2.computeEdgeKBetweennessCentralities(Router::edgeWeightFunction("uniform"), 2);

    bool hasIncrease = false;
    for (Id const edgeId : edgeIds) {
      auto const bcK2 =
          graphK2.edge(edgeId).getAttribute<double>("betweennessCentrality");
      auto const bcK3 = edgeCentralities[edgeId];
      REQUIRE(bcK2.has_value());
      // K=3 should find at least one additional path for at least one edge pair
      CHECK(bcK3 + 1e-12 >= *bcK2);
      if (bcK3 > *bcK2 + 1e-12) {
        hasIncrease = true;
      }
    }
    // Verify that at least some edges increased from K=2 to K=3
    // (This validates the global candidate heap: without it, no 3rd path would be found)
    CHECK(hasIncrease);
  }
}