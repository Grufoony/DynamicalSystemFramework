#include "dsf/mobility/RoadNetwork.hpp"
#include "dsf/base/Node.hpp"
#include "dsf/mobility/Road.hpp"
#include "dsf/mobility/Street.hpp"

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>

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
  }
  SUBCASE("Construction with addStreets") {
    Street s1(1, std::make_pair(0, 1));
    Street s2(2, std::make_pair(1, 2));
    Street s3(3, std::make_pair(0, 2));
    Street s4(4, std::make_pair(0, 3));
    Street s5(5, std::make_pair(2, 3));
    RoadNetwork graph;
    graph.addStreets(s1, s2, s3, s4, s5);
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
            switch (pEdge->roadType()) {
              case RoadType::HIGHWAY:
                ++nHighways;
                break;
              case RoadType::PRIMARY:
                ++nPrimary;
                break;
              case RoadType::SECONDARY:
                ++nSecondary;
                break;
              case RoadType::TERTIARY:
                ++nTertiary;
                break;
              case RoadType::RESIDENTIAL:
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
                    "residential;priority_street_true;1\n";
          tmpCsv << "101;2;3;False;120.0;LINESTRING (8.1 45.1, 8.2 45.2);14.4;30.0;1;"
                    "residential;priority_street_false;0\n";
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
        Street s1{1, std::make_pair(0, 1), 30., 15., 3};
        s1.setPriority();
        Street s2{11, std::make_pair(2, 1), 30., 15., 3};
        s2.setPriority();
        Street s3{16, std::make_pair(3, 1), 30., 15., 1};
        Street s4{21, std::make_pair(4, 1), 30., 15., 2};
        RoadNetwork graph2;
        graph2.addNDefaultNodes(5);
        graph2.makeTrafficLight(1, 120);
        graph2.addStreets(s1, s2, s3, s4);
        for (auto const& [streetId, pStreet] : graph2.edges()) {
          pStreet->setCapacity(2 * pStreet->nLanes());
        }
        WHEN("We auto-init Traffic Lights") {
          graph2.autoInitTrafficLights();
          THEN("Parameters are correctly set") {
            auto& tl{graph2.node<dsf::mobility::TrafficLight>(1)};
            CHECK_EQ(tl.cycleTime(), 120);
            auto const& cycles{tl.cycles()};
            CHECK_EQ(cycles.size(), 4);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::RIGHTANDSTRAIGHT).greenTime(), 48);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::LEFT).greenTime(), 24);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::RIGHTANDSTRAIGHT).phase(), 0);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::LEFT).phase(), 48);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::RIGHTANDSTRAIGHT).greenTime(), 48);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::LEFT).greenTime(), 24);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::RIGHTANDSTRAIGHT).phase(), 0);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::LEFT).phase(), 48);
            CHECK_EQ(cycles.at(16).at(dsf::Direction::ANY).greenTime(), 48);
            CHECK_EQ(cycles.at(16).at(dsf::Direction::ANY).phase(), 72);
            CHECK_EQ(cycles.at(21).at(dsf::Direction::ANY).greenTime(), 48);
            CHECK_EQ(cycles.at(21).at(dsf::Direction::ANY).phase(), 72);
          }
        }
      }
      GIVEN("A traffic light with streets having same names (priority by name)") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0, 100);
        // Constructor: id, nodePair, length, maxSpeed, nLanes, name
        Street s1{1, std::make_pair(1, 0), 50., 15., 1, "Main Street"};
        Street s2{2, std::make_pair(2, 0), 50., 15., 1, "Main Street"};
        Street s3{3, std::make_pair(3, 0), 50., 15., 1, "Side Street"};
        graph.addStreets(s1, s2, s3);

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
        graph.makeTrafficLight(0, 100);
        // All streets have unique names (no name-based priority) but different speeds
        Street s1{1, std::make_pair(1, 0), 50., 50., 1, "Road A"};  // Higher speed
        Street s2{2, std::make_pair(2, 0), 50., 50., 1, "Road B"};  // Higher speed
        Street s3{3, std::make_pair(3, 0), 50., 30., 1, "Road C"};  // Lower speed
        graph.addStreets(s1, s2, s3);

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
        graph.makeTrafficLight(0, 100);
        // All unique names, same speed, different lane counts
        Street s1{1, std::make_pair(1, 0), 50., 30., 2, "Road A"};  // Higher lanes
        Street s2{2, std::make_pair(2, 0), 50., 30., 2, "Road B"};  // Higher lanes
        Street s3{3, std::make_pair(3, 0), 50., 30., 1, "Road C"};  // Lower lanes
        graph.addStreets(s1, s2, s3);

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
        graph.makeTrafficLight(0, 100);
        Street s1{1, std::make_pair(1, 0), 50., 30., 1};
        Street s2{2, std::make_pair(2, 0), 50., 30., 1};
        graph.addStreets(s1, s2);

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Node is converted to intersection") {
            CHECK_FALSE(graph.node(0).isTrafficLight());
            CHECK(graph.node(0).isIntersection());
          }
        }
      }
      GIVEN("A traffic light with custom mainRoadPercentage") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0, 100);
        Street s1{1, std::make_pair(1, 0), 50., 15., 1};
        s1.setPriority();
        Street s2{2, std::make_pair(2, 0), 50., 15., 1};
        s2.setPriority();
        Street s3{3, std::make_pair(3, 0), 50., 15., 1};
        graph.addStreets(s1, s2, s3);

        WHEN("We auto-init Traffic Lights with 70% main road percentage") {
          graph.autoInitTrafficLights(0.7);
          THEN("Green times reflect the custom percentage") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& cycles{tl.cycles()};
            // Main road gets 70% of 100 = 70
            CHECK_EQ(cycles.at(1).at(dsf::Direction::ANY).greenTime(), 70);
            CHECK_EQ(cycles.at(2).at(dsf::Direction::ANY).greenTime(), 70);
            // Secondary road gets 30% of 100 = 30
            CHECK_EQ(cycles.at(3).at(dsf::Direction::ANY).greenTime(), 30);
            // Phases: main roads start at 0, secondary at 70
            CHECK_EQ(cycles.at(1).at(dsf::Direction::ANY).phase(), 0);
            CHECK_EQ(cycles.at(2).at(dsf::Direction::ANY).phase(), 0);
            CHECK_EQ(cycles.at(3).at(dsf::Direction::ANY).phase(), 70);
          }
        }
      }
      GIVEN("A traffic light already initialized with cycles") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        auto& tl = graph.makeTrafficLight(0, 100);
        Street s1{1, std::make_pair(1, 0), 50., 15., 1};
        Street s2{2, std::make_pair(2, 0), 50., 15., 1};
        Street s3{3, std::make_pair(3, 0), 50., 15., 1};
        graph.addStreets(s1, s2, s3);
        // Pre-initialize cycles
        tl.setCycle(1, dsf::Direction::ANY, dsf::mobility::TrafficLightCycle{50, 0});

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Traffic light is not modified") {
            auto const& cycles{tl.cycles()};
            CHECK_EQ(cycles.size(), 1);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::ANY).greenTime(), 50);
          }
        }
      }
      GIVEN("A traffic light with 4 streets all having the same characteristics") {
        RoadNetwork graph;
        graph.addNDefaultNodes(5);
        graph.makeTrafficLight(0, 120);
        // All streets identical with unique names - will fall back to angle or capacity-based selection
        Street s1{1, std::make_pair(1, 0), 50., 30., 1, "Road A"};
        Street s2{2, std::make_pair(2, 0), 50., 30., 1, "Road B"};
        Street s3{3, std::make_pair(3, 0), 50., 30., 1, "Road C"};
        Street s4{4, std::make_pair(4, 0), 50., 30., 1, "Road D"};
        graph.addStreets(s1, s2, s3, s4);
        for (auto const& [streetId, pStreet] : graph.edges()) {
          pStreet->setCapacity(10);
        }

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("Fallback assigns exactly 2 priorities") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& priorities{tl.streetPriorities()};
            CHECK_EQ(priorities.size(), 2);
            // Cycles should be initialized for all 4 streets
            auto const& cycles{tl.cycles()};
            CHECK_EQ(cycles.size(), 4);
          }
        }
      }
      GIVEN("A traffic light with 3-lane streets") {
        RoadNetwork graph;
        graph.addNDefaultNodes(4);
        graph.makeTrafficLight(0, 90);
        Street s1{1, std::make_pair(1, 0), 50., 15., 3};
        s1.setPriority();
        Street s2{2, std::make_pair(2, 0), 50., 15., 3};
        s2.setPriority();
        Street s3{3, std::make_pair(3, 0), 50., 15., 3};
        graph.addStreets(s1, s2, s3);

        WHEN("We auto-init Traffic Lights") {
          graph.autoInitTrafficLights();
          THEN("3-lane streets get RIGHTANDSTRAIGHT and LEFT cycles") {
            auto& tl{graph.node<dsf::mobility::TrafficLight>(0)};
            auto const& cycles{tl.cycles()};
            // Each 3-lane street should have 2 cycle entries
            CHECK(cycles.at(1).contains(dsf::Direction::RIGHTANDSTRAIGHT));
            CHECK(cycles.at(1).contains(dsf::Direction::LEFT));
            CHECK(cycles.at(2).contains(dsf::Direction::RIGHTANDSTRAIGHT));
            CHECK(cycles.at(2).contains(dsf::Direction::LEFT));
            CHECK(cycles.at(3).contains(dsf::Direction::RIGHTANDSTRAIGHT));
            CHECK(cycles.at(3).contains(dsf::Direction::LEFT));
            // Main road (60% of 90 = 54): RIGHTANDSTRAIGHT = 36, LEFT = 18
            CHECK_EQ(cycles.at(1).at(dsf::Direction::RIGHTANDSTRAIGHT).greenTime(), 36);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::LEFT).greenTime(), 18);
            // Secondary road (40% of 90 = 36): RIGHTANDSTRAIGHT = 24, LEFT = 12
            CHECK_EQ(cycles.at(3).at(dsf::Direction::RIGHTANDSTRAIGHT).greenTime(), 24);
            CHECK_EQ(cycles.at(3).at(dsf::Direction::LEFT).greenTime(), 12);
          }
        }
      }
    }
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(Street{1, std::make_pair(0, 1)});
      WHEN("We make node 0 a traffic light") {
        auto& tl = graph.makeTrafficLight(0, 60);
        THEN("The node 0 is a traffic light") { CHECK(graph.node(0).isTrafficLight()); }
        THEN("The traffic light has the correct parameters") {
          CHECK_EQ(tl.id(), 0);
          CHECK_EQ(tl.cycleTime(), 60);
        }
      }
    }
  }
  SUBCASE("make roundabout") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(Street{1, std::make_pair(0, 1)});
      WHEN("We make node 0 a roundabout") {
        graph.makeRoundabout(0);
        THEN("The node 0 is a roundabout") { CHECK(graph.node(0).isRoundabout()); }
      }
    }
  }
  SUBCASE("add coil on a street") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addEdge(Street{0, std::make_pair(0, 1)});
      WHEN("We add a coil to the street") {
        graph.addCoil(0);
        THEN("The street has a coil") { CHECK(graph.edge(0).hasCoil()); }
      }
    }
  }
  SUBCASE("street and oppositeStreet") {
    GIVEN("A RoadNetwork object with two streets") {
      RoadNetwork graph{};
      Street street{1, std::make_pair(0, 1), 1.};
      Street opposite{2, std::make_pair(1, 0), 1.};
      graph.addNDefaultNodes(2);
      graph.addStreets(street, opposite);
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
  }

  SUBCASE("adjustNodeCapacities and normalizeStreetCapacities") {
    GIVEN("A graph composed of three streets with a different lane number") {
      Street s1(0, std::make_pair(0, 1), 10., 30., 1);
      Street s2(1, std::make_pair(1, 2), 40., 30., 2);
      Street s3(2, std::make_pair(3, 1), 75., 30., 3);
      Street s4(3, std::make_pair(1, 4), 55., 30., 1);
      RoadNetwork graph{};
      graph.addStreets(s1, s2, s3, s4);
      WHEN("We adjust node capacities") {
        graph.adjustNodeCapacities();
        THEN("The node capacities are correct") {
          CHECK_EQ(graph.node(0).capacity(), 1);
          CHECK_EQ(graph.node(1).capacity(), 4);
          CHECK_EQ(graph.node(2).capacity(), 2);
          CHECK_EQ(graph.node(3).capacity(), 3);
          CHECK_EQ(graph.node(4).capacity(), 1);
        }
        THEN("The transport capacities are correct") {
          CHECK_EQ(graph.node(0).transportCapacity(), 1);
          CHECK_EQ(graph.node(1).transportCapacity(), 3);
          CHECK_EQ(graph.node(2).transportCapacity(), 1);
          CHECK_EQ(graph.node(3).transportCapacity(), 3);
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
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setRoadType(RoadType::HIGHWAY);

      // Edge 2: 2 -> 1 (HIGHWAY)
      Street s2(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setRoadType(RoadType::HIGHWAY);

      // Edge 3: 3 -> 1 (SECONDARY)
      Street s3(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setRoadType(RoadType::SECONDARY);

      graph.addStreets(s1, s2, s3);

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
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setRoadType(RoadType::HIGHWAY);
      graph.addStreets(s1);

      WHEN("We auto assign road priorities") {
        graph.autoAssignRoadPriorities();
        THEN("No priorities are assigned") { CHECK_FALSE(graph.edge(1).hasPriority()); }
      }
    }

    GIVEN("A node with 3 incoming edges of the same road type") {
      RoadNetwork graph{};
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setRoadType(RoadType::PRIMARY);
      Street s2(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setRoadType(RoadType::PRIMARY);
      Street s3(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setRoadType(RoadType::PRIMARY);
      graph.addStreets(s1, s2, s3);

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
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      Street s2(2, std::make_pair(2, 1), 100., 30., 1);
      Street s3(3, std::make_pair(3, 1), 100., 30., 1);
      graph.addStreets(s1, s2, s3);

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
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setRoadType(RoadType::HIGHWAY);
      Street s2(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setRoadType(RoadType::HIGHWAY);
      Street s3(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setRoadType(RoadType::HIGHWAY);
      // 2 SECONDARY roads (will match count == 2)
      Street s4(4, std::make_pair(4, 1), 100., 30., 1);
      s4.setRoadType(RoadType::SECONDARY);
      Street s5(5, std::make_pair(5, 1), 100., 30., 1);
      s5.setRoadType(RoadType::SECONDARY);
      graph.addStreets(s1, s2, s3, s4, s5);

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
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setRoadType(RoadType::PRIMARY);
      Street s2(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setRoadType(RoadType::PRIMARY);
      Street s3(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setRoadType(RoadType::TERTIARY);
      graph.addStreets(s1, s2, s3);

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
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setRoadType(RoadType::HIGHWAY);
      Street s2(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setRoadType(RoadType::PRIMARY);
      Street s3(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setRoadType(RoadType::RESIDENTIAL);
      Street s4(4, std::make_pair(4, 1), 100., 30., 1);
      s4.setRoadType(RoadType::RESIDENTIAL);
      graph.addStreets(s1, s2, s3, s4);

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
      Street s1(1, std::make_pair(0, 1), 100., 30., 1);
      s1.setRoadType(RoadType::HIGHWAY);
      Street s2(2, std::make_pair(2, 1), 100., 30., 1);
      s2.setRoadType(RoadType::HIGHWAY);
      Street s3(3, std::make_pair(3, 1), 100., 30., 1);
      s3.setRoadType(RoadType::SECONDARY);
      // Node 5: 2 TERTIARY + 1 RESIDENTIAL
      Street s4(4, std::make_pair(1, 5), 100., 30., 1);
      s4.setRoadType(RoadType::TERTIARY);
      Street s5(5, std::make_pair(4, 5), 100., 30., 1);
      s5.setRoadType(RoadType::TERTIARY);
      Street s6(6, std::make_pair(6, 5), 100., 30., 1);
      s6.setRoadType(RoadType::RESIDENTIAL);
      graph.addStreets(s1, s2, s3, s4, s5, s6);

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

    Street s{42,
             std::make_pair(0, 1),
             123.0,
             36.0 / 3.6,
             2,
             "Exported Street",
             dsf::geometry::PolyLine{
                 {dsf::geometry::Point(8.0, 45.0), dsf::geometry::Point(8.1, 45.1)}}};
    s.setRoadType(RoadType::PRIMARY);
    s.setPriority();
    graph.addStreet(std::move(s));
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
    CHECK(edgesCsv.find("id,source,target,length,maxspeed,nlanes,type,capacity,status,"
                        "name,priority,coilcode,geometry") != std::string::npos);
    CHECK(edgesCsv.find("Exported Street") != std::string::npos);
    CHECK(edgesCsv.find("primary") != std::string::npos);
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
    CHECK(nodesCsv.find("POINT (") != std::string::npos);

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

TEST_CASE("Dijkstra") {
  SUBCASE("Case 1") {
    Street s1{0, std::make_pair(0, 1), 3.};
    Street s2{1, std::make_pair(1, 2), 2.};
    Street s3{2, std::make_pair(2, 3), 4.};
    Street s4{3, std::make_pair(3, 0), 5.};
    Street s5{4, std::make_pair(0, 2), 6.};
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(s1, s2, s3, s4, s5);

    auto const pathMap = graph.allPathsTo(2);
    CHECK_FALSE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 0);
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);
  }

  SUBCASE("Case 2") {
    Street s1(0, std::make_pair(0, 1), 1.);
    Street s2(1, std::make_pair(1, 2), 1.);
    Street s3(2, std::make_pair(0, 2), 6.);
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(s1, s2, s3);

    auto const pathMap = graph.allPathsTo(2);
    CHECK_FALSE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);
  }

  SUBCASE("Case 3") {
    Street s1(0, std::make_pair(0, 1), 5.);
    Street s2(1, std::make_pair(1, 2), 4.);
    Street s3(2, std::make_pair(0, 2), 6.);
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(s1, s2, s3);

    auto const pathMap = graph.allPathsTo(2);
    CHECK_FALSE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 2);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);
  }

  SUBCASE("Case 4") {
    Street s1(0, std::make_pair(0, 1), 3.);
    Street s2(1, std::make_pair(0, 2), 1.);
    Street s3(2, std::make_pair(1, 2), 7.);
    Street s4(3, std::make_pair(2, 3), 2.);
    Street s5(4, std::make_pair(1, 4), 1.);
    Street s6(5, std::make_pair(1, 3), 5.);
    Street s7(6, std::make_pair(3, 4), 7.);
    Street s8(7, std::make_pair(1, 0), 3.);
    Street s9(8, std::make_pair(2, 0), 1.);
    Street s10(9, std::make_pair(2, 1), 7.);
    Street s11(10, std::make_pair(3, 2), 2.);
    Street s12(11, std::make_pair(4, 1), 1.);
    Street s13(12, std::make_pair(3, 1), 5.);
    Street s14(13, std::make_pair(4, 3), 7.);
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14);

    auto const pathMap = graph.allPathsTo(4);
    CHECK_FALSE(pathMap.contains(4));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 4);
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 0);
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 1);
  }

  SUBCASE("Case 5") {
    Street s1(0, std::make_pair(0, 1), 2.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(1, 3), 5.);
    Street s4(3, std::make_pair(2, 3), 8.);
    Street s5(4, std::make_pair(3, 5), 15.);
    Street s6(5, std::make_pair(3, 4), 10.);
    Street s7(6, std::make_pair(4, 5), 6.);
    Street s8(7, std::make_pair(4, 6), 2.);
    Street s9(8, std::make_pair(5, 6), 6.);
    Street s10(9, std::make_pair(1, 0), 2.);
    Street s11(10, std::make_pair(2, 0), 6.);
    Street s12(11, std::make_pair(3, 1), 5.);
    Street s13(12, std::make_pair(3, 2), 8.);
    Street s14(13, std::make_pair(5, 3), 15.);
    Street s15(14, std::make_pair(4, 3), 10.);
    Street s16(15, std::make_pair(5, 4), 6.);
    Street s17(16, std::make_pair(6, 4), 2.);
    Street s18(17, std::make_pair(6, 5), 6.);
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(
        s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16, s17, s18);

    auto const pathMap = graph.allPathsTo(6);
    CHECK_FALSE(pathMap.contains(6));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 3);
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 4);
    CHECK_EQ(pathMap.at(4).size(), 1);
    CHECK_EQ(pathMap.at(4)[0], 6);
    CHECK_EQ(pathMap.at(5).size(), 1);
    CHECK_EQ(pathMap.at(5)[0], 6);
  }

  SUBCASE("Case 6") {
    Street s1(0, std::make_pair(0, 1), 7.);
    Street s2(1, std::make_pair(0, 2), 9.);
    Street s3(2, std::make_pair(0, 5), 14.);
    Street s4(3, std::make_pair(1, 3), 15.);
    Street s5(4, std::make_pair(1, 2), 10.);
    Street s6(5, std::make_pair(2, 3), 11.);
    Street s7(6, std::make_pair(2, 5), 2.);
    Street s8(7, std::make_pair(3, 4), 6.);
    Street s9(8, std::make_pair(5, 4), 9.);
    Street s10(9, std::make_pair(1, 0), 7.);
    Street s11(10, std::make_pair(2, 0), 9.);
    Street s12(11, std::make_pair(5, 0), 14.);
    Street s13(12, std::make_pair(3, 1), 15.);
    Street s14(13, std::make_pair(2, 1), 10.);
    Street s15(14, std::make_pair(3, 2), 11.);
    Street s16(15, std::make_pair(5, 2), 2.);
    Street s17(16, std::make_pair(4, 3), 6.);
    Street s18(17, std::make_pair(4, 5), 9.);
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(
        s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16, s17, s18);

    auto const pathMap = graph.allPathsTo(4);
    CHECK_FALSE(pathMap.contains(4));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 2);
    CHECK_EQ(pathMap.at(1).size(), 2);
    CHECK_EQ(pathMap.at(1)[0], 3);
    CHECK_EQ(pathMap.at(1)[1], 2);
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 5);
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 4);
    CHECK_EQ(pathMap.at(5).size(), 1);
    CHECK_EQ(pathMap.at(5)[0], 4);
  }

  SUBCASE("Case 7") {
    Street s1(0, std::make_pair(1, 2), 1.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(2, 0), 6.);
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(s1, s2, s3);

    auto const pathMap = graph.allPathsTo(1);
    CHECK(pathMap.empty());
  }

  SUBCASE("Case 8") {
    Street s1(0, std::make_pair(1, 2), 1.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(2, 0), 6.);
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addStreets(s1, s2, s3);

    CHECK_THROWS_AS(graph.allPathsTo(3), std::out_of_range);
  }

  SUBCASE("Case 9 - Equal Lengths") {
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
    // check correct import
    CHECK_EQ(graph.nNodes(), 120);
    CHECK_EQ(graph.nEdges(), 436);

    auto const path = graph.allPathsTo(118);
    CHECK_EQ(path.size(), 119);
    CHECK_FALSE(path.contains(118));
  }
}

TEST_CASE("ShortestPath") {
  SUBCASE("Simple Path") {
    // Create a simple network: 0 -> 1 -> 2
    //                          |         ^
    //                          +-> 3 ----+
    // Path 0->1->2 has length 200, path 0->3->2 has length 300
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
    graph.addNode(3, dsf::geometry::Point(1.0, 1.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    Street s03(2, std::make_pair(0, 3), 150.0);
    Street s32(3, std::make_pair(3, 2), 150.0);
    graph.addStreets(s01, s12, s03, s32);

    auto pathMap = graph.shortestPath(0, 2);

    // Verify the shortest path is 0 -> 1 -> 2
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);

    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);

    // Target node should not be in the map
    CHECK_FALSE(pathMap.contains(2));

    // Node 3 is not on the shortest path
    CHECK_FALSE(pathMap.contains(3));
  }

  SUBCASE("Same Source and Target") {
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    Street s01(0, std::make_pair(0, 1), 100.0);
    graph.addStreet(std::move(s01));

    auto pathMap = graph.shortestPath(0, 0);

    // When source equals target, should return empty map (no hops needed)
    CHECK(pathMap.empty());
  }

  SUBCASE("No Path Exists") {
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s23(1, std::make_pair(2, 3), 100.0);
    graph.addStreets(s01, s23);

    auto pathMap = graph.shortestPath(0, 3);

    // No path exists, should return empty map
    CHECK(pathMap.empty());
  }

  SUBCASE("Invalid Source Node") {
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    Street s01(0, std::make_pair(0, 1), 100.0);
    graph.addStreet(std::move(s01));

    CHECK_THROWS_AS(graph.shortestPath(99, 1), std::out_of_range);
  }

  SUBCASE("Invalid Target Node") {
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    Street s01(0, std::make_pair(0, 1), 100.0);
    graph.addStreet(std::move(s01));

    CHECK_THROWS_AS(graph.shortestPath(0, 99), std::out_of_range);
  }

  SUBCASE("Multiple Paths - Choose Shortest") {
    // Diamond-shaped network
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 100.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s23(3, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s02, s13, s23);

    auto pathMap = graph.shortestPath(0, 3);

    // Verify the shortest path is 0 -> 1 -> 3 (length 20)
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);

    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);

    // Node 2 is not on the shortest path (0->2->3 has length 110)
    CHECK_FALSE(pathMap.contains(2));
    CHECK_FALSE(pathMap.contains(3));
  }

  SUBCASE("Complex Network with Coordinates") {
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(10.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(0.0, 10.0));
    graph.addNode(3, dsf::geometry::Point(10.0, 10.0));

    Street s01(0, std::make_pair(0, 1), 50.0);
    Street s02(1, std::make_pair(0, 2), 50.0);
    Street s13(2, std::make_pair(1, 3), 50.0);
    Street s23(3, std::make_pair(2, 3), 50.0);
    graph.addStreets(s01, s02, s13, s23);

    auto pathMap = graph.shortestPath(0, 3);

    // Both paths have same length (100), so node 0 should have two next hops
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 2);
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 1) !=
          pathMap.at(0).end());
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 2) !=
          pathMap.at(0).end());

    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);

    REQUIRE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 3);
  }

  SUBCASE("Large Network") {
    RoadNetwork graph{};
    graph.setEdgeWeight("length");
    graph.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
    graph.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());

    CHECK_EQ(graph.nNodes(), 120);
    CHECK_EQ(graph.nEdges(), 436);

    auto pathMap = graph.shortestPath(0, 119);

    // Verify that a path exists
    CHECK_FALSE(pathMap.empty());

    // Source should have next hops
    REQUIRE(pathMap.contains(0));
    CHECK_GT(pathMap.at(0).size(), 0);

    // Target should not be in the map
    CHECK_FALSE(pathMap.contains(119));

    // Verify connectivity: for each node in pathMap, verify edges exist to next hops
    for (auto const& [nodeId, nextHops] : pathMap) {
      for (auto const& nextHop : nextHops) {
        CHECK_NOTHROW(graph.edge(nodeId, nextHop));
      }
    }
  }

  SUBCASE("Equivalent Paths with Threshold") {
    // Create a network with two equal-length paths
    // 0 -> 1 -> 3 (length 20)
    // 0 -> 2 -> 3 (length 20)
    RoadNetwork graph{};
    graph.setEdgeWeight("length", 0.01);
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 10.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s23(3, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s02, s13, s23);

    auto pathMap = graph.shortestPath(0, 3);

    // Check that node 0 has multiple next hops (both 1 and 2)
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 2);
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 1) !=
          pathMap.at(0).end());
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 2) !=
          pathMap.at(0).end());

    // Both intermediate nodes should lead to target
    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);

    REQUIRE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 3);

    // Test explode function to get all path combinations
    auto allPaths = pathMap.explode(0, 3);
    CHECK_EQ(allPaths.size(), 2);

    // Verify both paths are present
    bool foundPath1 = false;
    bool foundPath2 = false;
    for (auto const& path : allPaths) {
      if (path.size() == 3 && path[0] == 0 && path[1] == 1 && path[2] == 3) {
        foundPath1 = true;
      }
      if (path.size() == 3 && path[0] == 0 && path[1] == 2 && path[2] == 3) {
        foundPath2 = true;
      }
    }
    CHECK(foundPath1);
    CHECK(foundPath2);
  }

  SUBCASE("PathCollection::explode - Complex Multiple Paths") {
    /* Create a more complex network with multiple equivalent paths
          1 -> 3
        /      \
        0        5
        \      /
          2 -> 4
      All edges have equal length, so there are multiple shortest paths */
    RoadNetwork graph{};
    graph.setEdgeWeight("length", 0.01);
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 10.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s24(3, std::make_pair(2, 4), 10.0);
    Street s35(4, std::make_pair(3, 5), 10.0);
    Street s45(5, std::make_pair(4, 5), 10.0);
    graph.addStreets(s01, s02, s13, s24, s35, s45);

    auto pathMap = graph.shortestPath(0, 5);

    // Test explode function
    auto allPaths = pathMap.explode(0, 5);
    CHECK_EQ(allPaths.size(), 2);

    // Verify the two paths are: 0 -> 1 -> 3 -> 5 and 0 -> 2 -> 4 -> 5
    bool foundPath1 = false;
    bool foundPath2 = false;
    for (auto const& path : allPaths) {
      CHECK_EQ(path.size(), 4);
      CHECK_EQ(path[0], 0);
      CHECK_EQ(path[3], 5);

      if (path[1] == 1 && path[2] == 3) {
        foundPath1 = true;
      }
      if (path[1] == 2 && path[2] == 4) {
        foundPath2 = true;
      }
    }
    CHECK(foundPath1);
    CHECK(foundPath2);
  }

  SUBCASE("Threshold - deep alternative paths") {
    Street s1(0, std::make_pair(0, 1), 3.);
    Street s2(1, std::make_pair(1, 2), 3.);
    Street s3(2, std::make_pair(2, 4), 3.);
    Street s4(3, std::make_pair(0, 3), 5.);
    Street s5(4, std::make_pair(3, 4), 5.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3, s4, s5);
    graph.setEdgeWeight("length", 0.7);

    auto const pathMap = graph.allPathsTo(4);
    CHECK_EQ(pathMap.at(0).size(), 2);
  }

  SUBCASE("Threshold - full path budget without accumulation") {
    // Best path: 0 -> 1 -> 2 -> 5 = 10.0
    // Alternative: 0 -> 1 -> 3 -> 5 = 10.89 (within 10%)
    // Over-budget path: 0 -> 1 -> 3 -> 4 -> 5 = 11.39 (must be excluded)
    Street s01(0, std::make_pair(0, 1), 1.0);
    Street s12(1, std::make_pair(1, 2), 4.0);
    Street s25(2, std::make_pair(2, 5), 5.0);
    Street s13(3, std::make_pair(1, 3), 4.89);
    Street s35(4, std::make_pair(3, 5), 5.0);
    Street s34(5, std::make_pair(3, 4), 0.5);
    Street s45(6, std::make_pair(4, 5), 5.0);

    RoadNetwork graph{};
    graph.addStreets(s01, s12, s25, s13, s35, s34, s45);
    graph.setEdgeWeight("length", 0.1);

    auto const pathMap = graph.shortestPath(0, 5);

    auto const allPaths = pathMap.explode(0, 5);
    CHECK_EQ(allPaths.size(), 2);

    bool foundBestPath{false};
    bool foundValidAlternative{false};
    bool foundOverBudgetPath{false};

    for (auto const& path : allPaths) {
      double totalWeight{0.0};
      for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        totalWeight += graph.edge(path[i], path[i + 1]).length();
      }
      CHECK_LE(totalWeight, 11.0 + 1e-9);

      if (path.size() == 4 && path[0] == 0 && path[1] == 1 && path[2] == 2 &&
          path[3] == 5) {
        foundBestPath = true;
      }
      if (path.size() == 4 && path[0] == 0 && path[1] == 1 && path[2] == 3 &&
          path[3] == 5) {
        foundValidAlternative = true;
      }
      if (path.size() == 5 && path[0] == 0 && path[1] == 1 && path[2] == 3 &&
          path[3] == 4 && path[4] == 5) {
        foundOverBudgetPath = true;
      }
    }

    CHECK(foundBestPath);
    CHECK(foundValidAlternative);
    CHECK_FALSE(foundOverBudgetPath);
  }

  SUBCASE("PathCollection::explode - Many Equivalent Paths") {
    // Create a grid-like network with many equivalent paths
    //   0 -> 1 -> 2
    //   |    |    |
    //   v    v    v
    //   3 -> 4 -> 5
    // All horizontal and vertical edges have equal length
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s12(1, std::make_pair(1, 2), 10.0);
    Street s03(2, std::make_pair(0, 3), 10.0);
    Street s14(3, std::make_pair(1, 4), 10.0);
    Street s25(4, std::make_pair(2, 5), 10.0);
    Street s34(5, std::make_pair(3, 4), 10.0);
    Street s45(6, std::make_pair(4, 5), 10.0);
    graph.addStreets(s01, s12, s03, s14, s25, s34, s45);
    graph.setEdgeWeight("length", 0.01);

    auto pathMap = graph.shortestPath(0, 5);

    // Test explode function
    auto allPaths = pathMap.explode(0, 5);

    // All shortest paths from 0 to 5 have length 40 (4 edges).
    // There are 3 such shortest paths: 0->1->2->5, 0->1->4->5, and 0->3->4->5.
    CHECK_GT(allPaths.size(), 0);

    // Verify all paths start at 0 and end at 5
    for (auto const& path : allPaths) {
      CHECK_EQ(path[0], 0);
      CHECK_EQ(path[path.size() - 1], 5);
    }
  }

  SUBCASE("PathCollection::explode - Single Path") {
    // Create a simple linear network
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s12(1, std::make_pair(1, 2), 10.0);
    Street s23(2, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s12, s23);
    graph.setEdgeWeight("length");

    auto pathMap = graph.shortestPath(0, 3);

    // Test explode function - should return only one path
    auto allPaths = pathMap.explode(0, 3);
    CHECK_EQ(allPaths.size(), 1);

    auto const& path = allPaths.front();
    CHECK_EQ(path.size(), 4);
    CHECK_EQ(path[0], 0);
    CHECK_EQ(path[1], 1);
    CHECK_EQ(path[2], 2);
    CHECK_EQ(path[3], 3);
  }

  SUBCASE("PathCollection::explode - No Path") {
    // Create a PathCollection with no path to target
    PathCollection pathMap;
    pathMap[0] = {1, 2};
    pathMap[1] = {3};
    pathMap[2] = {4};
    // No path from 0 to 5

    auto allPaths = pathMap.explode(0, 5);
    CHECK_EQ(allPaths.size(), 0);
  }

  SUBCASE("PathCollection::explode - Same Source and Target") {
    // Test when source equals target
    PathCollection pathMap;
    pathMap[0] = {1, 2};
    pathMap[1] = {3};

    auto allPaths = pathMap.explode(0, 0);
    CHECK_EQ(allPaths.size(), 1);

    auto const& path = allPaths.front();
    CHECK_EQ(path.size(), 1);
    CHECK_EQ(path[0], 0);
  }

  SUBCASE("DefaultWeight_UsesStreetEstimatedTravelTime") {
    Street::setEstimatedTravelTimeFunction(
        [](Street const& street) { return street.length() / street.maxSpeed(); });

    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 1.0, 1.0);
    Street s12(1, std::make_pair(1, 2), 1.0, 1.0);
    Street s02(2, std::make_pair(0, 2), 1.5, 1.0);
    graph.addStreets(s01, s12, s02);

    auto defaultPath = graph.shortestPath(0, 2);
    REQUIRE(defaultPath.contains(0));
    REQUIRE_EQ(defaultPath.at(0).size(), 1);
    CHECK_EQ(defaultPath.at(0).at(0), 2);

    Street::setEstimatedTravelTimeFunction([](Street const& street) {
      auto const t = street.length() / street.maxSpeed();
      return t * t;
    });

    auto customPath = graph.shortestPath(0, 2);
    REQUIRE(customPath.contains(0));
    REQUIRE_EQ(customPath.at(0).size(), 1);
    CHECK_EQ(customPath.at(0).at(0), 1);

    Street::setEstimatedTravelTimeFunction([](Street const& resetStreet) {
      return resetStreet.length() / resetStreet.maxSpeed();
    });
  }

  SUBCASE("TravelTimeWeight_UsesStreetEstimatedTravelTime") {
    Street::setEstimatedTravelTimeFunction(
        [](Street const& street) { return street.length() / street.maxSpeed(); });

    RoadNetwork graph{};
    graph.setEdgeWeight("traveltime");
    Street s01(0, std::make_pair(0, 1), 1.0, 1.0);
    Street s12(1, std::make_pair(1, 2), 1.0, 1.0);
    Street s02(2, std::make_pair(0, 2), 1.5, 1.0);
    graph.addStreets(s01, s12, s02);

    auto defaultPath = graph.shortestPath(0, 2);
    REQUIRE(defaultPath.contains(0));
    REQUIRE_EQ(defaultPath.at(0).size(), 1);
    CHECK_EQ(defaultPath.at(0).at(0), 2);

    Street::setEstimatedTravelTimeFunction([](Street const& street) {
      auto const t = street.length() / street.maxSpeed();
      return t * t;
    });

    auto customPath = graph.shortestPath(0, 2);
    REQUIRE(customPath.contains(0));
    REQUIRE_EQ(customPath.at(0).size(), 1);
    CHECK_EQ(customPath.at(0).at(0), 1);

    Street::setEstimatedTravelTimeFunction([](Street const& resetStreet) {
      return resetStreet.length() / resetStreet.maxSpeed();
    });
  }
}

TEST_CASE("RoadStatus") {
  SUBCASE("setStreetStatusById") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    graph.addStreets(s01, s12);

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
  }

  SUBCASE("setStreetStatusByName") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
    graph.addNode(3, dsf::geometry::Point(3.0, 0.0));

    Street s01(0, std::make_pair(0, 1), 100.0, 13.8888, 1, "Main Street");
    Street s12(1, std::make_pair(1, 2), 100.0, 13.8888, 1, "Main Street");
    Street s23(2, std::make_pair(2, 3), 100.0, 13.8888, 1, "Side Road");
    graph.addStreets(s01, s12, s23);

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

    Street s01(0, std::make_pair(0, 1), 100.0, 13.8888, 1, "Via Roma Nord");
    Street s12(1, std::make_pair(1, 2), 100.0, 13.8888, 1, "Via Roma Sud");
    graph.addStreets(s01, s12);

    // Close all streets containing "Roma" in the name
    graph.setStreetStatusByName("Roma", RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(0).roadStatus(), RoadStatus::CLOSED);
    CHECK_EQ(graph.edge(1).roadStatus(), RoadStatus::CLOSED);
  }
}

TEST_CASE("ShortestPath with closed roads") {
  SUBCASE("Closed road forces alternative path") {
    // Create a network: 0 -> 1 -> 2
    //                   |         ^
    //                   +-> 3 ----+
    // Path 0->1->2 has length 200, path 0->3->2 has length 300
    // If we close 0->1, shortest path should go through 3
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
    graph.addNode(3, dsf::geometry::Point(1.0, 1.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    Street s03(2, std::make_pair(0, 3), 150.0);
    Street s32(3, std::make_pair(3, 2), 150.0);
    graph.addStreets(s01, s12, s03, s32);
    graph.setEdgeWeight("length");

    // Initially, shortest path is 0 -> 1 -> 2
    auto pathMap = graph.shortestPath(0, 2);
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0)[0], 1);

    // Close street 0->1
    graph.setStreetStatusById(0, RoadStatus::CLOSED);

    // Now shortest path should be 0 -> 3 -> 2
    pathMap = graph.shortestPath(0, 2);
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 3);

    REQUIRE(pathMap.contains(3));
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 2);
  }

  SUBCASE("No path when all routes closed") {
    // Create a linear network: 0 -> 1 -> 2
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    graph.addStreets(s01, s12);
    graph.setEdgeWeight("length");

    // Close the middle street
    graph.setStreetStatusById(1, RoadStatus::CLOSED);

    // No path should exist from 0 to 2
    auto pathMap = graph.shortestPath(0, 2);
    CHECK(pathMap.empty());
  }

  SUBCASE("Reopening road restores path") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    graph.addStreets(s01, s12);
    graph.setEdgeWeight("length", 0.);

    // Close and then reopen
    graph.setStreetStatusById(0, RoadStatus::CLOSED);
    auto pathMap = graph.shortestPath(0, 2);
    CHECK(pathMap.empty());

    graph.setStreetStatusById(0, RoadStatus::OPEN);
    pathMap = graph.shortestPath(0, 2);
    CHECK_FALSE(pathMap.empty());
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0)[0], 1);
  }
}

TEST_CASE("allPathsTo with closed roads") {
  SUBCASE("Closed road excluded from paths") {
    // Create a network: 0 -> 1 -> 2
    //                   |         ^
    //                   +-> 3 ----+
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
    graph.addNode(3, dsf::geometry::Point(1.0, 1.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    Street s03(2, std::make_pair(0, 3), 150.0);
    Street s32(3, std::make_pair(3, 2), 150.0);
    graph.addStreets(s01, s12, s03, s32);
    graph.setEdgeWeight("length");

    // Close street 0->1
    graph.setStreetStatusById(0, RoadStatus::CLOSED);

    // allPathsTo should not include node 1 as next hop from 0
    auto pathMap = graph.allPathsTo(2);

    REQUIRE(pathMap.contains(0));
    // Node 0 should only have node 3 as next hop (not 1, since 0->1 is closed)
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 3);

    // Node 1 should still have a path to 2 via 1->2 (which is still open)
    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);
  }

  SUBCASE("All paths blocked to target") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    graph.addStreets(s01, s12);
    graph.setEdgeWeight("length");

    // Close the only path to node 2
    graph.setStreetStatusById(1, RoadStatus::CLOSED);

    auto pathMap = graph.allPathsTo(2);

    // Node 0 should not have a path to 2
    CHECK_FALSE(pathMap.contains(0));
    // Node 1 should not have a path to 2 either
    CHECK_FALSE(pathMap.contains(1));
  }

  SUBCASE("Closing road by name affects allPathsTo") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
    graph.addNode(3, dsf::geometry::Point(1.0, 1.0));

    Street s01(0, std::make_pair(0, 1), 100.0, 13.8888, 1, "Main Road");
    Street s12(1, std::make_pair(1, 2), 100.0, 13.8888, 1, "Main Road");
    Street s03(2, std::make_pair(0, 3), 150.0, 13.8888, 1, "Side Road");
    Street s32(3, std::make_pair(3, 2), 150.0, 13.8888, 1, "Side Road");
    graph.addStreets(s01, s12, s03, s32);
    graph.setEdgeWeight("length");

    // Close all "Main Road" streets
    graph.setStreetStatusByName("Main Road", RoadStatus::CLOSED);

    auto pathMap = graph.allPathsTo(2);

    // From node 0, only path should be via node 3
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 3);

    // Node 1 should have no path to 2 (since 1->2 is closed)
    CHECK_FALSE(pathMap.contains(1));
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

      Street s01(10, std::make_pair(0, 1), 100.0, 20.0, 2);
      Street s12(11, std::make_pair(1, 2), 150.0, 25.0, 3);
      graph.addStreets(s01, s12);

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
          CHECK_EQ(pStreet01->exitQueues().size(), 4);
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

      Street s01(10, std::make_pair(0, 1), 100.0, 20.0, 2, "Main Street");
      Street s12(11, std::make_pair(1, 2), 150.0, 25.0, 3, "Main Street");
      Street s03(12, std::make_pair(0, 3), 120.0, 15.0, 1, "Side Road");
      Street s32(13, std::make_pair(3, 2), 130.0, 18.0, 2, "Side Road");
      graph.addStreets(s01, s12, s03, s32);

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

TEST_CASE("BetweennessCentrality") {
  Road::setMeanVehicleLength(5.);

  SUBCASE("Linear chain: 0 -> 1 -> 2 -> 3") {
    // In a linear chain, all shortest paths between non-adjacent nodes pass
    // through the intermediate nodes. Node 1 and 2 should have BC > 0,
    // while endpoints 0 and 3 should have BC = 0.
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s12(1, std::make_pair(1, 2), 10.0);
    Street s23(2, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s12, s23);
    graph.setEdgeWeight("uniform");
    graph.computeBetweennessCentralities();

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
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 10.0);
    Street s03(2, std::make_pair(0, 3), 10.0);
    graph.addStreets(s01, s02, s03);
    graph.setEdgeWeight("uniform");
    graph.computeBetweennessCentralities();

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
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 10.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s23(3, std::make_pair(2, 3), 30.0);
    graph.addStreets(s01, s02, s13, s23);
    graph.setEdgeWeight("length");
    graph.computeBetweennessCentralities();

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
    Street s01(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(std::move(s01));
    graph.setEdgeWeight("uniform");
    graph.computeBetweennessCentralities();

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
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s23(1, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s23);
    graph.setEdgeWeight("uniform");
    graph.computeBetweennessCentralities();

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
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s12(1, std::make_pair(1, 2), 10.0);
    Street s23(2, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s12, s23);
    graph.setEdgeWeight("uniform");
    graph.computeEdgeBetweennessCentralities();

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
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 10.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s23(3, std::make_pair(2, 3), 30.0);
    graph.addStreets(s01, s02, s13, s23);
    graph.setEdgeWeight("length");
    graph.computeEdgeBetweennessCentralities();

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
    Street s01(0, std::make_pair(0, 1), 10.0);
    graph.addStreet(std::move(s01));
    graph.setEdgeWeight("uniform");
    graph.computeEdgeBetweennessCentralities();

    // Only one path: 0->1, using edge 0 => EBC = 1
    REQUIRE(graph.edge(static_cast<Id>(0))
                .getAttribute<double>("betweennessCentrality")
                .has_value());
    CHECK_EQ(
        *graph.edge(static_cast<Id>(0)).getAttribute<double>("betweennessCentrality"),
        doctest::Approx(1.0));
  }
}
