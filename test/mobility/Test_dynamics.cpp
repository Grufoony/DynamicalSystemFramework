#include "dsf/mobility/FirstOrderDynamics.hpp"
#include "dsf/mobility/RoadNetwork.hpp"
#include "dsf/mobility/Itinerary.hpp"
#include "dsf/mobility/Street.hpp"
#include "dsf/mobility/Intersection.hpp"
#include "dsf/mobility/Agent.hpp"

#include <tbb/tbb.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf;
using namespace dsf::mobility;

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "data";

TEST_CASE("Measurement") {
  SUBCASE("STL vector") {
    std::vector<float> data(100);
    std::iota(data.begin(), data.end(), 0.f);

    Measurement<float> m(data);
    CHECK_EQ(m.mean, 49.5f);
    CHECK_EQ(m.std, doctest::Approx(28.8661f));
    CHECK_EQ(m.n, 100);
    CHECK(m.is_valid);
  }
  SUBCASE("STL array") {
    std::array<float, 100> data;
    std::iota(data.begin(), data.end(), 0.f);

    Measurement<float> m(data);
    CHECK_EQ(m.mean, 49.5f);
    CHECK_EQ(m.std, doctest::Approx(28.8661f));
    CHECK_EQ(m.n, 100);
    CHECK(m.is_valid);
  }
  SUBCASE("STL span") {
    auto p = std::make_unique_for_overwrite<float[]>(100);
    std::span<float> data(p.get(), 100);
    std::iota(data.begin(), data.end(), 0.f);

    Measurement<float> m(data);
    CHECK_EQ(m.mean, 49.5f);
    CHECK_EQ(m.std, doctest::Approx(28.8661f));
    CHECK_EQ(m.n, 100);
    CHECK(m.is_valid);
  }
}

TEST_CASE("FirstOrderDynamics") {
  // dsf::Logger::setLogLevel(dsf::log_level_t::DEBUG);
  auto defaultNetwork = RoadNetwork{};
  defaultNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
  defaultNetwork.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());
  defaultNetwork.setEdgeWeight("length");
  SUBCASE("Constructor") {
    GIVEN("A graph object") {
      WHEN("A dynamics object is created") {
        FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
        THEN("The node and the street sets are the same") {
          CHECK_EQ(dynamics.graph().nNodes(), 120);
          CHECK_EQ(dynamics.graph().nEdges(), 436);
        }
        THEN("The mean speed, density, flow and travel time are 0") {
          // CHECK_EQ(dynamics.agentMeanSpeed().mean, 0.);
          // CHECK_EQ(dynamics.agentMeanSpeed().std, 0.);
          CHECK_EQ(dynamics.streetMeanDensity().mean, 0.);
          CHECK_EQ(dynamics.streetMeanDensity().std, 0.);
          CHECK_EQ(dynamics.streetMeanFlow().mean, 0.);
          CHECK_EQ(dynamics.streetMeanFlow().std, 0.);
          CHECK_EQ(dynamics.meanTravelTime().mean, 0.);
          CHECK_EQ(dynamics.meanTravelTime().std, 0.);
        }
      }
      WHEN("We transform a node into a traffic light and create the dynamics") {
        auto& tl = defaultNetwork.makeTrafficLight(0, 2);
        FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
        THEN("The node is a traffic light") {
          CHECK(dynamics.graph().node(0).isTrafficLight());
          CHECK_EQ(tl.cycleTime(), 2);
        }
      }
      WHEN("We transform a node into a roundabout and create the dynamics") {
        defaultNetwork.makeRoundabout(0);
        FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
        THEN("The node is a roundabout") {
          CHECK(dynamics.graph().node(0).isRoundabout());
        }
      }
      WHEN("We put a coil on the street and create the dynamics") {
        defaultNetwork.addCoil(8);
        FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
        THEN("The street has a coil") { CHECK(dynamics.graph().edge(8).hasCoil()); }
      }
      WHEN("We call summary") {
        FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
        std::ostringstream oss;
        dynamics.summary(oss);
        std::string summaryStr = oss.str();
        THEN("The summary contains expected information") {
          CHECK(summaryStr.find("FirstOrderDynamics Summary") != std::string::npos);
          CHECK(summaryStr.find("RoadNetwork with 120 nodes and 436 edges") !=
                std::string::npos);
          CHECK(summaryStr.find("Number of inserted agents") != std::string::npos);
          CHECK(summaryStr.find("Number of added agents") != std::string::npos);
          CHECK(summaryStr.find("Number of killed agents") != std::string::npos);
          CHECK(summaryStr.find("Current number of agents: 0") != std::string::npos);
        }
      }
    }
  }
  SUBCASE("setConcurrency") {
    GIVEN("A dynamics object") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      WHEN("We set concurrency to 2") {
        dynamics.setConcurrency(2);
        THEN("The concurrency is 2") { CHECK_EQ(dynamics.concurrency(), 2UL); }
      }
      WHEN("We set concurrency to 0 (invalid)") {
        dynamics.setConcurrency(0);
        THEN("The concurrency falls back to max") {
          CHECK_EQ(dynamics.concurrency(),
                   static_cast<std::size_t>(tbb::info::default_concurrency()));
        }
      }
      WHEN("We set concurrency to SIZE_MAX (invalid)") {
        dynamics.setConcurrency(std::numeric_limits<std::size_t>::max());
        THEN("The concurrency falls back to max") {
          CHECK_EQ(dynamics.concurrency(),
                   static_cast<std::size_t>(tbb::info::default_concurrency()));
        }
      }
    }
  }
  SUBCASE("setDestinationNodes") {
    GIVEN("A dynamics object and a destination node") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      WHEN("We add a span of destination nodes") {
        std::array<dsf::Id, 3> nodes{0, 1, 2};
        dynamics.setDestinationNodes(nodes);
        THEN("The destination nodes are added") {
          const auto& itineraries = dynamics.itineraries();
          CHECK_EQ(itineraries.size(), nodes.size());
          for (uint16_t i{0}; i < nodes.size(); ++i) {
            CHECK_EQ(itineraries.at(i)->destination(), nodes.at(i));
          }
        }
      }
    }
  }
  SUBCASE("addAgent") {
    GIVEN("A dynamics object, a source node and a destination node") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      dynamics.addItinerary(2, 2);
      WHEN("We add the agent") {
        dynamics.addAgent(dynamics.itineraries().at(2), 0);
        THEN("The agent is added") {
          CHECK_EQ(dynamics.nAgents(), 1);
          const auto& agent = dynamics.agents().at(0);
          // CHECK_EQ(agent->id(), 0);
          CHECK_EQ(agent->srcNodeId().value(), 0);
          CHECK_EQ(agent->itinerary()->id(), 2);
        }
      }
    }
  }
  SUBCASE("addAgentsUniformly") {
    GIVEN("A dynamics object and an itinerary") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      WHEN("We add a random agent") {
        dynamics.addItinerary(2, 2);
        dynamics.addAgentsUniformly(1);
        THEN(
            "The number of agents is 1 and the destination is the same as the "
            "itinerary") {
          CHECK_EQ(dynamics.nAgents(), 1);
        }
      }
    }
    GIVEN("A dynamics object and many itineraries") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(2, 2);
      dynamics.addItinerary(1, 1);
      WHEN("We add many agents") {
        // Logger::setLogLevel(dsf::log_level_t::DEBUG);
        dynamics.addAgentsUniformly(3);
        // Logger::setLogLevel(dsf::log_level_t::INFO);
        THEN(
            "The number of agents is 3, the destination and the street is the "
            "same as "
            "the itinerary") {
          CHECK_EQ(dynamics.nAgents(), 3);
#ifdef __APPLE__
          CHECK_EQ(dynamics.graph().edge(422).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(422).movingAgents().top()->itinerary()->id(), 2);
          CHECK_EQ(dynamics.graph().edge(199).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(199).movingAgents().top()->itinerary()->id(), 2);
          CHECK_EQ(dynamics.graph().edge(166).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(166).movingAgents().top()->itinerary()->id(), 1);
#else
          CHECK_EQ(dynamics.graph().edge(13).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(13).movingAgents().top()->itinerary()->id(), 2);
          CHECK_EQ(dynamics.graph().edge(370).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(370).movingAgents().top()->itinerary()->id(), 1);
          CHECK_EQ(dynamics.graph().edge(404).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(404).movingAgents().top()->itinerary()->id(), 2);
#endif
        }
      }
    }
  }
  SUBCASE("addAgentsRandomly") {
    GIVEN("A graph object") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      WHEN("We add one agent for existing itinerary") {
        std::unordered_map<dsf::Id, double> src{{0, 1.}};
        std::unordered_map<dsf::Id, double> dst{{2, 1.}};
        dynamics.setOriginNodes(src);
        dynamics.setDestinationNodes(dst);
        dynamics.updatePaths();
        dynamics.addAgents(1, AgentInsertionMethod::RANDOM_ODS);
        THEN("The agents are correctly set") {
          CHECK_EQ(dynamics.nAgents(), 1);
          CHECK_EQ(dynamics.agents().at(0)->itinerary()->destination(), 2);
          CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 0);
        }
      }
      WHEN("We add agents for existing itineraries") {
        std::unordered_map<dsf::Id, double> src{{1, 0.3}, {27, 0.3}, {118, 0.4}};
        std::unordered_map<dsf::Id, double> dst{{14, 0.3}, {102, 0.3}, {107, 0.4}};
        // std::vector<dsf::Id> destinations{14, 102, 107};
        dynamics.setOriginNodes(src);
        dynamics.setDestinationNodes(dst);
        dynamics.updatePaths();
        dynamics.addAgents(3, AgentInsertionMethod::RANDOM_ODS);
        THEN("The agents are correctly set") {
          CHECK_EQ(dynamics.nAgents(), 3);
          CHECK_EQ(dynamics.agents().at(0)->itinerary()->destination(), 107);
          CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 27);
          CHECK_EQ(dynamics.agents().at(1)->itinerary()->destination(), 14);
          CHECK_EQ(dynamics.agents().at(1)->srcNodeId().value(), 1);
          CHECK_EQ(dynamics.agents().at(2)->itinerary()->destination(), 14);
          CHECK_EQ(dynamics.agents().at(2)->srcNodeId().value(), 118);
        }
      }
    }
  }
  SUBCASE("addAgentsODs") {
    GIVEN("A graph object") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      WHEN("We add agents with a single OD pair") {
        std::vector<std::tuple<dsf::Id, dsf::Id, double>> ods{{0, 2, 1.}};
        dynamics.setODs(ods);
        dynamics.updatePaths();
        dynamics.addAgents(3, AgentInsertionMethod::ODS);
        THEN("All agents follow that single OD pair") {
          CHECK_EQ(dynamics.nAgents(), 3);
          for (size_t i = 0; i < 3; ++i) {
            CHECK_EQ(dynamics.agents().at(i)->itinerary()->destination(), 2);
            CHECK_EQ(dynamics.agents().at(i)->srcNodeId().value(), 0);
          }
        }
      }
      WHEN("We add agents with multiple OD pairs") {
        std::vector<std::tuple<dsf::Id, dsf::Id, double>> ods{{1, 14, 0.5},
                                                              {27, 107, 0.5}};
        dynamics.setODs(ods);
        dynamics.updatePaths();
        dynamics.addAgents(4, AgentInsertionMethod::ODS);
        THEN("The agents are assigned to the correct OD pairs") {
          CHECK_EQ(dynamics.nAgents(), 4);
          for (size_t i = 0; i < dynamics.agents().size(); ++i) {
            auto const& agent = dynamics.agents().at(i);
            auto dest = agent->itinerary()->destination();
            auto src = agent->srcNodeId().value();
            CHECK((dest == 14 || dest == 107));
            if (dest == 14) {
              CHECK_EQ(src, 1);
            } else {
              CHECK_EQ(src, 27);
            }
          }
        }
      }
      WHEN("We add agents with no OD pairs") {
        THEN("An exception is thrown") {
          CHECK_THROWS_AS(dynamics.addAgents(1, AgentInsertionMethod::ODS),
                          std::runtime_error);
        }
      }
    }
  }
  SUBCASE("addAgents") {
    GIVEN("A dynamics object and one itinerary") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      dynamics.addItinerary(2, 2);
      WHEN("We add an agent with itinerary 2") {
        dynamics.addAgent(dynamics.itineraries().at(2), 0);
        THEN(
            "The number of agents is 1 and the destination is the same as the "
            "itinerary") {
          CHECK_EQ(dynamics.nAgents(), 1);
          CHECK_EQ(dynamics.agents().at(0)->itinerary()->destination(), 2);
        }
      }
      WHEN("We add 69 agents") {
        dynamics.addAgents(69, AgentInsertionMethod::RANDOM);
        THEN("The number of agents is 69") { CHECK_EQ(dynamics.nAgents(), 69); }
      }
    }
  }
  SUBCASE("Update paths") {
    GIVEN("A dynamics object, many streets and an itinerary") {
      Street s1{0, std::make_pair(0, 1), 2.};
      Street s2{1, std::make_pair(1, 2), 5.};
      Street s3{2, std::make_pair(0, 2), 10.};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length", 0.0);
      graph2.addStreets(s1, s2, s3);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      WHEN("We add an itinerary and update the paths") {
        dynamics.addItinerary(std::make_shared<Itinerary>(0, 2));
        dynamics.updatePaths();
        THEN(
            "The number of itineraries is 1 and the path is updated and "
            "correctly "
            "formed") {
          CHECK_EQ(dynamics.itineraries().size(), 1);
          CHECK_EQ(dynamics.itineraries().at(0)->path().size(), 2);
          CHECK_EQ(dynamics.itineraries().at(0)->path().at(0).size(), 1);
          CHECK(dynamics.itineraries().at(0)->path().at(0)[0] == 1);
          CHECK_EQ(dynamics.itineraries().at(0)->path().at(1).size(), 1);
          CHECK(dynamics.itineraries().at(0)->path().at(1)[0] == 2);
          CHECK_FALSE(dynamics.itineraries().at(0)->path().at(0)[0] == 2);
        }
      }
    }
    GIVEN("A Dynamics object with a graph with 6 nodes and 7 streets") {
      RoadNetwork graph2{};
      graph2.addEdge(0, std::make_pair(0, 1), 10.);
      graph2.addEdge(1, std::make_pair(1, 2), 10.);
      graph2.addEdge(2, std::make_pair(2, 3), 10.);
      graph2.addEdge(3, std::make_pair(1, 4), 10.);
      graph2.addEdge(4, std::make_pair(4, 1), 10.);
      graph2.addEdge(5, std::make_pair(4, 5), 10.);
      graph2.addEdge(6, std::make_pair(5, 4), 10.);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      WHEN("We add an iitinerary to node 3 and update paths") {
        dynamics.addItinerary(3, 3);
        dynamics.updatePaths();
        THEN("The itinerary is correct, excluding paths passing in the same node twice") {
          auto const& path = dynamics.itineraries().at(3)->path();
          CHECK_EQ(path.size(), 5);
          CHECK_EQ(path.at(0).size(), 1);
          CHECK(path.at(0)[0] == 1);
          CHECK_EQ(path.at(1).size(), 1);
          CHECK(path.at(1)[0] == 2);
          CHECK_EQ(path.at(2).size(), 1);
          CHECK(path.at(2)[0] == 3);
          CHECK_FALSE(path.at(1)[0] == 4);
          CHECK_EQ(path.at(4).size(), 1);
          CHECK(path.at(4)[0] == 1);
          CHECK_FALSE(path.at(4)[0] == 5);
          CHECK_EQ(path.at(5).size(), 1);
          CHECK(path.at(5)[0] == 4);
        }
      }
    }
    GIVEN(
        "A dynamics objects, many streets and many itinearies with same "
        "destination") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 69};
      dynamics.addItinerary(0, 118);
      dynamics.addItinerary(1, 118);
      dynamics.addItinerary(2, 118);
      dynamics.addItinerary(3, 118);
      dynamics.updatePaths();
      WHEN("We update the paths") {
        dynamics.updatePaths();
        THEN("The paths are updated and correctly formed") {
          CHECK_EQ(dynamics.itineraries().size(), 4);
          for (auto const& it : dynamics.itineraries()) {
            auto const& path = it.second->path();
            for (uint16_t i{0}; i < path.size(); ++i) {
              if (i == it.second->destination()) {
                CHECK_FALSE(path.contains(i));
              } else {
                CHECK(path.contains(i));
              }
            }
          }
        }
      }
    }
    GIVEN("A dynamics objects, many streets and an itinerary with bifurcations") {
      Street s1{0, std::make_pair(0, 1), 5.};
      Street s2{1, std::make_pair(1, 2), 5.};
      Street s3{2, std::make_pair(0, 3), 5.};
      Street s4{3, std::make_pair(3, 2), 5.};
      RoadNetwork graph;
      graph.setEdgeWeight("length");
      graph.addStreets(s1, s2, s3, s4);
      FirstOrderDynamics dynamics{std::move(graph), false, 69};
      dynamics.addItinerary(0, 2);
      WHEN("We update the paths") {
        dynamics.updatePaths();
        THEN("The path is updated and correctly formed") {
          auto const& path{dynamics.itineraries().at(0)->path()};
          CHECK_EQ(dynamics.itineraries().size(), 1);
          CHECK_EQ(path.size(), 3);
          CHECK_EQ(path.at(0).size(), 2);
          CHECK_EQ(path.at(0)[0], 1);
          CHECK_EQ(path.at(0)[1], 3);
          CHECK_EQ(path.at(1).size(), 1);
          CHECK_EQ(path.at(1)[0], 2);
          CHECK_EQ(path.at(3).size(), 1);
          CHECK_EQ(path.at(3)[0], 2);
          CHECK_FALSE(path.contains(2));
        }
      }
    }
    GIVEN(
        "A dynamics objects, many streets and an itinerary with bifurcations (TIME "
        "WEIGHTED)") {
      Street s1{0, std::make_pair(0, 1), 5., 50.};
      Street s2{1, std::make_pair(1, 2), 7., 70.};
      Street s3{2, std::make_pair(0, 3), 9., 90.};
      Street s4{3, std::make_pair(3, 2), 10., 100.};
      RoadNetwork graph;
      graph.addStreets(s1, s2, s3, s4);
      FirstOrderDynamics dynamics{std::move(graph), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(0, 2);
      WHEN("We update the paths") {
        dynamics.updatePaths();
        THEN("The path is updated and correctly formed") {
          CHECK_EQ(dynamics.itineraries().size(), 1);
          CHECK_EQ(dynamics.itineraries().at(0)->path().size(), 3);
          CHECK(dynamics.itineraries().at(0)->path().at(0)[0] == 1);
          CHECK(dynamics.itineraries().at(0)->path().at(1)[0] == 2);
          CHECK(dynamics.itineraries().at(0)->path().at(0)[1] == 3);
          CHECK(dynamics.itineraries().at(0)->path().at(3)[0] == 2);
          CHECK_FALSE(dynamics.itineraries().at(0)->path().contains(2));
        }
      }
    }
    GIVEN("A dynamics object with a custom path weight callable") {
      Street s1{0, std::make_pair(0, 1), 5.};
      Street s2{1, std::make_pair(1, 2), 5.};
      Street s3{2, std::make_pair(0, 3), 5.};
      Street s4{3, std::make_pair(3, 2), 5.};
      s1.setAttribute("custom_cost", 100.0);
      s2.setAttribute("custom_cost", 100.0);
      s3.setAttribute("custom_cost", 1.0);
      s4.setAttribute("custom_cost", 1.0);

      RoadNetwork graph;
      graph.setEdgeWeight("custom_cost");
      graph.addStreets(s1, s2, s3, s4);
      FirstOrderDynamics dynamics{std::move(graph), false, 69};
      dynamics.addItinerary(0, 2);

      WHEN("We update the paths") {
        dynamics.updatePaths();
        THEN("The custom attribute-based weight drives the selected branch") {
          auto const& path{dynamics.itineraries().at(0)->path()};
          CHECK_EQ(path.at(0).size(), 1);
          CHECK_EQ(path.at(0)[0], 3);
          CHECK_EQ(path.at(3).size(), 1);
          CHECK_EQ(path.at(3)[0], 2);
          CHECK_FALSE(path.at(0)[0] == 1);
        }
      }
    }
    GIVEN("A disconnected graph") {
      Street s1{0, std::make_pair(0, 1), 10.};
      RoadNetwork graph;
      graph.setEdgeWeight("length");
      graph.addStreets(s1);
      FirstOrderDynamics dynamics{std::move(graph), false, 69};

      WHEN(
          "We add an impossible itinerary (to source node) and update paths with "
          "throw_on_empty=true") {
        dynamics.addItinerary(std::make_shared<Itinerary>(0, 0));
        THEN("It throws an exception") { CHECK_THROWS(dynamics.updatePaths(true)); }
      }

      WHEN(
          "We add an impossible itinerary (to source node) and update paths with "
          "throw_on_empty=false") {
        dynamics.addItinerary(std::make_shared<Itinerary>(0, 0));
        dynamics.updatePaths(false);
        THEN("The itinerary is removed") { CHECK(dynamics.itineraries().empty()); }
      }
    }
  }
  SUBCASE("Evolve") {
    GIVEN("A dynamics object with one non-random agent that reaches its destination") {
      Street s1{0, std::make_pair(0, 1), 13.8888888889};
      Street s2{1, std::make_pair(1, 0), 13.8888888889};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s1, s2);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(1, 1);
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(1), 0);

      WHEN("We evolve until the agent reaches the destination") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);

        THEN("The summary counts the agent as arrived but not killed") {
          std::ostringstream oss;
          dynamics.summary(oss);
          const auto summaryStr = oss.str();

          CHECK(summaryStr.find("Number of arrived agents: 1") != std::string::npos);
          CHECK(summaryStr.find("Number of killed agents: 0") != std::string::npos);
          CHECK(summaryStr.find("Current number of agents: 0") != std::string::npos);
        }
      }
    }
    GIVEN("A dynamics object and an itinerary") {
      Street s1{0, std::make_pair(0, 1), 2.};
      Street s2{1, std::make_pair(1, 2), 5.};
      Street s3{2, std::make_pair(0, 2), 10.};
      RoadNetwork graph;
      graph.setEdgeWeight("length");
      graph.addStreets(s1, s2, s3);
      FirstOrderDynamics dynamics{std::move(graph), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      WHEN("We add an agent randomly and evolve the dynamics") {
        dynamics.addAgent(dynamics.itineraries().at(2), 0);
        auto const& network{dynamics.graph()};
        dynamics.evolve(false);  // Agent goes into node 0
        dynamics.evolve(false);  // Agent goes from node 0 to street 0->1
        THEN("The agent evolves") {
          CHECK_EQ(network.edge(0).movingAgents().size(), 1);
          auto const& pAgent{network.edge(0).movingAgents().top()};
          CHECK(pAgent);
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), dynamics.time_step());
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 0);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
        }
        dynamics.evolve(false);  // Agent enqueues on street 0->1 and changes street
        THEN("The agent evolves again, changing street") {
          auto const& pAgent{network.edge(1).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), dynamics.time_step());
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 1);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
        }
        dynamics.evolve(
            false);  // Enqueues on street 5, goes into destination nodes and gets killed
        THEN("And again, reaching the destination") { CHECK_EQ(dynamics.nAgents(), 0); }
      }
    }
    GIVEN("A dynamics object, an itinerary and an agent") {
      Street s1{0, std::make_pair(0, 1), 13.8888888889};
      Street s2{1, std::make_pair(1, 0), 13.8888888889};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s1, s2);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(1, 1);
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(1), 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent evolves") {
          auto const& pAgent{dynamics.graph().edge(0).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), 2);
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 0);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
          CHECK_EQ(pAgent->distance(), 0.);  // Not updated yet
        }
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent reaches the destination") { CHECK_EQ(dynamics.nAgents(), 0); }
      }
    }
    GIVEN("A dynamics object, an itinerary and an agent") {
      Street s1{0, std::make_pair(0, 1), 13.8888888889};
      Street s2{1, std::make_pair(1, 0), 13.8888888889};
      s1.setTransportCapacity(0.3);
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s1, s2);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(std::make_shared<Itinerary>(0, 1));
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(0), 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent evolves") {
          auto const& pAgent{dynamics.graph().edge(0).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), 2);
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 0);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
          CHECK_EQ(pAgent->distance(), 0.);  // Not updated yet
        }
        auto i{0};
        while (dynamics.nAgents() > 0) {
          dynamics.evolve(false);
          ++i;
        }
        THEN("The agent reaches the destination") {
          CHECK(i > 0);
          CHECK_EQ(dynamics.nAgents(), 0);
        }
      }
    }
    GIVEN("A dynamics object, an itinerary and an agent") {
      Street s1{0, std::make_pair(0, 1), 13.8888888889};
      Street s2{1, std::make_pair(1, 0), 13.8888888889};
      RoadNetwork graph2;
      graph2.addStreets(s1, s2);
      graph2.setEdgeWeight("length");
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(1, 1);
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(1), 0);
      WHEN("We evolve the dynamics with reinsertion") {
        dynamics.evolve(true);
        dynamics.evolve(true);
        THEN("The agent has correct values") {
          auto const& pAgent{dynamics.graph().edge(0).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), 2);
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 0);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
          CHECK_EQ(pAgent->distance(), 0.);  // Not updated yet
        }
        dynamics.evolve(true);
        // dynamics.evolve(true);
        THEN("The agent is reinserted") {
          CHECK(dynamics.graph().node(0).density() > 0.);
          CHECK_EQ(dynamics.nAgents(), 1);
        }
      }
    }
    GIVEN("A simple network and an agent with forced itinerary") {
      // spdlog::set_level(spdlog::level::trace);
      Street s0_1{1, std::make_pair(0, 1), 30., 15.};
      Street s1_0{3, std::make_pair(1, 0), 30., 15.};
      Street s1_2{5, std::make_pair(1, 2), 30., 15.};
      Street s2_1{7, std::make_pair(2, 1), 30., 15.};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s0_1, s1_0, s1_2, s2_1);
      graph2.makeRoundabout(2);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setDestinationNodes({1, 2});
      dynamics.updatePaths();
      std::vector<std::shared_ptr<Itinerary>> trip{dynamics.itineraries().at(2),
                                                   dynamics.itineraries().at(1)};
      dynamics.addAgent(trip, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent goes first into node 2") {
          auto const& pAgent{dynamics.graph().edge(5).queue(0).front()};
          CHECK_EQ(pAgent->streetId().value(), 5);
          CHECK_EQ(pAgent->distance(), 60.);
        }
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent goes then to node 1") {
          auto const& pAgent{dynamics.graph().edge(7).queue(0).front()};
          CHECK_EQ(pAgent->streetId().value(), 7);
          CHECK_EQ(pAgent->distance(), 90.);
        }
        dynamics.evolve(false);
        THEN("The agent reaches the destination") {
          CHECK(dynamics.agents().empty());
          CHECK_EQ(dynamics.nAgents(), 0);
        }
      }
      // spdlog::set_level(spdlog::level::info);
    }
    GIVEN("A non-random agent at a dead-end node with no valid next street") {
      Street s0{0, std::make_pair(0, 1), 13.8888888889};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s0);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      // Manually construct an itinerary whose path leads through node 1, but
      // node 1 has no outgoing edges. m_nextStreetId will return nullopt and
      // the agent must be killed instead of throwing an exception.
      auto itinerary = std::make_shared<Itinerary>(0, 2);
      PathCollection path{{0, {1}}, {1, {2}}};
      itinerary->setPath(path);
      dynamics.addItinerary(itinerary);
      dynamics.addAgent(dynamics.itineraries().at(0), 0);
      WHEN("We evolve the dynamics until the agent reaches the dead-end") {
        dynamics.evolve(false);  // Agent enters intersection at node 0
        dynamics.evolve(false);  // Agent moves onto street 0
        THEN("The agent is alive and on street 0") {
          CHECK_EQ(dynamics.nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(0).nMovingAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(0).nAgents(), 1);
        }
        dynamics.evolve(false);  // Agent reaches dead-end at node 1: killed
        THEN("The agent is killed instead of throwing an exception") {
          CHECK_EQ(dynamics.nAgents(), 0);
          CHECK_EQ(dynamics.graph().edge(0).nMovingAgents(), 0);
          CHECK_EQ(dynamics.graph().edge(0).nAgents(), 0);
        }
      }
    }
  }
  SUBCASE("TrafficLights") {
    TrafficLight::setAllowFreeTurns(false);
    GIVEN(
        "A dynamics object, a network with traffic lights, an itinerary and "
        "an agent") {
      Street s1{1, std::make_pair(0, 1), 30., 15.};
      Street s2{7, std::make_pair(1, 2), 30., 15.};
      Street s3{16, std::make_pair(3, 1), 30., 15.};
      Street s4{9, std::make_pair(1, 4), 30., 15.};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addNode<TrafficLight>(1, 4);
      graph2.addStreets(s1, s2, s3, s4);
      auto& tl = graph2.node<TrafficLight>(1);
      tl.setCycle(1, dsf::Direction::ANY, {2, 0});
      tl.setCycle(16, dsf::Direction::ANY, {2, 2});
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(2), 0);
      auto const& network{dynamics.graph()};
      WHEN("We evolve the dynamics") {
        // Logger::setLogLevel(dsf::log_level_t::DEBUG);
        dynamics.evolve(false);
        THEN(
            "The agent is ready to go through the traffic light at time 3 but "
            "the "
            "traffic light is red"
            " until time 4, so the agent waits until time 4") {
          for (uint8_t i{0}; i < 5; ++i) {
            dynamics.evolve(false);
            if (i < 3) {
              CHECK_EQ(network.edge(1).nAgents(), 1);
            } else {
              CHECK_EQ(network.edge(7).nAgents(), 1);
            }
            if (i == 2) {
              CHECK_EQ(network.edge(1).queue(0).front()->distance(), 30.);
            }
          }
          CHECK_EQ(network.edge(7).queue(0).front()->distance(), 60.);
        }
        // Logger::setLogLevel(dsf::log_level_t::INFO);
      }
    }
    GIVEN(
        "A traffic light managing an intersection with 4 3-lanes streets and 4 1-lane "
        "streets") {
      // Streets
      Street s0_1{1, std::make_pair(0, 1), 30., 15., 3};
      Street s1_0{5, std::make_pair(1, 0), 30., 15., 3};
      Street s1_2{7, std::make_pair(1, 2), 30., 15., 3};
      Street s2_1{11, std::make_pair(2, 1), 30., 15., 3};

      Street s3_1{8, std::make_pair(3, 1), 30., 15.};
      Street s1_3{16, std::make_pair(1, 3), 30., 15.};
      Street s4_1{21, std::make_pair(4, 1), 30., 15.};
      Street s1_4{9, std::make_pair(1, 4), 30., 15.};

      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      {
        graph2.addNode<TrafficLight>(1, 6, dsf::geometry::Point(0., 0.));
        auto& tl = graph2.node<TrafficLight>(1);
        tl.setCycle(1, dsf::Direction::RIGHTANDSTRAIGHT, {2, 2});
        tl.setCycle(1, dsf::Direction::LEFT, {1, 4});
        tl.setCycle(11, dsf::Direction::ANY, {3, 2});
        tl.setComplementaryCycle(8, 11);
        tl.setComplementaryCycle(21, 11);
      }
      graph2.addNode<dsf::mobility::Intersection>(0, dsf::geometry::Point(-1, 0));
      graph2.addNode<dsf::mobility::Intersection>(2, dsf::geometry::Point(1, 0));
      graph2.addNode<dsf::mobility::Intersection>(3, dsf::geometry::Point(0, -1));
      graph2.addNode<dsf::mobility::Intersection>(4, dsf::geometry::Point(0, 1));
      graph2.addStreets(s0_1, s1_0, s1_2, s2_1, s3_1, s1_3, s4_1, s1_4);
      graph2.adjustNodeCapacities();

      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setDestinationNodes({0, 2, 3, 4});
      dynamics.updatePaths();

      WHEN("We add agents and make the system evolve") {
        // Logger::setLogLevel(dsf::log_level_t::DEBUG);
        dynamics.addAgent(dynamics.itineraries().at(2), 0);
        dynamics.addAgent(dynamics.itineraries().at(4), 0);
        THEN("The agents are not yet on the streets") {
          CHECK_FALSE(dynamics.agents().at(0)->streetId().has_value());
          CHECK_FALSE(dynamics.agents().at(1)->streetId().has_value());
        }
        dynamics.evolve(false);  // Counter 0
        dynamics.evolve(false);  // Counter 1
        THEN("The agents are correctly placed") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 2);
        }
        dynamics.evolve(false);  // Counter 2
        dynamics.evolve(false);  // Counter 3
        THEN("The agent 0 passes and agent 1 waits") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(7).nAgents(), 1);
        }
        dynamics.evolve(false);  // Counter 4
        THEN("The agent 1 passes") {
          CHECK_EQ(dynamics.graph().edge(7).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(9).nAgents(), 1);
        }
        // Logger::setLogLevel(dsf::log_level_t::INFO);
      }
    }
    GIVEN(
        "A traffic light managing an intersection with 4 3-lanes streets and 4 1-lane "
        "streets") {
      // Streets
      Street s0_1{1, std::make_pair(0, 1), 30., 15., 3};
      Street s1_0{5, std::make_pair(1, 0), 30., 15., 3};
      Street s1_2{7, std::make_pair(1, 2), 30., 15., 3};
      Street s2_1{11, std::make_pair(2, 1), 30., 15., 3};

      Street s3_1{8, std::make_pair(3, 1), 30., 15.};
      Street s1_3{16, std::make_pair(1, 3), 30., 15.};
      Street s4_1{21, std::make_pair(4, 1), 30., 15.};
      Street s1_4{9, std::make_pair(1, 4), 30., 15.};

      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      {
        graph2.addNode<TrafficLight>(1, 6, dsf::geometry::Point(0, 0));
        auto& tl = graph2.node<TrafficLight>(1);
        // Now testing red light = NO PHASE
        tl.setCycle(1, dsf::Direction::RIGHTANDSTRAIGHT, {2, 0});
        tl.setCycle(1, dsf::Direction::LEFT, {1, 2});
        tl.setCycle(11, dsf::Direction::ANY, {3, 0});
        tl.setComplementaryCycle(8, 11);
        tl.setComplementaryCycle(21, 11);
      }
      graph2.addNode<dsf::mobility::Intersection>(0, dsf::geometry::Point(-1, 0));
      graph2.addNode<dsf::mobility::Intersection>(2, dsf::geometry::Point(1, 0));
      graph2.addNode<dsf::mobility::Intersection>(3, dsf::geometry::Point(0, -1));
      graph2.addNode<dsf::mobility::Intersection>(4, dsf::geometry::Point(0, 1));
      graph2.addStreets(s0_1, s1_0, s1_2, s2_1, s3_1, s1_3, s4_1, s1_4);
      graph2.adjustNodeCapacities();
      graph2.autoMapStreetLanes();

      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setDestinationNodes({0, 2, 3, 4});
      dynamics.updatePaths();

      WHEN("We add agents and make the system evolve") {
        dynamics.addAgent(dynamics.itineraries().at(2), 0);
        dynamics.addAgent(dynamics.itineraries().at(4), 0);
        dynamics.evolve(false);  // Counter 0
        dynamics.evolve(false);  // Counter 1
        THEN("The agents are correctly placed") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 2);
        }
        dynamics.evolve(false);  // Counter 2
        dynamics.evolve(false);  // Counter 3
        THEN("The agents are still") {
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::ANY, true),
                   doctest::Approx(0.666667));
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::RIGHT), 0);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::STRAIGHT), 1);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::LEFT), 1);
        }
        dynamics.evolve(false);  // Counter 4
        dynamics.evolve(false);  // Counter 5
        dynamics.evolve(false);  // Counter 0
        THEN("The agent 0 passes and agent 1 waits") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(7).nAgents(), 1);
        }
        dynamics.evolve(false);  // Counter 1
        dynamics.evolve(false);  // Counter 2
        THEN("The agent 1 passes") { CHECK_EQ(dynamics.graph().edge(9).nAgents(), 1); }
      }
    }
    SUBCASE("Traffic Lights optimization algorithm") {
      GIVEN("A dynamics object with a traffic light intersection") {
        double length{90.}, max_speed{15.};
        Street s_01{1, std::make_pair(0, 1), length, max_speed};
        Street s_10{5, std::make_pair(1, 0), length, max_speed};
        Street s_12{7, std::make_pair(1, 2), length, max_speed};
        Street s_21{11, std::make_pair(2, 1), length, max_speed};
        Street s_13{8, std::make_pair(1, 3), length, max_speed};
        Street s_31{16, std::make_pair(3, 1), length, max_speed};
        Street s_14{9, std::make_pair(1, 4), length, max_speed};
        Street s_41{21, std::make_pair(4, 1), length, max_speed};
        RoadNetwork graph2;
        graph2.setEdgeWeight("length");
        graph2.addStreets(s_01, s_10, s_12, s_21, s_13, s_31, s_14, s_41);
        auto& tl = graph2.makeTrafficLight(1, 8, 3);
        tl.addStreetPriority(1);
        tl.addStreetPriority(11);
        tl.setCycle(1, dsf::Direction::ANY, {4, 0});
        tl.setCycle(11, dsf::Direction::ANY, {4, 0});
        tl.setComplementaryCycle(16, 11);
        tl.setComplementaryCycle(21, 11);
        FirstOrderDynamics dynamics{std::move(graph2), false, 69};
        dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
        dynamics.setDestinationNodes({0, 2, 3, 4});
        dynamics.updatePaths();
        WHEN("We evolve the dynamics and optimize traffic lights") {
          for (int i = 0; i < 7; ++i) {
            dynamics.addAgent(dynamics.itineraries().at(0), 2);
            dynamics.addAgent(dynamics.itineraries().at(2), 0);
          }
          dynamics.setDataUpdatePeriod(4);
          for (int i = 0; i < 9; ++i) {
            dynamics.evolve(false);
          }
          dynamics.optimizeTrafficLights(
              dsf::TrafficLightOptimization::SINGLE_TAIL, std::string(), 1);
          THEN("Green and red time are different") {
            CHECK(tl.meanGreenTime(true) > tl.meanGreenTime(false));
          }
        }
        WHEN(
            "We evolve the dynamics and optimize traffic lights with outgoing "
            "streets "
            "full") {
          for (int i = 0; i < 5; ++i) {
            dynamics.addAgent(dynamics.itineraries().at(0), 1);
            dynamics.addAgent(dynamics.itineraries().at(2), 1);
            dynamics.addAgent(dynamics.itineraries().at(3), 1);
            dynamics.addAgent(dynamics.itineraries().at(4), 1);
          }
          dynamics.setDataUpdatePeriod(8);
          for (int i = 0; i < 15; ++i) {
            dynamics.evolve(false);
          }
          dynamics.optimizeTrafficLights(
              dsf::TrafficLightOptimization::SINGLE_TAIL, std::string(), 1);
          THEN("Green and red time are equal") {
            CHECK_EQ(tl.meanGreenTime(true), tl.meanGreenTime(false));
          }
        }
      }
    }
  }
  SUBCASE("Roundabout") {
    GIVEN(
        "A dynamics object with four streets, one agent for each street, two "
        "itineraries "
        "and a roundabout") {
      Road::setMeanVehicleLength(10.);
      Street s1{1, std::make_pair(0, 1), 10., 10.};
      Street s2{7, std::make_pair(2, 1), 10., 10.};
      Street s3{3, std::make_pair(1, 0), 10., 10.};
      Street s4{5, std::make_pair(1, 2), 10., 10.};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s1, s2, s3, s4);
      auto& rb = graph2.makeRoundabout(1);
      graph2.adjustNodeCapacities();
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(0, 0);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(2), 0);
      dynamics.addAgent(dynamics.itineraries().at(0), 2);
      WHEN(
          "We evolve the dynamics adding an agent on the path of the agent "
          "with "
          "priority") {
        dynamics.evolve(false);  // Agents into sources
        // dynamics.evolve(false);  // Agents from sources to streets
        dynamics.addAgent(dynamics.itineraries().at(2), 1);
        dynamics.evolve(false);  // Agents into queues, other agent into roundabout
        dynamics.evolve(false);  // Agents from queues to roundabout
        auto const& network{dynamics.graph()};
        THEN("The agents are trapped into the roundabout") {
          CHECK_EQ(network.edge(1).nAgents(), 0);
          CHECK_EQ(network.edge(5).nAgents(), 1);
          CHECK_EQ(network.edge(7).nAgents(), 1);
          CHECK_EQ(rb.agents().size(), 1);
        }
        dynamics.evolve(false);
        THEN("The agent with priority leaves the roundabout") {
          CHECK_EQ(network.edge(3).nAgents(), 1);
          CHECK_EQ(network.edge(5).nAgents(), 1);
          CHECK_EQ(network.edge(7).nAgents(), 0);
          CHECK(rb.agents().empty());
        }
        dynamics.evolve(false);
        THEN("The agent with priority leaves the roundabout") {
          CHECK_EQ(network.edge(3).nAgents(), 0);
          CHECK_EQ(network.edge(5).nAgents(), 0);
          CHECK(rb.agents().empty());
        }
      }
    }
  }
  SUBCASE("Travelled distance") {
    GIVEN("A dynamics with a two-streets network and an agent") {
      Street s1{0, std::make_pair(0, 1), 3.};
      Street s2{5, std::make_pair(1, 2), 1.};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s1, s2);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(2), 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent has travelled the correct distance") {
          auto const& pAgent{dynamics.graph().edge(5).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), 3);
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 5);
          CHECK_EQ(pAgent->speed(), doctest::Approx(13.8888888889));
          CHECK_EQ(pAgent->distance(), 3.);
        }
      }
    }
    GIVEN(
        "A dynamics with one street with limited flow and one normal street network and "
        "an agent") {
      Street s1{0, std::make_pair(0, 1), 3.};
      Street s2{1, std::make_pair(1, 2), 1.};
      s1.setTransportCapacity(0.3);
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s1, s2);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(dynamics.itineraries().at(2), 0);
      WHEN("We evolve the dynamics") {
        // Logger::setLogLevel(dsf::log_level_t::DEBUG);
        CHECK_EQ(dynamics.nAgents(), 1);
        while (dynamics.nAgents() > 0) {
          dynamics.evolve(false);
        }
        // Logger::setLogLevel(dsf::log_level_t::INFO);
        THEN("The agent has travelled the correct distance") {
          auto const& distance{dynamics.meanTravelDistance()};
          CHECK_EQ(distance.mean, 4.);
          CHECK_EQ(distance.std, doctest::Approx(0.));
          CHECK_EQ(dynamics.nAgents(), 0);
        }
      }
    }
  }
  SUBCASE("Intersection right of way") {
    GIVEN("A dynamics object with five nodes and eight streets") {
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addNode<Intersection>(0, dsf::geometry::Point(0, 0));
      graph2.addNode<Intersection>(1, dsf::geometry::Point(1, -1));   // A
      graph2.addNode<Intersection>(2, dsf::geometry::Point(1, 1));    // B
      graph2.addNode<Intersection>(3, dsf::geometry::Point(-1, 1));   // C
      graph2.addNode<Intersection>(4, dsf::geometry::Point(-1, -1));  // D
      graph2.addEdge<Street>(1, std::make_pair(0, 1), 30., 10.);
      graph2.addEdge<Street>(2, std::make_pair(0, 2), 30., 10.);
      graph2.addEdge<Street>(3, std::make_pair(0, 3), 10., 10.);
      graph2.addEdge<Street>(4, std::make_pair(0, 4), 10., 10.);
      graph2.addEdge<Street>(5, std::make_pair(1, 0), 10., 10.);
      graph2.addEdge<Street>(10, std::make_pair(2, 0), 10., 10.);
      graph2.addEdge<Street>(15, std::make_pair(3, 0), 10., 10.);
      graph2.addEdge<Street>(20, std::make_pair(4, 0), 10., 10.);
      FirstOrderDynamics dynamics{std::move(graph2), false, 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.graph().node(0).setCapacity(3);
      dynamics.graph().node(0).setTransportCapacity(1);
      auto& nodeO{dynamic_cast<Intersection&>(dynamics.graph().node(0))};
      dynamics.addItinerary(1, 1);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      WHEN("We add agents and evolve the dynamics") {
        // add an agent in C, D, A
        dynamics.addAgent(dynamics.itineraries().at(2), 4);  // Second
        dynamics.addAgent(dynamics.itineraries().at(2), 3);  // Third
        dynamics.addAgent(dynamics.itineraries().at(2), 1);  // First
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent in A passes first") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 2);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 20);
        }
        dynamics.evolve(false);
        THEN("The agent in D passes second") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 2);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 15);
        }
        dynamics.evolve(false);
        THEN("The agent in C passes last") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 3);
          CHECK(nodeO.agents().empty());
        }
      }
      WHEN("We add agents of another itinerary and update the dynamics") {
        dynamics.addAgent(dynamics.itineraries().at(1), 2);
        dynamics.addAgent(dynamics.itineraries().at(1), 3);
        dynamics.addAgent(dynamics.itineraries().at(1), 4);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent in D passes first") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 2);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 15);
        }
        dynamics.evolve(false);
        THEN("The agent in C passes second") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 2);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 10);
        }
        dynamics.evolve(false);
        THEN("The agent in C passes last") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 3);
          CHECK(nodeO.agents().empty());
        }
      }
      WHEN("We set street priorities and add agents") {
        nodeO.addStreetPriority(5);
        nodeO.addStreetPriority(15);
        dynamics.addAgent(dynamics.itineraries().at(2), 1);
        dynamics.addAgent(dynamics.itineraries().at(2), 4);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent in A passes first because it's on the main road") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 20);
        }
      }
    }
  }
  SUBCASE("Transition probabilities") {
    GIVEN("A simple network with a Y-junction") {
      // Create a network with one incoming street and two outgoing streets
      // Street layout:
      //   0 --[s0]--> 1 --[s1]--> 2
      //                \--[s2]--> 3
      Street s0{0, std::make_pair(0, 1), 300., 50.};
      Street s1{1, std::make_pair(1, 2), 1000., 50.};
      Street s2{2, std::make_pair(1, 3), 1000., 30.};

      RoadNetwork graph;
      graph.addStreets(s0, s1, s2);
      graph.autoMapStreetLanes();
      graph.adjustNodeCapacities();
      graph.setEdgeWeight("length");

      FirstOrderDynamics dynamics{std::move(graph), false, 42};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setOriginNodes({{0, 1.0}});

      WHEN("We add multiple random agents and evolve the system") {
        // spdlog::set_level(spdlog::level::debug);
        dynamics.addAgents(6, AgentInsertionMethod::RANDOM);
        CHECK_EQ(dynamics.nAgents(), 6);
        // Evolve to get agents onto street 0
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        CHECK_EQ(dynamics.graph().edge(0).nAgents(), 6);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);

        THEN("The distribution of agents follows the transition probabilities") {
          CHECK_EQ(dynamics.graph().edge(0).nAgents(), 0);
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 4);
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 2);
        }
        // spdlog::set_level(spdlog::level::info);
      }
    }

    GIVEN("A network with three possible exits from a node") {
      // Create a network with one incoming street and three outgoing streets
      // Street layout:
      //   0 --[s0]--> 1 --[s1]--> 2
      //                |--[s2]--> 3
      //                \--[s3]--> 4
      Street s0{0, std::make_pair(0, 1), 300., 50.};
      Street s1{1, std::make_pair(1, 2), 1000., 50.};
      Street s2{2, std::make_pair(1, 3), 1000., 40.};
      Street s3{3, std::make_pair(1, 4), 1000., 30.};

      RoadNetwork graph;
      graph.addStreets(s0, s1, s2, s3);
      graph.autoMapStreetLanes();
      graph.adjustNodeCapacities();
      graph.setEdgeWeight("length");

      FirstOrderDynamics dynamics{std::move(graph), false, 123};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.);
      dynamics.setOriginNodes({{0, 1.0}});

      WHEN("We add multiple random agents and evolve the system") {
        dynamics.addAgents(6, AgentInsertionMethod::RANDOM);
        CHECK_EQ(dynamics.nAgents(), 6);
        // Evolve to get agents onto street 0
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        CHECK_EQ(dynamics.graph().edge(0).nAgents(), 6);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);

        THEN("The distribution of agents follows the transition probabilities") {
          CHECK_EQ(dynamics.graph().edge(0).nAgents(), 0);
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(3).nAgents(), 2);
        }
      }
    }
  }
}

TEST_CASE("RoadDynamics Configuration") {
  auto defaultNetwork = RoadNetwork{};
  // We need at least some nodes to add agents
  // Assuming the data files exist where Test_dynamics.cpp expects them
  defaultNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
  defaultNetwork.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());

  FirstOrderDynamics dynamics{std::move(defaultNetwork), false, 42};

  SUBCASE("originCounts and destinationCounts") {
    GIVEN("A simple network with origin and destination nodes") {
      Street s1{0, std::make_pair(0, 1), 13.8888888889};
      Street s2{1, std::make_pair(1, 2), 13.8888888889};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s1, s2);
      FirstOrderDynamics dyn{std::move(graph2), false, 42};
      dyn.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dyn.addItinerary(2, 2);
      dyn.updatePaths();

      WHEN("We add agents with source nodes and evolve until they reach destination") {
        dyn.addAgent(dyn.itineraries().at(2), 0);
        dyn.addAgent(dyn.itineraries().at(2), 0);
        dyn.addAgent(dyn.itineraries().at(2), 1);

        THEN("originCounts returns the correct counts") {
          auto counts = dyn.originCounts(false);
          CHECK_EQ(counts.at(0), 2);
          CHECK_EQ(counts.at(1), 1);
        }

        THEN("originCounts with bReset=true clears the counts") {
          auto counts = dyn.originCounts(true);
          CHECK_EQ(counts.at(0), 2);
          CHECK_EQ(counts.at(1), 1);

          auto countsAfterReset = dyn.originCounts(false);
          CHECK(countsAfterReset.empty());
        }

        // Evolve until agents reach destination
        while (dyn.nAgents() > 0) {
          dyn.evolve(false);
        }

        THEN("destinationCounts returns the correct counts") {
          auto destCounts = dyn.destinationCounts(false);
          CHECK_EQ(destCounts.at(2), 3);
        }

        THEN("destinationCounts with bReset=true clears the counts") {
          auto destCounts = dyn.destinationCounts(true);
          CHECK_EQ(destCounts.at(2), 3);

          auto destCountsAfterReset = dyn.destinationCounts(false);
          CHECK(destCountsAfterReset.empty());
        }
      }
    }

    GIVEN("Multiple destinations") {
      Street s0_1{0, std::make_pair(0, 1), 13.8888888889};
      Street s1_2{1, std::make_pair(1, 2), 13.8888888889};
      Street s1_3{2, std::make_pair(1, 3), 13.8888888889};
      RoadNetwork graph2;
      graph2.setEdgeWeight("length");
      graph2.addStreets(s0_1, s1_2, s1_3);
      FirstOrderDynamics dyn{std::move(graph2), false, 42};
      dyn.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dyn.setDestinationNodes({2, 3});
      dyn.updatePaths();

      WHEN("Agents travel to different destinations") {
        dyn.addAgent(dyn.itineraries().at(2), 0);
        dyn.addAgent(dyn.itineraries().at(3), 0);
        dyn.addAgent(dyn.itineraries().at(3), 0);

        // Evolve until all agents reach destination
        while (dyn.nAgents() > 0) {
          dyn.evolve(false);
        }

        THEN("destinationCounts tracks each destination separately") {
          auto destCounts = dyn.destinationCounts(false);
          CHECK_EQ(destCounts.at(2), 1);
          CHECK_EQ(destCounts.at(3), 2);
        }
      }
    }
  }

  SUBCASE("setMeanTravelDistance") {
    CHECK_THROWS_AS(dynamics.setMeanTravelDistance(-1.0), std::invalid_argument);
    CHECK_THROWS_AS(dynamics.setMeanTravelDistance(0.0), std::invalid_argument);

    dynamics.setMeanTravelDistance(1000.0);

    // Set origin nodes
    std::unordered_map<Id, double> origins;
    for (const auto& [id, node] : defaultNetwork.nodes()) {
      origins[id] = 1.0;
    }
    dynamics.setOriginNodes(origins);

    dynamics.addAgents(10, AgentInsertionMethod::RANDOM);

    const auto& agents = dynamics.agents();
    CHECK(agents.size() == 10);
    for (const auto& agent : agents) {
      CHECK_NOTHROW(agent->maxDistance());
      CHECK(agent->maxDistance() >= 0.0);
    }
  }

  SUBCASE("setMeanTravelTime") {
    dynamics.setMeanTravelTime(3600);

    std::unordered_map<Id, double> origins;
    for (const auto& [id, node] : defaultNetwork.nodes()) {
      origins[id] = 1.0;
    }
    dynamics.setOriginNodes(origins);

    dynamics.addAgents(10, AgentInsertionMethod::RANDOM);

    const auto& agents = dynamics.agents();
    CHECK(agents.size() == 10);
    for (const auto& agent : agents) {
      CHECK_NOTHROW(agent->maxTime());
      CHECK(agent->maxTime() >= 0);
    }
  }
}
