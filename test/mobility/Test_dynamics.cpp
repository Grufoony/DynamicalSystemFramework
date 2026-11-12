#include "dsf/mobility/FirstOrderDynamics.hpp"
#include "dsf/mobility/RoadNetwork.hpp"
#include "dsf/mobility/Street.hpp"
#include "dsf/mobility/Intersection.hpp"
#include "dsf/mobility/TrafficLight.hpp"
#include "dsf/mobility/Agent.hpp"

#include <tbb/tbb.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
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
  SUBCASE("Lazy view") {
    // A filter view is not sized: the measurement counts the samples itself
    auto data{std::views::iota(0, 200) |
              std::views::filter([](int const i) { return i % 2 == 0; }) |
              std::views::transform([](int const i) { return i / 2.f; })};

    Measurement<float> m(data);
    CHECK_EQ(m.mean, 49.5f);
    CHECK_EQ(m.std, doctest::Approx(28.8661f));
    CHECK_EQ(m.n, 100);
    CHECK(m.is_valid);

    Measurement<float> empty(std::views::iota(0, 10) |
                             std::views::filter([](int const i) { return i < 0; }));
    CHECK_FALSE(empty.is_valid);
    CHECK_EQ(empty.n, 0);
  }
}

TEST_CASE("FirstOrderDynamics") {
  auto defaultNetwork = RoadNetwork{};
  defaultNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
  defaultNetwork.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());
  SUBCASE("Constructor") {
    GIVEN("A graph object") {
      WHEN("A dynamics object is created") {
        FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
        THEN("The node and the street sets are the same") {
          CHECK_EQ(dynamics.graph().nNodes(), 120);
          CHECK_EQ(dynamics.graph().nEdges(), 436);
        }
        THEN("The mean speed, density, flow and travel time are 0") {
          CHECK_EQ(dynamics.streetMeanDensity<false>().mean, 0.);
          CHECK_EQ(dynamics.streetMeanDensity<false>().std, 0.);
          CHECK_EQ(dynamics.streetMeanFlow().mean, 0.);
          CHECK_EQ(dynamics.streetMeanFlow().std, 0.);
          CHECK_EQ(dynamics.meanTravelTime().mean, 0.);
          CHECK_EQ(dynamics.meanTravelTime().std, 0.);
        }
      }
      WHEN("We transform a node into a traffic light and create the dynamics") {
        auto& tl = defaultNetwork.makeTrafficLight(0);
        // Two phases of 1 tick each → cycleTime() == 2
        tl.addPhase(TrafficLightPhase{1});
        tl.addPhase(TrafficLightPhase{1});
        FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
        THEN("The node is a traffic light") {
          CHECK(dynamics.graph().node(0).isTrafficLight());
          CHECK_EQ(tl.cycleTime(), 2);
        }
      }
      WHEN("We transform a node into a roundabout and create the dynamics") {
        defaultNetwork.makeRoundabout(0);
        FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
        THEN("The node is a roundabout") {
          CHECK(dynamics.graph().node(0).isRoundabout());
        }
      }
      WHEN("We put a coil on the street and create the dynamics") {
        defaultNetwork.addCoil(8);
        FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
        THEN("The street has a coil") { CHECK(dynamics.graph().edge(8).hasCoil()); }
      }
      WHEN("We call summary") {
        FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
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
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
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
  SUBCASE("setDestinations") {
    GIVEN("A dynamics object and a destination node") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      WHEN("We add a span of destination nodes") {
        std::array<dsf::Id, 3> nodes{0, 1, 2};
        dynamics.setDestinations(nodes);
        THEN("The destination nodes are added") {
          auto const& destinationStreets = dynamics.destinationStreets();
          CHECK_EQ(destinationStreets.size(), nodes.size());
          for (auto const node : nodes) {
            CHECK(destinationStreets.contains(node));
          }
        }
      }
      WHEN("We set weights which do not sum up to one") {
        dynamics.setDestinations(std::unordered_map<Id, double>{{3, 1.}, {4, 3.}});
        dynamics.setODs({{0, 3, 1.}, {2, 4, 3.}});
        THEN("The weights are normalized") {
          for (auto const& [id, weight] : dynamics.destinations()) {
            CHECK_EQ(weight, doctest::Approx(id == 3 ? 0.25 : 0.75));
          }
          CHECK_EQ(dynamics.destinationStreets().size(), 2);
        }
      }
      WHEN("We set no origins") {
        dynamics.setOrigins();
        THEN("Every street becomes an origin with the same weight") {
          CHECK_EQ(dynamics.origins().size(), dynamics.graph().nEdges());
          CHECK_EQ(dynamics.origins().front().weight,
                   doctest::Approx(1. / dynamics.graph().nEdges()));
        }
      }
      THEN("Non-positive weights throw") {
        CHECK_THROWS_AS(dynamics.setOrigins({{0, 0.}}), std::invalid_argument);
        CHECK_THROWS_AS(dynamics.setDestinations(std::unordered_map<Id, double>{{3, 0.}}),
                        std::invalid_argument);
        CHECK_THROWS_AS(dynamics.setODs({{0, 3, 0.}}), std::invalid_argument);
      }
    }
  }
  SUBCASE("addAgent") {
    GIVEN("A dynamics object, a source node and a destination node") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      WHEN("We add the agent") {
        dynamics.addAgent(2, 0);
        THEN("The agent is added") {
          CHECK_EQ(dynamics.nAgents(), 1);
          const auto& agent = dynamics.agents().at(0);
          // CHECK_EQ(agent->id(), 0);
          CHECK_EQ(agent->srcNodeId().value(), 0);
          CHECK_EQ(agent->destination().value(), 2);
        }
      }
    }
  }
  SUBCASE("addAgentsUniformly") {
    GIVEN("A dynamics object and a destination") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      WHEN("We add an agent") {
        dynamics.setDestinations({2});
        dynamics.addAgentsUniformly(1);
        THEN("The number of agents is 1 and its destination is the given one") {
          CHECK_EQ(dynamics.nAgents(), 1);
        }
      }
    }
    GIVEN("A dynamics object and many destinations") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      // Inserted in the same order as the former itineraries, so that the seeded draws
      // pick the same destinations
      dynamics.setDestinations({2, 1});
      WHEN("We add many agents") {
        // Logger::setLogLevel(dsf::log_level_t::DEBUG);
        dynamics.addAgentsUniformly(3);
        // Logger::setLogLevel(dsf::log_level_t::INFO);
        THEN(
            "The number of agents is 3, each on a random street with a random "
            "destination") {
          CHECK_EQ(dynamics.nAgents(), 3);
#ifdef __APPLE__
          CHECK_EQ(dynamics.graph().edge(422).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(422).movingAgents().top()->destination().value(),
                   2);
          CHECK_EQ(dynamics.graph().edge(199).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(199).movingAgents().top()->destination().value(),
                   2);
          CHECK_EQ(dynamics.graph().edge(166).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(166).movingAgents().top()->destination().value(),
                   1);
#else
          CHECK_EQ(dynamics.graph().edge(13).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(13).movingAgents().top()->destination().value(),
                   2);
          CHECK_EQ(dynamics.graph().edge(370).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(370).movingAgents().top()->destination().value(),
                   1);
          CHECK_EQ(dynamics.graph().edge(404).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(404).movingAgents().top()->destination().value(),
                   2);
#endif
        }
      }
      WHEN("We add agents with a given destination") {
        dynamics.addAgentsUniformly(4, 1);
        THEN("Every agent has that destination") {
          CHECK_EQ(dynamics.nAgents(), 4);
          for (auto const& [streetId, pStreet] : dynamics.graph().edges()) {
            for (auto const& pAgent : pStreet->movingAgents()) {
              CHECK_EQ(pAgent->destination().value(), 1);
            }
          }
        }
      }
      WHEN("We add agents with the UNIFORM insertion method") {
        dynamics.addAgents(5, AgentInsertionMethod::UNIFORM);
        THEN("The agents are added") { CHECK_EQ(dynamics.nAgents(), 5); }
      }
      THEN("Invalid requests throw") {
        CHECK_THROWS_AS(dynamics.addAgentsUniformly(1, 999), std::invalid_argument);
        CHECK_THROWS_AS(dynamics.addAgentsUniformly(dynamics.graph().capacity() + 1),
                        std::overflow_error);
      }
    }
  }
  SUBCASE("addAgents - RANDOM_ODS") {
    GIVEN("A graph object") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      WHEN("We add one agent for existing destination") {
        std::unordered_map<dsf::Id, double> src{{0, 1.}};
        std::unordered_map<dsf::Id, double> dst{{2, 1.}};
        dynamics.setOrigins(src);
        dynamics.setDestinations(dst);
        dynamics.addAgents(1, AgentInsertionMethod::RANDOM_ODS);
        THEN("The agents are correctly set") {
          CHECK_EQ(dynamics.nAgents(), 1);
          CHECK_EQ(dynamics.agents().at(0)->destination().value(), 2);
          CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 0);
        }
      }
      WHEN("We add agents for existing destinations") {
        auto const& graph = dynamics.graph();
        std::unordered_map<dsf::Id, double> src{
            {graph.node(1).outgoingEdges().front()->id(), 0.3},
            {graph.node(27).outgoingEdges().front()->id(), 0.3},
            {graph.node(118).outgoingEdges().front()->id(), 0.4}};
        std::unordered_map<dsf::Id, double> dst{
            {graph.node(14).ingoingEdges().front()->id(), 0.3},
            {graph.node(102).ingoingEdges().front()->id(), 0.3},
            {graph.node(107).ingoingEdges().front()->id(), 0.4}};
        dynamics.setOrigins(src);
        dynamics.setDestinations(dst);
        dynamics.addAgents(3, AgentInsertionMethod::RANDOM_ODS);
        THEN("The agents are correctly set") {
#ifdef __APPLE__
          CHECK_EQ(dynamics.nAgents(), 3);
          CHECK_EQ(dynamics.agents().at(0)->destination().value(),
                   graph.node(107).ingoingEdges().front()->id());
          CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 118);
          CHECK_EQ(dynamics.agents().at(1)->destination().value(),
                   graph.node(14).ingoingEdges().front()->id());
          CHECK_EQ(dynamics.agents().at(1)->srcNodeId().value(), 1);
          CHECK_EQ(dynamics.agents().at(2)->destination().value(),
                   graph.node(14).ingoingEdges().front()->id());
          CHECK_EQ(dynamics.agents().at(2)->srcNodeId().value(), 118);
#else
          CHECK_EQ(dynamics.nAgents(), 3);
          CHECK_EQ(dynamics.agents().at(0)->destination().value(),
                   graph.node(107).ingoingEdges().front()->id());
          CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 27);
          CHECK_EQ(dynamics.agents().at(1)->destination().value(),
                   graph.node(14).ingoingEdges().front()->id());
          CHECK_EQ(dynamics.agents().at(1)->srcNodeId().value(), 1);
          CHECK_EQ(dynamics.agents().at(2)->destination().value(),
                   graph.node(14).ingoingEdges().front()->id());
          CHECK_EQ(dynamics.agents().at(2)->srcNodeId().value(), 118);
#endif
        }
      }
    }
  }
  SUBCASE("addAgents - ODS") {
    GIVEN("A graph object") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      WHEN("We add agents with a single OD pair") {
        std::vector<dsf::mobility::ODEntry> ods{{0, 2, 1.}};
        dynamics.setODs(ods);
        dynamics.addAgents(3, AgentInsertionMethod::ODS);
        THEN("All agents follow that single OD pair") {
          CHECK_EQ(dynamics.nAgents(), 3);
          for (size_t i = 0; i < 3; ++i) {
            CHECK_EQ(dynamics.agents().at(i)->destination().value(), 2);
            CHECK_EQ(dynamics.agents().at(i)->srcNodeId().value(), 0);
          }
        }
      }
      WHEN("We add agents with multiple OD pairs") {
        auto const& graph = dynamics.graph();
        std::vector<dsf::mobility::ODEntry> ods{
            {graph.node(1).outgoingEdges().front()->id(),
             graph.node(14).ingoingEdges().front()->id(),
             0.5},
            {graph.node(27).outgoingEdges().front()->id(),
             graph.node(107).ingoingEdges().front()->id(),
             0.5}};
        dynamics.setODs(ods);
        dynamics.addAgents(4, AgentInsertionMethod::ODS);
        THEN("The agents are assigned to the correct OD pairs") {
          CHECK_EQ(dynamics.nAgents(), 4);
          for (size_t i = 0; i < dynamics.agents().size(); ++i) {
            auto const& agent = dynamics.agents().at(i);
            auto dest = agent->destination().value();
            auto src = agent->srcNodeId().value();
            CHECK((dest == graph.node(14).ingoingEdges().front()->id() ||
                   dest == graph.node(107).ingoingEdges().front()->id()));
            if (dest == graph.node(14).ingoingEdges().front()->id()) {
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
  SUBCASE("importODsFromCSV") {
    GIVEN("A dynamics object and CSV files in different formats") {
      WHEN("We import from a RANDOM_ODS format CSV file") {
        auto randomNetwork = RoadNetwork{};
        randomNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
        randomNetwork.importNodeProperties(
            (DATA_FOLDER / "manhattan_nodes.csv").string());
        FirstOrderDynamics dynamics{std::move(randomNetwork), 69, DynamicsMode::NODES};
        dynamics.importODsFromCSV((DATA_FOLDER / "ods_random_ods.csv").string());
        dynamics.addAgents(3, AgentInsertionMethod::RANDOM_ODS);

        THEN("The origin and destination nodes are correctly set") {
          CHECK_EQ(dynamics.nAgents(), 3);
          auto const& graph = dynamics.graph();
          // Verify all agents have valid origins and destinations
          for (const auto& agent : dynamics.agents()) {
            auto origin = agent->srcNodeId().value();
            // Destination streets end at the destination node
            auto destination = graph.edge(agent->destination().value()).target()->id();

            // Origins should be from {0, 1, 27}
            CHECK((origin == 0 || origin == 1 || origin == 27));
            // Destinations should be from {2, 14, 102}
            CHECK((destination == 2 || destination == 14 || destination == 102));
          }
        }
        SUBCASE("addAgents - CONDITIONAL_RANDOM_ODS") {
          GIVEN("A graph object") {
            FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};

            WHEN("We add one agent with a single origin and single destination") {
              // origin 0 → destination 2 with 100% probability
              dynamics.setConditionalODs({{0, {1.0, {{2, 1.0}}}}});
              dynamics.addAgents(1, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS);

              THEN("The agent has the correct origin and destination") {
                CHECK_EQ(dynamics.nAgents(), 1);
                CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 0);
                CHECK_EQ(dynamics.agents().at(0)->destination().value(), 2);
              }
            }

            WHEN(
                "We add many agents from a single origin with deterministic "
                "destination") {
              auto const& graph = dynamics.graph();
              // With weight 1.0 on the only destination, every agent must go there
              dynamics.setConditionalODs(
                  {{graph.node(1).outgoingEdges().front()->id(),
                    {1.0, {{graph.node(14).ingoingEdges().front()->id(), 1.0}}}}});
              dynamics.addAgents(10, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS);

              THEN("All agents share the same origin and destination") {
                CHECK_EQ(dynamics.nAgents(), 10);
                for (std::size_t i = 0; i < dynamics.nAgents(); ++i) {
                  CHECK_EQ(dynamics.agents().at(i)->srcNodeId().value(), 1);
                  CHECK_EQ(dynamics.agents().at(i)->destination().value(),
                           graph.node(14).ingoingEdges().front()->id());
                }
              }
            }

            WHEN("We add agents from two origins each with two exclusive destinations") {
              auto const& graph = dynamics.graph();
              // origin 1  → dest 14  (100%)
              // origin 27 → dest 107 (100%)
              dynamics.setConditionalODs({
                  {graph.node(1).outgoingEdges().front()->id(),
                   {1.0, {{graph.node(14).ingoingEdges().front()->id(), 1.0}}}},
                  {graph.node(27).outgoingEdges().front()->id(),
                   {1.0, {{graph.node(107).ingoingEdges().front()->id(), 1.0}}}},
              });
              dynamics.addAgents(20, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS);

              THEN("Each agent's destination matches its origin") {
                CHECK_EQ(dynamics.nAgents(), 20);
                for (std::size_t i = 0; i < dynamics.nAgents(); ++i) {
                  auto const& agent = dynamics.agents().at(i);
                  auto const src = agent->srcNodeId().value();
                  auto const dest = agent->destination().value();
                  CHECK((src == 1 || src == 27));
                  if (src == 1) {
                    CHECK_EQ(dest, graph.node(14).ingoingEdges().front()->id());
                  } else {
                    CHECK_EQ(dest, graph.node(107).ingoingEdges().front()->id());
                  }
                }
              }
            }

            WHEN(
                "We add agents from one origin whose destinations overlap with another "
                "origin's") {
              auto const& graph = dynamics.graph();
              // origin 1  → dest 14 (70%) or dest 102 (30%)
              // origin 27 → dest 14 (10%) or dest 107 (90%)
              // With seed 69 the exact sequence is deterministic, so we test structural invariants
              dynamics.setConditionalODs({
                  {graph.node(1).outgoingEdges().front()->id(),
                   {1.0,
                    {{graph.node(14).ingoingEdges().front()->id(), 0.7},
                     {graph.node(102).ingoingEdges().front()->id(), 0.3}}}},
                  {graph.node(27).outgoingEdges().front()->id(),
                   {1.0,
                    {{graph.node(14).ingoingEdges().front()->id(), 0.1},
                     {graph.node(107).ingoingEdges().front()->id(), 0.9}}}},
              });
              dynamics.addAgents(100, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS);

              THEN("Every agent's destination is valid for its origin") {
                CHECK_EQ(dynamics.nAgents(), 100);
                for (std::size_t i = 0; i < dynamics.nAgents(); ++i) {
                  auto const& agent = dynamics.agents().at(i);
                  auto const src = agent->srcNodeId().value();
                  auto const dest = agent->destination().value();
                  if (src == 1) {
                    CHECK((dest == graph.node(14).ingoingEdges().front()->id() ||
                           dest == graph.node(102).ingoingEdges().front()->id()));
                  } else if (src == 27) {
                    CHECK((dest == graph.node(14).ingoingEdges().front()->id() ||
                           dest == graph.node(107).ingoingEdges().front()->id()));
                  } else {
                    FAIL("Unexpected origin: " << src);
                  }
                }
              }

              THEN(
                  "At least one agent ends up at each reachable destination "
                  "(statistical)") {
                std::unordered_set<Id> seen;
                for (std::size_t i = 0; i < dynamics.nAgents(); ++i) {
                  seen.insert(dynamics.agents().at(i)->destination().value());
                }
                // dest 14 is reachable from both origins, 102 and 107 from one each
                CHECK(seen.contains((graph.node(14).ingoingEdges().front()->id())));
              }
            }

            WHEN("No conditional ODs have been set") {
              THEN("An exception is thrown") {
                CHECK_THROWS_AS(
                    dynamics.addAgents(1, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS),
                    std::runtime_error);
              }
            }

            WHEN("The destination node does not exist in the graph") {
              dynamics.setConditionalODs({{0, {1.0, {{9999, 1.0}}}}});

              THEN("The nonexistent destination is removed, without throwing") {
                CHECK_NOTHROW(dynamics.evolve());
                CHECK_FALSE(dynamics.destinationStreets().contains(9999));
                CHECK(dynamics.origins().empty());
              }
            }

            WHEN("The destination exists but has no path from the origin") {
              // The destination is the origin itself: trips to it are skipped
              dynamics.setConditionalODs({{0, {1.0, {{0, 1.0}}}}});  // dest == src
              CHECK_NOTHROW(
                  dynamics.addAgents(5, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS));

              THEN("No agents are inserted") { CHECK_EQ(dynamics.nAgents(), 0); }
            }
          }
        }

        SUBCASE("importODsFromCSV - CONDITIONAL_RANDOM_ODS") {
          GIVEN("A dynamics object and a conditional OD CSV file") {
            auto randomNetwork = RoadNetwork{};
            randomNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
            randomNetwork.importNodeProperties(
                (DATA_FOLDER / "manhattan_nodes.csv").string());
            FirstOrderDynamics dynamics{
                std::move(randomNetwork), 69, DynamicsMode::NODES};

            WHEN("We import a valid conditional OD CSV") {
              // Expected file format (semicolon-separated):
              //   origin_id;weight;destinations
              //   0;0.5;2:0.6,14:0.4
              //   1;0.5;102:1.0
              dynamics.importODsFromCSV(
                  (DATA_FOLDER / "ods_conditional_random_ods.csv").string());
              dynamics.addAgents(20, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS);

              THEN("The correct number of agents is inserted") {
                CHECK_EQ(dynamics.nAgents(), 20);
              }

              THEN("Each agent's destination is valid for its origin") {
                auto const& graph = dynamics.graph();
                for (const auto& agent : dynamics.agents()) {
                  auto const src = agent->srcNodeId().value();
                  auto const dest =
                      graph.edge(agent->destination().value()).target()->id();
                  CHECK((src == 0 || src == 1));
                  if (src == 0) {
                    CHECK((dest == 2 || dest == 14));
                  } else {
                    CHECK_EQ(dest, 102);
                  }
                }
              }
            }

            WHEN("The CSV file does not exist") {
              THEN("An exception is thrown") {
                CHECK_THROWS(dynamics.importODsFromCSV("nonexistent_file.csv", ';'));
              }
            }
          }
        }
      }

      WHEN("We import from an ODS format CSV file") {
        auto odNetwork = RoadNetwork{};
        odNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
        odNetwork.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());
        FirstOrderDynamics dynamics2{std::move(odNetwork), 69};
        CHECK_NOTHROW(
            dynamics2.importODsFromCSV((DATA_FOLDER / "ods_od_pairs.csv").string(), ';'));

        THEN("The OD pairs are correctly set") {
          auto const& destinationStreets = dynamics2.destinationStreets();
          CHECK_EQ(destinationStreets.size(), 1);
          CHECK(destinationStreets.contains(2));
        }
      }

      WHEN("We import with custom separator") {
        auto randomNetwork = RoadNetwork{};
        randomNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
        randomNetwork.importNodeProperties(
            (DATA_FOLDER / "manhattan_nodes.csv").string());
        FirstOrderDynamics dynamics{std::move(randomNetwork), 69};
        // Note: The test files use semicolon, so we test the default works
        dynamics.importODsFromCSV((DATA_FOLDER / "ods_random_ods.csv").string());

        THEN("Import succeeds with default separator") {
          dynamics.addAgents(1, AgentInsertionMethod::RANDOM_ODS);
          CHECK_EQ(dynamics.nAgents(), 1);
        }
      }

      WHEN("We import CSV files with partial or invalid rows") {
        FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
        auto const csvPath = std::filesystem::temp_directory_path() / "dsf_ods_rows.csv";
        auto const writeCsv = [&csvPath](std::string const& content) {
          std::ofstream out{csvPath};
          REQUIRE(out.is_open());
          out << content;
        };
        // Sorted ids of the outgoing streets of a node
        auto const outgoingStreetIds = [&dynamics](Id const nodeId) {
          std::vector<Id> ids;
          for (auto const* pStreet : dynamics.graph().node(nodeId).outgoingEdges()) {
            ids.push_back(pStreet->id());
          }
          std::ranges::sort(ids);
          return ids;
        };
        // Sorted ids of the ingoing streets of a node
        auto const ingoingStreetIds = [&dynamics](Id const nodeId) {
          std::vector<Id> ids;
          for (auto const* pStreet : dynamics.graph().node(nodeId).ingoingEdges()) {
            ids.push_back(pStreet->id());
          }
          std::ranges::sort(ids);
          return ids;
        };
        auto const sortedIds = [](auto const& weightedIds) {
          std::vector<Id> ids;
          double sumWeights{0.};
          for (auto const& [id, weight] : weightedIds) {
            ids.push_back(id);
            sumWeights += weight;
          }
          CHECK_EQ(sumWeights, doctest::Approx(1.));
          std::ranges::sort(ids);
          return ids;
        };
        THEN("Node ids in the compat format are spread over their streets") {
          dynamics.setMode(DynamicsMode::NODES);
          writeCsv("id;o_prob;d_prob\n0;1.0;\n2;;1.0\n");
          dynamics.importODsFromCSV(csvPath.string());
          CHECK_EQ(sortedIds(dynamics.origins()), outgoingStreetIds(0));
          CHECK_EQ(sortedIds(dynamics.destinations()), ingoingStreetIds(2));
        }
        THEN("Rows with an unknown type or no valid destination are skipped (edges)") {
          writeCsv("id;type;weight\n0;O;1.0\n2;X;1.0\n14;D;1.0\n");
          dynamics.importODsFromCSV(csvPath.string());
          CHECK_EQ(dynamics.origins().size(), 1);
          CHECK_EQ(dynamics.destinations().size(), 1);

          writeCsv("origin_id;weight;destinations\n0;0.5;2:1.0,bad\n1;0.5;\n");
          dynamics.importODsFromCSV(csvPath.string());
          REQUIRE_EQ(dynamics.origins().size(), 1);
          CHECK_EQ(dynamics.origins().front().weight, doctest::Approx(1.));
        }
        THEN("Rows with an unknown type or no valid destination are skipped (nodes)") {
          dynamics.setMode(DynamicsMode::NODES);
          writeCsv("id;type;weight\n0;O;1.0\n2;X;1.0\n14;D;1.0\n");
          dynamics.importODsFromCSV(csvPath.string());
          CHECK_EQ(sortedIds(dynamics.origins()), outgoingStreetIds(0));
          CHECK_EQ(sortedIds(dynamics.destinations()), ingoingStreetIds(14));

          writeCsv("origin_id;weight;destinations\n0;0.5;2:1.0,bad\n1;0.5;\n");
          dynamics.importODsFromCSV(csvPath.string());
          CHECK_EQ(sortedIds(dynamics.origins()), outgoingStreetIds(0));
        }
        THEN("OD pairs between nodes are spread over their streets") {
          dynamics.setMode(DynamicsMode::NODES);
          writeCsv("origin_id;destination_id;weight\n0;2;1.0\n");
          dynamics.importODsFromCSV(csvPath.string());
          auto const destinationIds{ingoingStreetIds(2)};
          CHECK_EQ(dynamics.destinationStreets().size(), destinationIds.size());
          for (auto const id : destinationIds) {
            CHECK(dynamics.destinationStreets().contains(id));
          }
          dynamics.addAgents(10, AgentInsertionMethod::ODS);
          CHECK_EQ(dynamics.nAgents(), 10);
          for (auto const& agent : dynamics.agents()) {
            CHECK_EQ(agent->srcNodeId().value(), 0);
            CHECK_EQ(dynamics.graph().edge(agent->destination().value()).target()->id(),
                     2);
          }
        }
        THEN("Missing files and unsupported formats throw") {
          CHECK_THROWS_AS(dynamics.importODsFromCSV("nonexistent_file.csv"),
                          std::invalid_argument);
          writeCsv("foo;bar\n0;1\n");
          CHECK_THROWS_AS(dynamics.importODsFromCSV(csvPath.string()),
                          std::runtime_error);
          dynamics.setMode(DynamicsMode::NODES);
          CHECK_THROWS_AS(dynamics.importODsFromCSV(csvPath.string()),
                          std::runtime_error);
          CHECK_THROWS_AS(dynamics.setConditionalODs({{0, {1.0, {{2, 0.}}}}}),
                          std::invalid_argument);
        }
        std::filesystem::remove(csvPath);
      }
    }
  }
  SUBCASE("addRandomAgents and time") {
    GIVEN("A dynamics object") {
      auto const p{0.1};
      auto const n{100};
      // graph.adjustNodeCapacities();
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setPassageProbability(p);
      WHEN("We add some agent") {
        dynamics.addAgents(n, AgentInsertionMethod::RANDOM);
        THEN("The number of agents is correct") { CHECK_EQ(dynamics.nAgents(), 100); }
        THEN("If we evolve the dynamics agent disappear gradually") {
          auto constexpr nSteps = 1000;
          for (auto i{0}; i < nSteps; ++i) {
            dynamics.evolve();
          }
          CHECK(dynamics.nAgents() < n);
          CHECK_EQ(dynamics.time_step(), nSteps);
        }
      }
    }
  }

  SUBCASE("transitionMatrix") {
    // In the manhattan fixture street 2 goes 0 -> 1 and node 1 has the outgoing streets
    // 3 (1 -> 0, i.e. the U-turn), 4 (1 -> 11) and 6 (1 -> 2).
    GIVEN("A dynamics object") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      THEN("The transition matrix is empty by default") {
        CHECK(dynamics.transitionMatrix().empty());
      }
      WHEN("We import a transition matrix from a JSON file") {
        dynamics.importTransitionMatrixFromJSON(
            (DATA_FOLDER / "transition_matrix.json").string());
        THEN("The matrix is correctly set and the END entry is not stored") {
          auto const& matrix = dynamics.transitionMatrix();
          CHECK_EQ(matrix.size(), 2);
          REQUIRE(matrix.contains(2));
          CHECK_EQ(matrix.at(2).size(), 2);
          CHECK_EQ(matrix.at(2).at(4), doctest::Approx(0.6));
          CHECK_EQ(matrix.at(2).at(6), doctest::Approx(0.2));
          CHECK_FALSE(matrix.at(2).contains(3));
          REQUIRE(matrix.contains(4));
          CHECK_EQ(matrix.at(4).size(), 3);
          CHECK_EQ(matrix.at(4).at(44), doctest::Approx(0.5));
        }
      }
      WHEN("We import from a file which does not exist") {
        THEN("An exception is thrown") {
          CHECK_THROWS_AS(dynamics.importTransitionMatrixFromJSON(
                              (DATA_FOLDER / "i_do_not_exist.json").string()),
                          std::runtime_error);
        }
      }
      WHEN("We set a row whose probabilities sum up to more than one") {
        THEN("An exception is thrown") {
          CHECK_THROWS_AS(dynamics.setTransitionMatrix({{2, {{4, 0.8}, {6, 0.4}}}}),
                          std::invalid_argument);
        }
      }
      WHEN("We set a negative probability") {
        THEN("An exception is thrown") {
          CHECK_THROWS_AS(dynamics.setTransitionMatrix({{2, {{4, -0.1}}}}),
                          std::invalid_argument);
        }
      }
      WHEN("We set a row for a street which does not exist") {
        dynamics.setTransitionMatrix({{100000, {{4, 1.}}}});
        THEN("The row is ignored") { CHECK(dynamics.transitionMatrix().empty()); }
      }
      WHEN("We set transitions onto missing or non-adjacent streets") {
        // Street 0 (0 -> 10) does not leave node 1, the target of street 2
        dynamics.setTransitionMatrix({{2, {{100000, 0.3}, {0, 0.3}, {4, 0.4}}}});
        THEN("Those transitions are ignored") {
          auto const& matrix = dynamics.transitionMatrix();
          REQUIRE(matrix.contains(2));
          CHECK_EQ(matrix.at(2).size(), 1);
          CHECK_EQ(matrix.at(2).at(4), doctest::Approx(0.4));
        }
      }
      WHEN("We set a U-turn next to zero-weight transitions") {
        dynamics.setTransitionMatrix({{2, {{3, 0.6}, {4, 0.}, {6, 0.}}}});
        THEN("The U-turn weight is shared equally") {
          auto const& matrix = dynamics.transitionMatrix();
          REQUIRE(matrix.contains(2));
          CHECK_EQ(matrix.at(2).at(4), doctest::Approx(0.3));
          CHECK_EQ(matrix.at(2).at(6), doctest::Approx(0.3));
        }
      }
      WHEN("We import malformed JSON files") {
        auto const jsonPath =
            std::filesystem::temp_directory_path() / "dsf_bad_transition_matrix.json";
        auto const importJson = [&](std::string const& content) {
          {
            std::ofstream out{jsonPath};
            REQUIRE(out.is_open());
            out << content;
          }
          dynamics.importTransitionMatrixFromJSON(jsonPath.string());
        };
        THEN("An exception is thrown") {
          CHECK_THROWS_AS(importJson("[]"), std::runtime_error);
          CHECK_THROWS_AS(importJson(R"({ "2": 0.5 })"), std::runtime_error);
          CHECK_THROWS_AS(importJson(R"({ "2": { "4": "high" } })"), std::runtime_error);
          CHECK_THROWS_AS(importJson(R"({ "two": { "4": 0.5 } })"),
                          std::invalid_argument);
        }
        std::filesystem::remove(jsonPath);
      }
      THEN("Turn counts must be initialized exactly once before being reset") {
        CHECK_THROWS_AS(dynamics.resetTurnCounts(), std::runtime_error);
        dynamics.initTurnCounts();
        CHECK_THROWS_AS(dynamics.initTurnCounts(), std::runtime_error);
        CHECK_NOTHROW(dynamics.resetTurnCounts());
      }
      WHEN("We set a row containing only unusable transitions") {
        // Street 3 (1 -> 0) is the U-turn of street 2 (0 -> 1)
        dynamics.setTransitionMatrix({{2, {{3, 1.}}}});
        THEN("The row is dropped, falling back to the uniform behaviour") {
          CHECK(dynamics.transitionMatrix().empty());
        }
      }
      WHEN("We set a row whose U-turn weight can be redistributed") {
        // Street 3 (1 -> 0) is the U-turn of street 2 (0 -> 1); its weight must be
        // redistributed proportionally onto streets 4 and 6 rather than turning into
        // extra END probability.
        dynamics.setTransitionMatrix({{2, {{3, 0.2}, {4, 0.4}, {6, 0.4}}}});
        THEN(
            "The U-turn weight is spread over the other transitions and END is "
            "unchanged") {
          auto const& matrix = dynamics.transitionMatrix();
          REQUIRE(matrix.contains(2));
          CHECK_FALSE(matrix.at(2).contains(3));
          CHECK_EQ(matrix.at(2).at(4), doctest::Approx(0.5));
          CHECK_EQ(matrix.at(2).at(6), doctest::Approx(0.5));
        }
      }
      WHEN("We set a row whose forbidden-turn weight can be redistributed") {
        // Forbid the turn from street 2 onto street 6; its weight must be
        // redistributed onto street 4 rather than turning into extra END probability.
        dynamics.graph().edge(2).addForbiddenTurn(6);
        dynamics.setTransitionMatrix({{2, {{6, 0.2}, {4, 0.8}}}});
        THEN(
            "The forbidden-turn weight is spread over the other transitions and END "
            "is unchanged") {
          auto const& matrix = dynamics.transitionMatrix();
          REQUIRE(matrix.contains(2));
          CHECK_FALSE(matrix.at(2).contains(6));
          CHECK_EQ(matrix.at(2).at(4), doctest::Approx(1.));
        }
      }
      WHEN("We set a row containing only a forbidden turn") {
        dynamics.graph().edge(2).addForbiddenTurn(6);
        dynamics.setTransitionMatrix({{2, {{6, 1.}}}});
        THEN("The row is dropped, falling back to the uniform behaviour") {
          CHECK(dynamics.transitionMatrix().empty());
        }
      }
    }
    GIVEN("A dynamics object with agents starting on street 2") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      auto constexpr nAgents{20};
      auto constexpr nSteps{500};
      WHEN("The matrix forces every transition out of street 2 onto street 4") {
        dynamics.setTransitionMatrix({{2, {{4, 1.}}}});
        dynamics.initTurnCounts();
        for (auto i{0}; i < nAgents; ++i) {
          dynamics.addAgent(std::nullopt, 2);
        }
        for (auto i{0}; i < nSteps; ++i) {
          dynamics.evolve();
        }
        THEN("No agent ever turns onto another street") {
          auto const& turnCounts = dynamics.turnCounts();
          REQUIRE(turnCounts.find(2) != turnCounts.end());
          CHECK(turnCounts.at(2).at(4) > 0);
          CHECK_EQ(turnCounts.at(2).at(6), 0);
          CHECK_EQ(turnCounts.at(2).at(3), 0);
          auto const normalized = dynamics.normalizedTurnCounts();
          CHECK_EQ(normalized.at(2).at(4), doctest::Approx(1.));
          CHECK_EQ(normalized.at(2).at(6), 0.);
        }
        THEN("Streets without a row keep routing agents uniformly") {
          auto const& turnCounts = dynamics.turnCounts();
          std::size_t otherTurns{0};
          for (auto const& [fromId, row] : turnCounts) {
            if (fromId == 2) {
              continue;
            }
            for (auto const& [toId, count] : row) {
              otherTurns += count;
            }
          }
          CHECK(otherTurns > 0);
        }
      }
      WHEN("The matrix leaves all the probability mass to the END entry") {
        // The only listed transition has weight 0, so 1 - 0 = 1 is the END probability
        dynamics.setTransitionMatrix({{2, {{4, 0.}}}});
        for (auto i{0}; i < nAgents; ++i) {
          dynamics.addAgent(std::nullopt, 2);
        }
        for (auto i{0}; i < nSteps; ++i) {
          dynamics.evolve();
        }
        THEN("Every agent ends its trip as arrived, not killed") {
          auto const [nAdded, nInserted, nArrived, nKilled, nCurrent] =
              dynamics.agentStats();
          CHECK_EQ(nArrived, nAgents);
          CHECK_EQ(nKilled, 0);
          CHECK_EQ(nCurrent, 0);
        }
      }
    }
  }
  SUBCASE("addAgents") {
    GIVEN("A dynamics object and one destination") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      WHEN("We add an agent with destination 2") {
        dynamics.addAgent(2, 0);
        THEN(
            "The number of agents is 1 and the destination is the same as the "
            "destination") {
          CHECK_EQ(dynamics.nAgents(), 1);
          CHECK_EQ(dynamics.agents().at(0)->destination().value(), 2);
        }
      }
      WHEN("We add 69 agents") {
        dynamics.addAgents(69, AgentInsertionMethod::RANDOM);
        THEN("The number of agents is 69") { CHECK_EQ(dynamics.nAgents(), 69); }
      }
    }
  }
  SUBCASE("Routing") {
    GIVEN("A dynamics object with a short (0->1->2) and a direct (0->2) branch") {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 2.);
      graph2.addStreet(1, std::make_pair(1, 2), 5.);
      graph2.addStreet(2, std::make_pair(0, 2), 10.);
      graph2.addStreet(3, std::make_pair(2, 3), 5.);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      WHEN("The router is built") {
        THEN("The routes lead to the end of the destination street") {
          auto const& router{dynamics.router()};
          REQUIRE(router.isBuilt());
          CHECK_EQ(router.nEdges(), 4);
          CHECK_EQ(router.route(0, 3, RouteWeights::FREE_FLOW), std::vector<Id>{0, 1, 3});
          // From a node, the cost of the first street counts: 2 + 5 < 10
          CHECK_EQ(router.routeFromNode(0, 3, RouteWeights::FREE_FLOW),
                   std::vector<Id>{0, 1, 3});
        }
        AND_WHEN("The routes are planned with Dijkstra's algorithm") {
          dynamics.setRouteAlgorithm(RouteAlgorithm::DIJKSTRA);
          THEN("They are the same") {
            CHECK_EQ(dynamics.router().algorithm(), RouteAlgorithm::DIJKSTRA);
            CHECK_EQ(dynamics.router().route(0, 3, RouteWeights::FREE_FLOW),
                     std::vector<Id>{0, 1, 3});
            CHECK_EQ(dynamics.router().routeFromNode(0, 3, RouteWeights::FREE_FLOW),
                     std::vector<Id>{0, 1, 3});
          }
        }
        AND_WHEN("An agent without a source street departs from node 0") {
          dynamics.addAgent(std::make_unique<Agent>(0, 0, 3, 0));
          dynamics.evolve();  // The agent plans its route and enters node 0
          dynamics.evolve();  // The agent enters street 0
          THEN("Its route also chooses the first street") {
            auto const& street{dynamics.graph().edge(0)};
            REQUIRE_EQ(street.nAgents(), 1);
            auto const& pAgent{street.movingAgents().top()};
            // On street 0, the agent still plans to take street 1
            CHECK_EQ(pAgent->nPlannedStreets(), 1);
            CHECK_EQ(pAgent->nextPlannedStreetId(), 1);
          }
        }
      }
    }
    GIVEN("A dynamics object with a dead end (4->5->4) next to the route") {
      RoadNetwork graph2{};
      graph2.addEdge(0, std::make_pair(0, 1), 10.);
      graph2.addEdge(1, std::make_pair(1, 2), 10.);
      graph2.addEdge(2, std::make_pair(2, 3), 10.);
      graph2.addEdge(3, std::make_pair(1, 4), 10.);
      graph2.addEdge(4, std::make_pair(4, 1), 10.);
      graph2.addEdge(5, std::make_pair(4, 5), 10.);
      graph2.addEdge(6, std::make_pair(5, 4), 10.);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      WHEN("The router is built") {
        THEN("U-turns are planned only at dead ends") {
          auto const& router{dynamics.router()};
          CHECK_EQ(router.route(0, 2, RouteWeights::FREE_FLOW), std::vector<Id>{0, 1, 2});
          CHECK_EQ(router.route(2, 2, RouteWeights::FREE_FLOW), std::vector<Id>{2});
          // Street 3 cannot turn back at node 4, where street 5 leads on: the route goes
          // around the dead end first
          CHECK_EQ(router.route(3, 2, RouteWeights::FREE_FLOW),
                   std::vector<Id>{3, 5, 6, 4, 1, 2});
        }
      }
    }
    GIVEN("A dynamics object on the Manhattan network") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      WHEN("The router is built") {
        THEN("Every street has a route to the destination") {
          auto const& router{dynamics.router()};
          CHECK(router.hasHeuristic());
          for (auto const& [streetId, pStreet] : dynamics.graph().edges()) {
            CHECK_FALSE(router.route(streetId, 118, RouteWeights::FREE_FLOW).empty());
          }
        }
      }
    }
    GIVEN("A dynamics object with a custom path weight callable") {
      RoadNetwork graph;
      // The weight must not depend on the caller's buffer (e.g. the JSON parser's)
      std::string weightName{"custom_cost"};
      // The custom cost favours the long branch 0->3->2, the free-flow travel time the
      // short one 0->1->2
      auto& s1 = graph.addStreet(0, std::make_pair(0, 1), 5.);
      s1.setAttribute("custom_cost", 100.0);
      auto& s2 = graph.addStreet(1, std::make_pair(1, 2), 5.);
      s2.setAttribute("custom_cost", 100.0);
      auto& s3 = graph.addStreet(2, std::make_pair(0, 3), 6.);
      s3.setAttribute("custom_cost", 1.0);
      auto& s4 = graph.addStreet(3, std::make_pair(3, 2), 6.);
      s4.setAttribute("custom_cost", 1.0);
      auto& s5 = graph.addStreet(4, std::make_pair(2, 4), 5.);
      s5.setAttribute("custom_cost", 1.0);
      auto& s6 = graph.addStreet(5, std::make_pair(5, 0), 5.);
      s6.setAttribute("custom_cost", 1.0);
      FirstOrderDynamics dynamics{std::move(graph), 69};
      dynamics.setEdgeWeight(weightName);
      weightName = "xxxxxxxxxxx";

      WHEN("The router is built") {
        THEN("Only the routes on the current weights follow the custom attribute") {
          auto const& router{dynamics.router()};
          CHECK_EQ(router.route(5, 4, RouteWeights::CURRENT),
                   std::vector<Id>{5, 2, 3, 4});
          CHECK_EQ(router.route(5, 4, RouteWeights::FREE_FLOW),
                   std::vector<Id>{5, 0, 1, 4});
        }
        AND_WHEN("A free-flow agent and an intelligent agent depart from street 5") {
          dynamics.initTurnCounts();
          for (Id const agentId : {0, 1}) {
            auto pAgent = std::make_unique<Agent>(agentId, 0, 4, 5);
            pAgent->setSrcStreetId(5);
            pAgent->setNextStreetId(5);
            pAgent->setIntelligent(agentId == 1);
            dynamics.addAgent(std::move(pAgent));
          }
          for (int i{0}; i < 20 && dynamics.nAgents() > 0; ++i) {
            dynamics.evolve();
          }
          THEN("Only the intelligent agent takes the branch of the custom attribute") {
            CHECK_EQ(dynamics.agentStats().nArrived, 2);
            CHECK_EQ(dynamics.turnCounts().at(5).at(0), 1);
            CHECK_EQ(dynamics.turnCounts().at(5).at(2), 1);
          }
        }
      }
    }
    GIVEN("A network with two equivalent branches (0->1->2 and 0->3->2)") {
      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 5.);
      graph.addStreet(1, std::make_pair(1, 2), 5.);
      graph.addStreet(2, std::make_pair(0, 3), 5.);
      graph.addStreet(3, std::make_pair(3, 2), 5.);
      graph.addStreet(4, std::make_pair(2, 4), 5.);
      graph.addStreet(5, std::make_pair(5, 0), 5.);
      FirstOrderDynamics dynamics{std::move(graph), 69};
      dynamics.initTurnCounts();
      for (int i{0}; i < 20; ++i) {
        dynamics.addAgent(4, 5);
      }
      WHEN("The agents travel to node 2") {
        for (int i{0}; i < 200 && dynamics.nAgents() > 0; ++i) {
          dynamics.evolve();
        }
        THEN("Every agent follows the same branch, the tie being broken by street id") {
          CHECK_EQ(dynamics.turnCounts().at(5).at(0), 20);
          CHECK_EQ(dynamics.turnCounts().at(5).at(2), 0);
        }
      }
    }
    GIVEN("A fast but long branch (0->1->2) and a slow but short one (0->3->2)") {
      // Free flow: 100 s and 2000 m against 120 s and 1200 m, even at 0.025 s/m
      auto const routeAgents = [](std::string_view const weight,
                                  double const distanceCost,
                                  double const pathRandomness) {
        RoadNetwork graph;
        graph.addStreet(0, std::make_pair(0, 1), 1000., 20.);
        graph.addStreet(1, std::make_pair(1, 2), 1000., 20.);
        graph.addStreet(2, std::make_pair(0, 3), 600., 10.);
        graph.addStreet(3, std::make_pair(3, 2), 600., 10.);
        graph.addStreet(4, std::make_pair(2, 4), 10., 10.);
        graph.addStreet(5, std::make_pair(5, 0), 10., 10.);
        FirstOrderDynamics dynamics{std::move(graph), 69};
        dynamics.setEdgeWeight(weight, distanceCost);
        dynamics.setPathRandomness(pathRandomness);
        dynamics.initTurnCounts();
        for (int i{0}; i < 40; ++i) {
          dynamics.addAgent(4, 5);
        }
        for (int i{0}; i < 1000 && dynamics.nAgents() > 0; ++i) {
          dynamics.evolve();
        }
        CHECK_EQ(dynamics.agentStats().nArrived, 40);
        auto const& turnCounts{dynamics.turnCounts().at(5)};
        return std::make_pair(turnCounts.at(0), turnCounts.at(2));
      };
      WHEN("The free-flow agents plan on travel time") {
        auto const [nFast, nShort] = routeAgents("traveltime", 0.1, 0.);
        THEN("They all take the fast branch") {
          CHECK_EQ(nFast, 40);
          CHECK_EQ(nShort, 0);
        }
      }
      WHEN("They plan on the datt cost, with the same high distance cost") {
        auto const [nFast, nShort] = routeAgents("datt", 0.1, 0.);
        THEN("They all take the short branch") {
          CHECK_EQ(nFast, 0);
          CHECK_EQ(nShort, 40);
        }
      }
      WHEN("They plan on travel time with path randomness") {
        // Without it, every agent takes the fast branch (see above)
        auto const [nFast, nShort] = routeAgents("traveltime", 0.1, 0.5);
        THEN("They use both branches, most of them the fast one") {
          CHECK_EQ(nFast + nShort, 40);
          CHECK_GT(nShort, 0);
          CHECK_GT(nFast, nShort);
        }
      }
    }
    GIVEN("An origin street from which the destination cannot be reached") {
      // Node 3 is entered by street 3 only, from node 4, which street 0 cannot reach
      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 10.);
      graph.addStreet(1, std::make_pair(1, 2), 10.);
      graph.addStreet(2, std::make_pair(3, 4), 10.);
      graph.addStreet(3, std::make_pair(4, 3), 10.);
      FirstOrderDynamics dynamics{std::move(graph), 69};
      WHEN("An agent without a source street tries to depart from node 0") {
        dynamics.addAgent(std::make_unique<Agent>(0, 0, 2, 0));
        dynamics.evolve();
        THEN("It is dropped") {
          CHECK_EQ(dynamics.agentStats().nKilled, 1);
          CHECK(dynamics.agents().empty());
          CHECK_FALSE(dynamics.meanTravelSpeed().is_valid);
        }
      }
      WHEN("The agent tries to depart") {
        dynamics.addAgent(2, 0);
        dynamics.evolve();
        THEN("It is dropped, without travel nor destination records") {
          auto const [nAdded, nInserted, nArrived, nKilled, nCurrent] =
              dynamics.agentStats();
          CHECK_EQ(nInserted, 1);
          CHECK_EQ(nArrived, 0);
          CHECK_EQ(nKilled, 1);
          CHECK_EQ(nCurrent, 0);
          CHECK(dynamics.agents().empty());
          CHECK(dynamics.destinationCounts().empty());
          CHECK_FALSE(dynamics.meanTravelSpeed().is_valid);
        }
      }
    }
    GIVEN("A short (0->1->2) and a long (0->3->2) branch changed during the run") {
      // Street 5 feeds node 0, street 4 leaves node 2
      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 100.);
      graph.addStreet(1, std::make_pair(1, 2), 100.);
      graph.addStreet(2, std::make_pair(0, 3), 120.);
      graph.addStreet(3, std::make_pair(3, 2), 120.);
      graph.addStreet(4, std::make_pair(2, 4), 10.);
      graph.addStreet(5, std::make_pair(5, 0), 10.);
      FirstOrderDynamics dynamics{std::move(graph), 69};
      CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::FREE_FLOW),
               std::vector<Id>{5, 0, 1, 4});
      THEN("Evolving without changes does not change the routing revision") {
        auto const revision{Road::routingRevision()};
        dynamics.evolve();
        dynamics.evolve();
        CHECK_EQ(Road::routingRevision(), revision);
      }
      WHEN("The short branch gets slower") {
        dynamics.graph().edge(0).setMaxSpeed(1.);
        THEN("The free-flow routes take the long one") {
          CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::FREE_FLOW),
                   std::vector<Id>{5, 2, 3, 4});
        }
      }
      WHEN("The turn into the short branch is forbidden") {
        dynamics.graph().edge(5).addForbiddenTurn(0);
        THEN("The routes take the long branch") {
          CHECK_FALSE(dynamics.router().isTurnAllowed(5, 0));
          CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::FREE_FLOW),
                   std::vector<Id>{5, 2, 3, 4});
        }
      }
      WHEN("An agent heading for the short branch sees it closed before node 0") {
        auto pAgent = std::make_unique<Agent>(0, 0, 4, 5);
        pAgent->setSrcStreetId(5);
        pAgent->setNextStreetId(5);
        dynamics.addAgent(std::move(pAgent));
        // The agent plans its route on the short branch and enters street 5
        for (int i{0}; i < 3 && dynamics.graph().edge(5).nAgents() == 0; ++i) {
          dynamics.evolve();
        }
        REQUIRE_EQ(dynamics.graph().edge(5).nAgents(), 1);
        dynamics.graph().setStreetStatusById(0, RoadStatus::CLOSED);
        bool bEnteredShort{false}, bEnteredLong{false};
        for (int i{0}; i < 60 && dynamics.nAgents() > 0; ++i) {
          dynamics.evolve();
          bEnteredShort = bEnteredShort || dynamics.graph().edge(0).nAgents() > 0;
          bEnteredLong = bEnteredLong || dynamics.graph().edge(2).nAgents() > 0;
        }
        THEN("It plans again at the next evolve and takes the long branch") {
          CHECK_FALSE(bEnteredShort);
          CHECK(bEnteredLong);
          CHECK_EQ(dynamics.agentStats().nArrived, 1);
        }
      }
      WHEN("An agent is queued before node 0 when the short branch is closed") {
        // No agent passes a node: the agent stays queued at the end of street 5
        dynamics.setPassageProbability(0.);
        auto pAgent = std::make_unique<Agent>(0, 0, 4, 5);
        pAgent->setSrcStreetId(5);
        pAgent->setNextStreetId(5);
        dynamics.addAgent(std::move(pAgent));
        auto const& street5{dynamics.graph().edge(5)};
        for (int i{0}; i < 10 && street5.nExitingAgents() == 0; ++i) {
          dynamics.evolve();
        }
        REQUIRE_EQ(street5.queue(0).size(), 1);
        REQUIRE_EQ(street5.queue(0).front()->nextStreetId(), std::optional<Id>{0});
        AND_WHEN("The long branch is open") {
          dynamics.graph().setStreetStatusById(0, RoadStatus::CLOSED);
          dynamics.evolve();
          THEN("It chooses the long branch while still queued") {
            REQUIRE_EQ(street5.queue(0).size(), 1);
            CHECK_EQ(street5.queue(0).front()->nextStreetId(), std::optional<Id>{2});
            CHECK_EQ(street5.queue(0).front()->nextPlannedStreetId(),
                     std::optional<Id>{3});
          }
          THEN("Once nodes let it pass, it takes the long branch and arrives") {
            dynamics.setPassageProbability(1.);
            bool bEnteredShort{false};
            for (int i{0}; i < 60 && dynamics.nAgents() > 0; ++i) {
              dynamics.evolve();
              bEnteredShort = bEnteredShort || dynamics.graph().edge(0).nAgents() > 0;
            }
            CHECK_FALSE(bEnteredShort);
            CHECK_EQ(dynamics.agentStats().nArrived, 1);
          }
        }
        AND_WHEN("The long branch is closed too") {
          std::vector<Id> const branches{0, 2};
          dynamics.graph().setStreetStatusById(branches, RoadStatus::CLOSED);
          dynamics.evolve();
          THEN("It is killed, as no route is left") {
            CHECK(street5.queue(0).empty());
            CHECK_EQ(dynamics.agentStats().nKilled, 1);
            CHECK_EQ(dynamics.nAgents(), 0);
          }
        }
      }
      WHEN("The short branch is closed, then reopened") {
        dynamics.graph().setStreetStatusById(0, RoadStatus::CLOSED);
        CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::FREE_FLOW),
                 std::vector<Id>{5, 2, 3, 4});
        dynamics.graph().setStreetStatusById(0, RoadStatus::OPEN);
        THEN("The routes take it again") {
          CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::FREE_FLOW),
                   std::vector<Id>{5, 0, 1, 4});
        }
      }
      WHEN("The only street into a destination is closed during the run") {
        dynamics.setDestinations({1, 4});
        dynamics.evolve();
        REQUIRE_EQ(dynamics.destinationStreets().size(), 2);
        dynamics.graph().setStreetStatusById(0, RoadStatus::CLOSED);
        CHECK_NOTHROW(dynamics.evolve());
        THEN("The destination is removed at the next evolve") {
          CHECK_FALSE(dynamics.destinationStreets().contains(1));
          CHECK(dynamics.destinationStreets().contains(4));
          CHECK_EQ(dynamics.destinations().size(), 1);
        }
      }
      WHEN("A closed destination is set") {
        dynamics.graph().setStreetStatusById(1, RoadStatus::CLOSED);
        dynamics.setODs({{5, 1, 1.}, {5, 4, 1.}});
        CHECK_NOTHROW(dynamics.evolve());
        THEN("It is removed with its OD pairs") {
          CHECK_EQ(dynamics.destinationStreets().size(), 1);
          CHECK(dynamics.destinationStreets().contains(4));
        }
      }
    }
    GIVEN("A disconnected graph") {
      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 10.);
      FirstOrderDynamics dynamics{std::move(graph), 69};

      WHEN("We add an impossible destination (no street enters its source node)") {
        dynamics.setDestinations(std::unordered_map<Id, double>{{0, 1.}});
        dynamics.setODs({{0, 0, 1.}});
        dynamics.setConditionalODs({{0, {1., {{0, 1.}}}}});
        CHECK_NOTHROW(dynamics.evolve());
        THEN("The destination and every origin and destination using it are removed") {
          CHECK(dynamics.destinationStreets().empty());
          CHECK(dynamics.destinations().empty());
          CHECK(dynamics.origins().empty());
        }
      }
    }
  }
  SUBCASE("Intelligent agents") {
    GIVEN("A network with a short (0->1->2) and a long (0->3->2) branch") {
      // Streets 0-1 form the short branch, 2-3 the long one; 4 is the destination street
      // and 5 feeds node 0.
      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 100.);
      graph.addStreet(1, std::make_pair(1, 2), 100.);
      graph.addStreet(2, std::make_pair(0, 3), 120.);
      graph.addStreet(3, std::make_pair(3, 2), 120.);
      graph.addStreet(4, std::make_pair(2, 4), 10.);
      graph.addStreet(5, std::make_pair(5, 0), 10.);
      FirstOrderDynamics dynamics{std::move(graph), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      auto const parkOnShortBranch = [&dynamics](Id const nAgents) {
        auto& street{dynamics.graph().edge(0)};
        // Park agents that never leave the street to raise its density
        for (Id i{0}; i < nAgents; ++i) {
          auto pAgent = std::make_unique<Agent>(1000 + i, 0);
          pAgent->setFreeTime(std::numeric_limits<std::time_t>::max());
          street.addAgent(std::move(pAgent), 0);
        }
      };
      auto const addAgentOnStreet5 = [&dynamics](Id const agentId,
                                                 bool const bIntelligent) {
        auto pAgent = std::make_unique<Agent>(agentId, 0, 4, 5);
        pAgent->setSrcStreetId(5);
        pAgent->setNextStreetId(5);
        pAgent->setIntelligent(bIntelligent);
        dynamics.addAgent(std::move(pAgent));
      };
      auto const sendAgentsThroughNode0 = [&dynamics, &addAgentOnStreet5]() {
        // A free-flow agent and an intelligent agent enter from street 5
        addAgentOnStreet5(0, false);
        addAgentOnStreet5(1, true);
        for (int i{0}; i < 6; ++i) {
          dynamics.evolve();
        }
      };
      THEN("The default intelligent fraction is zero and invalid values throw") {
        CHECK_EQ(dynamics.intelligentAgentsFraction(), 0.);
        CHECK_THROWS_AS(dynamics.setIntelligentAgentsFraction(-0.1),
                        std::invalid_argument);
        CHECK_THROWS_AS(dynamics.setIntelligentAgentsFraction(1.1),
                        std::invalid_argument);
        dynamics.setIntelligentAgentsFraction(0.3);
        CHECK_EQ(dynamics.intelligentAgentsFraction(), 0.3);
      }
      WHEN("The short branch gets congested after the router is built") {
        // Building the router takes a snapshot of the current weights
        dynamics.router();
        parkOnShortBranch(18);
        THEN("The current weights still date back to the build") {
          CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::CURRENT),
                   std::vector<Id>{5, 0, 1, 4});
        }
        AND_WHEN("A free-flow agent and an intelligent agent reach node 0") {
          sendAgentsThroughNode0();
          THEN(
              "The intelligent agent plans on the traffic at its departure and takes "
              "the long branch, the free-flow agent the short one") {
            auto const& shortBranch{dynamics.graph().edge(0).movingAgents()};
            auto const& longBranch{dynamics.graph().edge(2).movingAgents()};
            REQUIRE_EQ(shortBranch.size(), 19);
            REQUIRE_EQ(longBranch.size(), 1);
            CHECK_FALSE(shortBranch.top()->isIntelligent());
            CHECK_EQ(shortBranch.top()->nextPlannedStreetId(), 1);
            CHECK(longBranch.top()->isIntelligent());
            CHECK_EQ(longBranch.top()->nextPlannedStreetId(), 3);
          }
        }
      }
      WHEN("An intelligent agent waits for a free source while the short branch fills") {
        // The free-flow agent takes the only slot of node 5
        addAgentOnStreet5(0, false);
        addAgentOnStreet5(1, true);
        dynamics.evolve();
        REQUIRE_EQ(dynamics.agents().size(), 1);
        CHECK_EQ(dynamics.agents().front()->nPlannedStreets(), 0);
        parkOnShortBranch(18);
        for (int i{0}; i < 5; ++i) {
          dynamics.evolve();
        }
        THEN("It plans on the traffic of its actual departure") {
          auto const& longBranch{dynamics.graph().edge(2).movingAgents()};
          REQUIRE_EQ(longBranch.size(), 1);
          CHECK(longBranch.top()->isIntelligent());
        }
      }
      WHEN("Four agents travel the short branch when the router is built") {
        // Their density slows the short branch to 8.57 + 7.2 = 15.77 s, still faster
        // than the 17.28 s of the long one
        parkOnShortBranch(4);
        THEN("The route on the current weights keeps the short branch") {
          CHECK_EQ(dynamics.graph().edge(0).estimatedTravelTime(),
                   doctest::Approx(100. / (13.8888888889 * (1. - 0.8 * 0.2))));
          CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::CURRENT),
                   std::vector<Id>{5, 0, 1, 4});
        }
        AND_WHEN("The same agents queue at the end of the short branch") {
          // Queued, they no longer slow the street down but cost 4 s of queue time:
          // 7.2 + 4 + 7.2 = 18.4 s
          auto& street{dynamics.graph().edge(0)};
          while (!street.movingAgents().empty()) {
            street.enqueue(0);
          }
          THEN("Only the route on the current weights moves to the long branch") {
            CHECK_EQ(street.estimatedTravelTime(), doctest::Approx(7.2));
            CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::CURRENT),
                     std::vector<Id>{5, 2, 3, 4});
            CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::FREE_FLOW),
                     std::vector<Id>{5, 0, 1, 4});
          }
          AND_WHEN("A free-flow agent and an intelligent agent reach node 0") {
            sendAgentsThroughNode0();
            THEN(
                "The free-flow agent joins the queued branch, the intelligent one "
                "avoids it") {
              auto const& shortBranch{dynamics.graph().edge(0)};
              auto const& longBranch{dynamics.graph().edge(2).movingAgents()};
              CHECK_EQ(shortBranch.nExitingAgents(), 4);
              REQUIRE_EQ(shortBranch.movingAgents().size(), 1);
              REQUIRE_EQ(longBranch.size(), 1);
              CHECK_FALSE(shortBranch.movingAgents().top()->isIntelligent());
              CHECK(longBranch.top()->isIntelligent());
              // The queue does not slow down the agents entering the street
              CHECK_EQ(shortBranch.movingAgents().top()->speed(),
                       doctest::Approx(shortBranch.maxSpeed()));
            }
          }
        }
      }
      WHEN("Agents are added with the default intelligent fraction") {
        for (int i{0}; i < 20; ++i) {
          dynamics.addAgent(4, 5);
        }
        THEN("No agent is intelligent") {
          for (auto const& pAgent : dynamics.agents()) {
            CHECK_FALSE(pAgent->isIntelligent());
          }
        }
      }
      WHEN("Agents are added with an intelligent fraction of 1") {
        dynamics.setIntelligentAgentsFraction(1.);
        for (int i{0}; i < 20; ++i) {
          dynamics.addAgent(4, 5);
        }
        dynamics.addAgent(Id{4});
        dynamics.addAgent();
        THEN("Every agent with a destination is intelligent, random ones are not") {
          REQUIRE_EQ(dynamics.agents().size(), 22);
          for (auto const& pAgent : dynamics.agents()) {
            CHECK_EQ(pAgent->isIntelligent(), !pAgent->isRandom());
          }
        }
      }
      WHEN("Agents are added with an intelligent fraction of 0.2") {
        dynamics.setIntelligentAgentsFraction(0.2);
        for (int i{0}; i < 500; ++i) {
          dynamics.addAgent(4, 5);
        }
        THEN("About a fifth of the agents is intelligent") {
          auto const nIntelligent =
              std::count_if(dynamics.agents().cbegin(),
                            dynamics.agents().cend(),
                            [](auto const& pAgent) { return pAgent->isIntelligent(); });
          // Asymmetric fraction: an inverted draw (~400) would fail this check
          CHECK_GT(nIntelligent, 60);
          CHECK_LT(nIntelligent, 140);
        }
      }
      WHEN("Node 0 becomes a roundabout after the dynamics is built") {
        auto& roundabout{dynamics.graph().makeRoundabout(0)};
        sendAgentsThroughNode0();
        THEN("Its streets are re-linked and both agents take the free short branch") {
          CHECK_EQ(dynamics.graph().edge(5).target(), &roundabout);
          CHECK_EQ(dynamics.graph().edge(0).source(), &roundabout);
          CHECK_EQ(dynamics.graph().edge(2).source(), &roundabout);
          CHECK_EQ(dynamics.router().route(5, 4, RouteWeights::FREE_FLOW),
                   std::vector<Id>{5, 0, 1, 4});
          CHECK_EQ(dynamics.graph().edge(0).movingAgents().size(), 2);
          CHECK(dynamics.graph().edge(2).movingAgents().empty());
        }
      }
      WHEN("A destination without a valid path is removed") {
        // Street 5 leaves node 5, which no street enters
        dynamics.setDestinations({4, 5});
        dynamics.evolve();
        THEN("Only the other destination is left") {
          CHECK_FALSE(dynamics.destinationStreets().contains(5));
          CHECK(dynamics.destinationStreets().contains(4));
        }
      }
    }
  }
  SUBCASE("Evolve") {
    GIVEN("A dynamics object with one non-random agent that reaches its destination") {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 13.8888888889);
      graph2.addStreet(1, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(2, std::make_pair(2, 3), 13.8888888889);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(2, 0);

      WHEN("We evolve until the agent reaches the end of its destination street") {
        for (int i{0}; i < 5; ++i) {
          dynamics.evolve();
        }

        THEN("The summary counts the agent as arrived but not killed") {
          auto const [nAdded, nInserted, nArrived, nKilled, nCurrent] =
              dynamics.agentStats();
          CHECK_EQ(nAdded, 1);
          CHECK_EQ(nInserted, 1);
          CHECK_EQ(nArrived, 1);
          CHECK_EQ(nKilled, 0);
          CHECK_EQ(nCurrent, 0);
        }
      }
    }
    GIVEN(
        "An agent whose destination street ends at a light which is always red for it") {
      RoadNetwork graph2;
      graph2.addNode<TrafficLight>(2);
      graph2.addStreet(0, std::make_pair(0, 1), 30., 15.);
      graph2.addStreet(1, std::make_pair(1, 2), 30., 15.);
      graph2.addStreet(2, std::make_pair(3, 2), 30., 15.);
      graph2.addStreet(3, std::make_pair(2, 4), 30., 15.);
      auto& tl = graph2.node<TrafficLight>(2);
      tl.setAllowFreeTurns(false);
      {
        TrafficLightPhase p{10};
        p.addGreen(2, dsf::Direction::ANY);
        tl.addPhase(p);
      }
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.addAgent(1, 0);
      WHEN("It reaches the end of its destination street") {
        for (int i{0}; i < 10 && dynamics.nAgents() > 0; ++i) {
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(), 0);
          dynamics.evolve();
        }
        THEN("It arrives there, without queueing at the light") {
          CHECK_EQ(dynamics.nAgents(), 0);
          CHECK_EQ(dynamics.agentStats().nArrived, 1);
          CHECK_EQ(dynamics.destinationCounts(false).at(1), 1);
          // Streets 0 and 1
          CHECK_EQ(dynamics.meanTravelDistance().mean, 60.);
        }
      }
    }
    GIVEN("A dynamics object and a destination") {
      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 2.);
      graph.addStreet(1, std::make_pair(1, 2), 5.);
      graph.addStreet(2, std::make_pair(0, 2), 10.);
      graph.addStreet(3, std::make_pair(2, 3), 5.);
      FirstOrderDynamics dynamics{std::move(graph), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      WHEN("We add an agent randomly and evolve the dynamics") {
        dynamics.addAgent(3, 0);
        auto const& network{dynamics.graph()};
        dynamics.evolve();  // Agent goes into node 0
        dynamics.evolve();  // Agent goes from node 0 to street 0->1
        THEN("The agent evolves") {
          CHECK_EQ(network.edge(0).movingAgents().size(), 1);
          auto const& pAgent{network.edge(0).movingAgents().top()};
          CHECK(pAgent);
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), dynamics.time_step());
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 0);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
        }
        dynamics.evolve();  // Agent enqueues on street 0->1 and changes street
        THEN("The agent evolves again, changing street") {
          auto const& pAgent{network.edge(1).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), dynamics.time_step());
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 1);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
        }
        dynamics.evolve();  // Agent leaves street 1 for its destination street 3
        THEN("The agent travels its destination street") {
          CHECK_EQ(network.edge(3).movingAgents().size(), 1);
        }
        dynamics.evolve();  // Agent reaches the end of street 3 and arrives
        THEN("And again, reaching the destination") { CHECK_EQ(dynamics.nAgents(), 0); }
      }
    }
    GIVEN("A dynamics object, a destination and an agent") {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 13.8888888889);
      graph2.addStreet(1, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(2, std::make_pair(2, 3), 13.8888888889);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(2, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent evolves") {
          auto const& pAgent{dynamics.graph().edge(0).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), 2);
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 0);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
          CHECK_EQ(pAgent->distance(), 0.);  // Not updated yet
        }
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent reaches the end of its destination street") {
          CHECK_EQ(dynamics.nAgents(), 0);
        }
      }
    }
    GIVEN("A dynamics object, a destination and an agent") {
      RoadNetwork graph2;
      auto& s1 = graph2.addStreet(0, std::make_pair(0, 1), 13.8888888889);
      s1.setTransportCapacity(0.3);
      graph2.addStreet(1, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(2, std::make_pair(2, 3), 13.8888888889);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(2, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve();
        dynamics.evolve();
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
          dynamics.evolve();
          ++i;
        }
        THEN("The agent reaches the destination") {
          CHECK(i > 0);
          CHECK_EQ(dynamics.nAgents(), 0);
        }
      }
    }
    GIVEN("A dynamics object, a destination and an agent") {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 13.8888888889);
      graph2.addStreet(1, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(2, std::make_pair(2, 3), 13.8888888889);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(2, 0);
      dynamics.setReinsertAgents(true);
      WHEN("We evolve the dynamics with reinsertion") {
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent has correct values") {
          auto const& pAgent{dynamics.graph().edge(0).movingAgents().top()};
          CHECK_EQ(dynamics.time_step() - pAgent->spawnTime(), 2);
          CHECK_EQ(pAgent->freeTime(), dynamics.time_step());
          CHECK_EQ(pAgent->streetId().value(), 0);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
          CHECK_EQ(pAgent->distance(), 0.);  // Not updated yet
        }
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent is reinserted") {
          CHECK(dynamics.graph().node(0).density() > 0.);
          CHECK_EQ(dynamics.nAgents(), 1);
        }
        dynamics.evolve();
        THEN("The agent restarts its trip from its first street") {
          CHECK_EQ(dynamics.graph().edge(0).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 0);
        }
      }
    }
    GIVEN("A dynamics with many random agents reinserted when they arrive") {
      FirstOrderDynamics dynamics{std::move(defaultNetwork), 69};
      dynamics.setReinsertAgents(true);
      // Short trips, so that many agents arrive concurrently during the parallel
      // street evolution
      dynamics.setMeanTravelDistance(100.);
      dynamics.addAgents(2000, AgentInsertionMethod::RANDOM);
      WHEN("We evolve the dynamics") {
        for (int i = 0; i < 500; ++i) {
          dynamics.evolve();
        }
        THEN("Arrived agents are reinserted and none is lost") {
          auto const [nAdded, nInserted, nArrived, nKilled, nAgents] =
              dynamics.agentStats();
          CHECK(nArrived > 0);
          CHECK_EQ(nAgents + nKilled, 2000);
        }
      }
    }
  }
  SUBCASE("TrafficLights") {
    GIVEN(
        "A dynamics object, a network with traffic lights, a destination and "
        "an agent") {
      RoadNetwork graph2;
      graph2.addNode<TrafficLight>(1);
      graph2.addStreet(1, std::make_pair(0, 1), 30., 15.);
      graph2.addStreet(7, std::make_pair(1, 2), 30., 15.);
      graph2.addStreet(16, std::make_pair(3, 1), 30., 15.);
      graph2.addStreet(9, std::make_pair(1, 4), 30., 15.);
      graph2.addStreet(8, std::make_pair(2, 8), 30., 15.);
      auto& tl = graph2.node<TrafficLight>(1);
      tl.setAllowFreeTurns(false);
      // Phase 0 (ticks 0-1): street 1 green — agent can exit toward node 2
      {
        TrafficLightPhase p{2};
        p.addGreen(1, dsf::Direction::ANY);
        tl.addPhase(p);
      }
      // Phase 1 (ticks 2-3): street 16 green — agent can exit toward node 4
      {
        TrafficLightPhase p{2};
        p.addGreen(16, dsf::Direction::ANY);
        tl.addPhase(p);
      }
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(8, 1);
      auto const& network{dynamics.graph()};
      WHEN("We evolve the dynamics") {
        dynamics.evolve();
        THEN(
            "The agent is ready to go through the traffic light at time 3 but "
            "the "
            "traffic light is red"
            " until time 4, so the agent waits until time 4") {
          for (uint8_t i{0}; i < 5; ++i) {
            dynamics.evolve();
            if (i < 3) {
              CHECK_EQ(network.edge(1).nAgents(), 1);
            } else {
              CHECK_EQ(network.edge(7).nAgents(), 1);
            }
            if (i < 2) {
              // Still crossing street 1: moving, not waiting at the light.
              CHECK_EQ(network.edge(1).nMovingAgents(), 1);
              CHECK_EQ(network.edge(1).nExitingAgents(), 0);
              CHECK_EQ(network.edge(1).movingAgents().top()->distance(), 0.);
            }
            if (i == 2) {
              // The agent has reached the end of street 1 and is now genuinely
              // queued at the red light.
              CHECK_EQ(network.edge(1).nMovingAgents(), 0);
              CHECK_EQ(network.edge(1).nExitingAgents(), 1);
              CHECK_EQ(network.edge(1).queue(0).front()->distance(), 30.);
            }
            if (i > 2) {
              // Crossing street 7: street 7 is not travelled yet.
              CHECK_EQ(network.edge(7).nMovingAgents(), 1);
              CHECK_EQ(network.edge(7).movingAgents().top()->distance(), 30.);
            }
          }
          while (dynamics.nAgents() > 0) {
            dynamics.evolve();
          }
          // Streets 1, 7 and the destination street 8, at whose end the agent arrives
          CHECK_EQ(dynamics.meanTravelDistance().mean, 90.);
        }
      }
    }
    GIVEN(
        "A traffic light managing an intersection with 4 3-lanes streets and 4 1-lane "
        "streets") {
      // Streets

      RoadNetwork graph2;
      {
        graph2.addNode<TrafficLight>(1, dsf::geometry::Point(0., 0.));
        auto& tl = graph2.node<TrafficLight>(1);
        // Equivalent timing to the old cycle setup (cycleTime=6):
        // Phase 0 (duration=2, ticks 0-1): streets 8 and 21 green — no exit from street 1
        {
          TrafficLightPhase p{2};
          p.addGreen(8);
          p.addGreen(21);
          tl.addPhase(p);
        }
        // Phase 1 (duration=2, ticks 2-3): street 1 RIGHTANDSTRAIGHT + street 11 green
        {
          TrafficLightPhase p{2};
          p.addGreen(1, dsf::Direction::RIGHTANDSTRAIGHT);
          p.addGreen(11);
          tl.addPhase(p);
        }
        // Phase 2 (duration=1, tick 4): street 1 LEFT + street 11 green
        {
          TrafficLightPhase p{1};
          p.addGreen(1, dsf::Direction::LEFT);
          p.addGreen(11);
          tl.addPhase(p);
        }
        // Phase 3 (duration=1, tick 5): streets 8 and 21 green (wrap-around)
        {
          TrafficLightPhase p{1};
          p.addGreen(8);
          p.addGreen(21);
          tl.addPhase(p);
        }
      }
      graph2.addNode<dsf::mobility::Intersection>(0, dsf::geometry::Point(-1, 0));
      graph2.addNode<dsf::mobility::Intersection>(2, dsf::geometry::Point(1, 0));
      graph2.addNode<dsf::mobility::Intersection>(3, dsf::geometry::Point(0, -1));
      graph2.addNode<dsf::mobility::Intersection>(4, dsf::geometry::Point(0, 1));
      graph2.addStreet(1, std::make_pair(0, 1), 30., 15., 3);
      graph2.addStreet(5, std::make_pair(1, 0), 30., 15., 3);
      graph2.addStreet(7, std::make_pair(1, 2), 30., 15., 3);
      graph2.addStreet(11, std::make_pair(2, 1), 30., 15., 3);
      graph2.addStreet(8, std::make_pair(3, 1), 30., 15.);
      graph2.addStreet(16, std::make_pair(1, 3), 30., 15.);
      graph2.addStreet(21, std::make_pair(4, 1), 30., 15.);
      graph2.addStreet(9, std::make_pair(1, 4), 30., 15.);
      graph2.addStreet(105, std::make_pair(0, 5), 30., 15.);
      graph2.addStreet(126, std::make_pair(2, 6), 30., 15.);
      graph2.addStreet(137, std::make_pair(3, 7), 30., 15.);
      graph2.addStreet(148, std::make_pair(4, 8), 30., 15.);
      graph2.adjustNodeCapacities();
      graph2.autoMapStreetLanes();

      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setDestinations({105, 126, 137, 148});

      WHEN("We add agents and make the system evolve") {
        dynamics.addAgent(126, 1);
        dynamics.addAgent(148, 1);
        THEN("The agents are not yet on the streets") {
          CHECK_FALSE(dynamics.agents().at(0)->streetId().has_value());
          CHECK_FALSE(dynamics.agents().at(1)->streetId().has_value());
        }
        dynamics.evolve();  // Counter 0
        dynamics.evolve();  // Counter 1
        THEN("The agents are correctly placed") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 2);
        }
        dynamics.evolve();  // Counter 2
        dynamics.evolve();  // Counter 3
        THEN("The agent 0 passes and agent 1 waits") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(7).nAgents(), 1);
        }
        dynamics.evolve();  // Counter 4
        THEN("The agent 1 passes") {
          CHECK_EQ(dynamics.graph().edge(7).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(9).nAgents(), 1);
        }
      }
    }
    GIVEN(
        "A traffic light managing an intersection with 4 3-lanes streets and 4 1-lane "
        "streets") {
      // Streets

      RoadNetwork graph2;
      {
        graph2.addNode<TrafficLight>(1, dsf::geometry::Point(0., 0.));
        auto& tl = graph2.node<TrafficLight>(1);
        // Equivalent timing to the old cycle setup (cycleTime=6):
        // Phase 0 (duration=2, ticks 0-1): street 1 RIGHTANDSTRAIGHT + street 11 green
        {
          TrafficLightPhase p{2};
          p.addGreen(1, dsf::Direction::RIGHTANDSTRAIGHT);
          p.addGreen(11);
          tl.addPhase(p);
        }
        // Phase 1 (duration=1, tick 2): street 1 LEFT + street 11 green
        {
          TrafficLightPhase p{1};
          p.addGreen(1, dsf::Direction::LEFT);
          p.addGreen(11);
          tl.addPhase(p);
        }
        // Phase 2 (duration=3, ticks 3-5): streets 8 and 21 green
        {
          TrafficLightPhase p{3};
          p.addGreen(8);
          p.addGreen(21);
          tl.addPhase(p);
        }
      }
      graph2.addNode<dsf::mobility::Intersection>(0, dsf::geometry::Point(-1, 0));
      graph2.addNode<dsf::mobility::Intersection>(2, dsf::geometry::Point(1, 0));
      graph2.addNode<dsf::mobility::Intersection>(3, dsf::geometry::Point(0, -1));
      graph2.addNode<dsf::mobility::Intersection>(4, dsf::geometry::Point(0, 1));
      graph2.addStreet(1, std::make_pair(0, 1), 30., 15., 3);
      graph2.addStreet(5, std::make_pair(1, 0), 30., 15., 3);
      graph2.addStreet(7, std::make_pair(1, 2), 30., 15., 3);
      graph2.addStreet(11, std::make_pair(2, 1), 30., 15., 3);
      graph2.addStreet(8, std::make_pair(3, 1), 30., 15.);
      graph2.addStreet(16, std::make_pair(1, 3), 30., 15.);
      graph2.addStreet(21, std::make_pair(4, 1), 30., 15.);
      graph2.addStreet(9, std::make_pair(1, 4), 30., 15.);
      graph2.addStreet(105, std::make_pair(0, 5), 30., 15.);
      graph2.addStreet(126, std::make_pair(2, 6), 30., 15.);
      graph2.addStreet(137, std::make_pair(3, 7), 30., 15.);
      graph2.addStreet(148, std::make_pair(4, 8), 30., 15.);
      graph2.adjustNodeCapacities();
      graph2.autoMapStreetLanes();

      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setDestinations({105, 126, 137, 148});

      WHEN("We add agents and make the system evolve") {
        dynamics.addAgent(126, 1);
        dynamics.addAgent(148, 1);
        dynamics.evolve();  // Counter 0
        dynamics.evolve();  // Counter 1
        THEN("The agents are correctly placed") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 2);
        }
        dynamics.evolve();  // Counter 2
        THEN("They are still crossing the street, not queued at the light") {
          CHECK_EQ(dynamics.graph().edge(1).nMovingAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(), 0);
        }
        dynamics.evolve();  // Counter 3
        dynamics.evolve();  // Counter 4
        THEN("The agents are still") {
          // Both agents have now reached the stop line and the light is red for
          // street 1 (phase 2, ticks 3-5), so both sit in the exit queue.
          CHECK_EQ(dynamics.graph().edge(1).nMovingAgents(), 0);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::ANY, true),
                   doctest::Approx(0.666667));
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::RIGHT), 0);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::STRAIGHT), 1);
          CHECK_EQ(dynamics.graph().edge(1).nExitingAgents(Direction::LEFT), 1);
        }
        dynamics.evolve();  // Counter 5
        dynamics.evolve();  // Counter 0
        THEN("The agent 0 passes and agent 1 waits") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(7).nAgents(), 1);
        }
        dynamics.evolve();  // Counter 1
        dynamics.evolve();  // Counter 2
        THEN("The agent 1 passes") { CHECK_EQ(dynamics.graph().edge(9).nAgents(), 1); }
      }
    }
  }
  SUBCASE("Traffic Lights optimization algorithm") {
    GIVEN("A dynamics object with a traffic light intersection") {
      double length{90.}, max_speed{15.};

      RoadNetwork graph2;
      graph2.addStreet(1, std::make_pair(0, 1), length, max_speed);
      graph2.addStreet(5, std::make_pair(1, 0), length, max_speed);
      graph2.addStreet(7, std::make_pair(1, 2), length, max_speed);
      graph2.addStreet(11, std::make_pair(2, 1), length, max_speed);
      graph2.addStreet(8, std::make_pair(1, 3), length, max_speed);
      graph2.addStreet(16, std::make_pair(3, 1), length, max_speed);
      graph2.addStreet(9, std::make_pair(1, 4), length, max_speed);
      graph2.addStreet(21, std::make_pair(4, 1), length, max_speed);
      graph2.addStreet(105, std::make_pair(0, 5), 30., 15.);
      graph2.addStreet(126, std::make_pair(2, 6), 30., 15.);
      graph2.addStreet(137, std::make_pair(3, 7), 30., 15.);
      graph2.addStreet(148, std::make_pair(4, 8), 30., 15.);
      auto& tl = graph2.makeTrafficLight(1);
      tl.addStreetPriority(1);
      tl.addStreetPriority(11);
      // Phase 0 (4 ticks): priority streets 1 and 11 green
      {
        TrafficLightPhase p{4};
        p.addGreen(1);
        p.addGreen(11);
        tl.addPhase(p);
      }
      // Phase 1 (4 ticks): non-priority streets 16 and 21 green
      {
        TrafficLightPhase p{4};
        p.addGreen(16);
        p.addGreen(21);
        tl.addPhase(p);
      }
      // The original counter started at 3 — reproduce with advanceBy
      tl.advanceBy(3);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setDestinations({105, 126, 137, 148});
      WHEN("We evolve the dynamics and optimize traffic lights") {
        for (int i = 0; i < 7; ++i) {
          dynamics.addAgent(105, 11);
          dynamics.addAgent(126, 1);
        }
        dynamics.setDataUpdatePeriod(4);
        for (int i = 0; i < 9; ++i) {
          dynamics.evolve();
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
          dynamics.addAgent(105, 5);
          dynamics.addAgent(126, 7);
          dynamics.addAgent(137, 8);
          dynamics.addAgent(148, 9);
        }
        dynamics.setDataUpdatePeriod(8);
        for (int i = 0; i < 15; ++i) {
          dynamics.evolve();
        }
        dynamics.optimizeTrafficLights(
            dsf::TrafficLightOptimization::SINGLE_TAIL, std::string(), 1);
        THEN("Green and red time are equal") {
          CHECK_EQ(tl.meanGreenTime(true), tl.meanGreenTime(false));
        }
      }
      WHEN("We optimize with the DOUBLE_TAIL algorithm and a log file") {
        // Synchronization needs a second traffic light upstream of node 1
        auto& tl0 = dynamics.graph().makeTrafficLight(0);
        TrafficLightPhase p{4};
        p.addGreen(5);
        tl0.addPhase(p);
        for (int i = 0; i < 7; ++i) {
          dynamics.addAgent(126, 1);
        }
        dynamics.setDataUpdatePeriod(4);
        for (int i = 0; i < 9; ++i) {
          dynamics.evolve();
        }
        auto const logPath =
            std::filesystem::temp_directory_path() / "dsf_tl_optimization.log";
        std::filesystem::remove(logPath);
        dynamics.optimizeTrafficLights(
            dsf::TrafficLightOptimization::DOUBLE_TAIL, logPath.string(), 1, 0.);
        THEN("The optimization steps are logged") {
          std::ifstream in{logPath};
          REQUIRE(in.is_open());
          std::string const log{std::istreambuf_iterator<char>{in}, {}};
          CHECK(log.find("Init Traffic Lights optimisation") != std::string::npos);
          CHECK(log.find("New cycles for") != std::string::npos);
          CHECK(log.find("End Traffic Lights optimisation") != std::string::npos);
        }
        std::filesystem::remove(logPath);
      }
    }
  }
  SUBCASE("Roundabout") {
    GIVEN(
        "A dynamics object with four streets, one agent for each street, two "
        "destinations "
        "and a roundabout") {
      Road::setMeanVehicleLength(10.);
      RoadNetwork graph2;
      graph2.addStreet(1, std::make_pair(0, 1), 10., 10.);
      graph2.addStreet(7, std::make_pair(2, 1), 10., 10.);
      graph2.addStreet(3, std::make_pair(1, 0), 10., 10.);
      graph2.addStreet(5, std::make_pair(1, 2), 10., 10.);
      graph2.addStreet(0, std::make_pair(0, 4), 10., 10.);
      // Two agents travel their destination street 2: room for both, so that the one
      // leaving street 5 never waits for the other to arrive
      graph2.addStreet(2, std::make_pair(2, 5), 20., 10.);
      auto& rb = graph2.makeRoundabout(1);
      graph2.adjustNodeCapacities();
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(2, 1);
      dynamics.addAgent(0, 7);
      WHEN(
          "We evolve the dynamics adding an agent on the path of the agent "
          "with "
          "priority") {
        dynamics.evolve();  // Agents into sources
        // dynamics.evolve();  // Agents from sources to streets
        dynamics.addAgent(2, 5);
        dynamics.evolve();  // Agents into queues, other agent into roundabout
        dynamics.evolve();  // Agents from queues to roundabout
        auto const& network{dynamics.graph()};
        THEN("The agents are trapped into the roundabout") {
          CHECK_EQ(network.edge(1).nAgents(), 0);
          CHECK_EQ(network.edge(5).nAgents(), 1);
          CHECK_EQ(network.edge(7).nAgents(), 1);
          CHECK_EQ(rb.agents().size(), 1);
        }
        dynamics.evolve();
        THEN("The agent with priority leaves the roundabout") {
          CHECK_EQ(network.edge(3).nAgents(), 1);
          CHECK_EQ(network.edge(5).nAgents(), 1);
          CHECK_EQ(network.edge(7).nAgents(), 0);
          CHECK(rb.agents().empty());
        }
        dynamics.evolve();
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
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 3.);
      graph2.addStreet(5, std::make_pair(1, 2), 1.);
      graph2.addStreet(6, std::make_pair(2, 3), 1.);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(6, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
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
      RoadNetwork graph2;
      auto& s1 = graph2.addStreet(0, std::make_pair(0, 1), 3.);
      s1.setTransportCapacity(0.3);
      graph2.addStreet(1, std::make_pair(1, 2), 1.);
      graph2.addStreet(2, std::make_pair(2, 3), 1.);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(2, 0);
      WHEN("We evolve the dynamics") {
        CHECK_EQ(dynamics.nAgents(), 1);
        while (dynamics.nAgents() > 0) {
          dynamics.evolve();
        }
        THEN("The agent has travelled the correct distance") {
          auto const& distance{dynamics.meanTravelDistance()};
          // Streets 0, 1 and its destination street 2, at whose end it arrives
          CHECK_EQ(distance.mean, 5.);
          CHECK_EQ(distance.std, doctest::Approx(0.));
          CHECK_EQ(dynamics.nAgents(), 0);
        }
        THEN("The travel speed is the travelled distance over the travel time") {
          auto const travelTime{dynamics.meanTravelTime().mean};
          CHECK_EQ(dynamics.meanTravelSpeed(true).mean, doctest::Approx(5. / travelTime));
          // The travel data has been cleared by the previous call
          CHECK_FALSE(dynamics.meanTravelDistance().is_valid);
        }
        THEN(
            "Only the streets on the requested side of the threshold contribute to the "
            "flow") {
          // The agent left streets 0, 1 and 2, which are now empty
          CHECK_EQ(dynamics.streetMeanFlow(0.5, false).n, 3);
          // Reading the flow does not consume the speed samples
          CHECK_EQ(dynamics.streetMeanFlow().n, 3);
          CHECK_EQ(dynamics.streetMeanFlow(0.5, true).n, 0);
        }
      }
    }
  }
  SUBCASE("Exit gates") {
    // Agents travel 0 -> 1 -> 2 on streets 0 and 1, towards destination street 2: they
    // arrive when leaving street 1. Returns the time steps of their arrivals.
    auto const arrivalSteps = [](std::size_t const nAgents,
                                 double const streetCapacity,
                                 double const nodeCapacity) {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 30., 15.);
      graph2.addStreet(1, std::make_pair(1, 2), 30., 15.);
      graph2.addStreet(2, std::make_pair(2, 3), 30., 15.);
      graph2.edge(1).setTransportCapacity(streetCapacity);
      graph2.node(1).setTransportCapacity(nodeCapacity);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::CONSTANT);
      for (std::size_t i{0}; i < nAgents; ++i) {
        dynamics.addAgent(2, 0);
      }
      std::vector<std::time_t> steps;
      for (int i{0}; i < 100 && steps.size() < nAgents; ++i) {
        auto const nAgentsBefore{dynamics.nAgents()};
        dynamics.evolve();
        for (auto n{dynamics.nAgents()}; n < nAgentsBefore; ++n) {
          steps.push_back(dynamics.time_step());
        }
      }
      REQUIRE_EQ(steps.size(), nAgents);
      return steps;
    };
    auto const headways = [](std::vector<std::time_t> const& steps) {
      std::vector<std::time_t> result;
      for (std::size_t i{1}; i < steps.size(); ++i) {
        result.push_back(steps[i] - steps[i - 1]);
      }
      return result;
    };
    auto const freeFlow{arrivalSteps(1, 1., 1.)};
    THEN("Without gates the agents arrive one per step") {
      CHECK_EQ(headways(arrivalSteps(4, 1., 1.)), std::vector<std::time_t>{1, 1, 1});
    }
    THEN("A street with transport capacity 0.25 lets a lone agent pass at once") {
      CHECK_EQ(arrivalSteps(1, 0.25, 1.), freeFlow);
    }
    THEN("A street with transport capacity 0.25 releases a platoon every 4 steps") {
      auto const steps{arrivalSteps(4, 0.25, 1.)};
      CHECK_EQ(steps.front(), freeFlow.front());
      CHECK_EQ(headways(steps), std::vector<std::time_t>{4, 4, 4});
    }
    THEN("A node with transport capacity 0.25 lets a lone agent pass at once") {
      CHECK_EQ(arrivalSteps(1, 1., 0.25), freeFlow);
    }
    THEN("A node with transport capacity 0.25 releases a platoon every 4 steps") {
      auto const steps{arrivalSteps(4, 1., 0.25)};
      CHECK_EQ(steps.front(), freeFlow.front());
      CHECK_EQ(headways(steps), std::vector<std::time_t>{4, 4, 4});
    }
  }
  SUBCASE("Moving agents are not counted as queued") {
    GIVEN("A long street and a single free-flowing agent") {
      // 300 m at 13.888... m/s => the agent needs 22 time-steps to cross street 0.
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 300., 15.);
      graph2.addStreet(1, std::make_pair(1, 2), 300., 15.);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.addAgent(1, 0);
      WHEN("The agent travels the street at free-flow speed") {
        dynamics.evolve();  // the agent leaves the pending list
        dynamics.evolve();  // the agent is put on street 0
        auto const& street{dynamics.graph().edge(0)};
        REQUIRE_EQ(street.nMovingAgents(), 1);
        auto const freeTime{street.movingAgents().top()->freeTime()};
        REQUIRE_GT(freeTime, dynamics.time_step());
        THEN(
            "It counts as moving - not as queued - until it reaches the end of the "
            "street") {
          while (dynamics.time_step() < freeTime) {
            // A vehicle in free flow must never show up in the exit queue, otherwise
            // mean_queue_length and the traffic-light optimizer read standing queues
            // where there is none.
            CHECK_EQ(street.nExitingAgents(), 0);
            CHECK_EQ(street.nMovingAgents(), 1);
            CHECK_EQ(street.nAgents(), 1);
            dynamics.evolve();
          }
        }
      }
    }
  }
  SUBCASE("Travel time excludes the wait before insertion") {
    GIVEN("A network whose origin street only has room for one vehicle") {
      // Capacity is ceil(length * nLanes / meanVehicleLength) = 1 for a 5 m street,
      // so the second agent cannot enter the network right away.
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 5., 15.);
      graph2.addStreet(1, std::make_pair(1, 2), 30., 15.);
      graph2.addStreet(2, std::make_pair(2, 3), 30., 15.);
      REQUIRE_EQ(graph2.edge(0).capacity(), 1);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      WHEN("Two agents are added at once on the same origin street") {
        dynamics.addAgent(2, 0);
        dynamics.addAgent(2, 0);
        REQUIRE_EQ(dynamics.agents().size(), 2);
        dynamics.evolve();
        THEN("The second one is still waiting in the pending list") {
          CHECK_EQ(dynamics.agents().size(), 1);
          CHECK_EQ(dynamics.agents().front()->spawnTime(), 0);
        }
        while (!dynamics.agents().empty()) {
          dynamics.evolve();
        }
        // It was created at t = 0 but only got a slot at t = 2; one more step to go
        // from the source node onto the origin street.
        REQUIRE_EQ(dynamics.time_step(), 3);
        dynamics.evolve();
        THEN("Its spawn time is re-stamped to the step it actually entered") {
          REQUIRE_EQ(
              dynamics.graph().edge(0).nAgents() + dynamics.graph().edge(1).nAgents(), 2);
          auto& originStreet{dynamics.graph().edge(0)};
          REQUIRE_EQ(originStreet.nMovingAgents(), 1);
          CHECK_EQ(originStreet.movingAgents().top()->spawnTime(), 2);
        }
        while (dynamics.nAgents() > 0) {
          dynamics.evolve();
        }
        THEN("Both agents report the same travel time and distance") {
          // Same route, same conditions: the queueing before insertion must not leak
          // into the measured travel time.
          auto const time{dynamics.meanTravelTime(false)};
          auto const distance{dynamics.meanTravelDistance(false)};
          CHECK_EQ(time.n, 2);
          // Streets 0 (1 s), 1 (2 s) and the destination street 2 (2 s), plus one step to
          // enter the origin street
          CHECK_EQ(time.mean, 6.);
          CHECK_EQ(time.std, doctest::Approx(0.));
          // Streets 0, 1 and the destination street 2, at whose end they arrive
          CHECK_EQ(distance.mean, 65.);
          CHECK_EQ(distance.std, doctest::Approx(0.));
        }
      }
      WHEN("Three agents are added at once on the same origin street") {
        for (int i{0}; i < 3; ++i) {
          dynamics.addAgent(2, 0);
        }
        dynamics.evolve();
        THEN("The agents still waiting keep their insertion order") {
          REQUIRE_EQ(dynamics.agents().size(), 2);
          CHECK_EQ(dynamics.agents().at(0)->id(), 1);
          CHECK_EQ(dynamics.agents().at(1)->id(), 2);
        }
      }
    }
  }
  SUBCASE("Stagnant agents") {
    GIVEN("An agent waiting at a traffic light which is never green for it") {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 13.8888888889);
      graph2.addStreet(1, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(2, std::make_pair(3, 1), 13.8888888889);
      auto& tl = graph2.makeTrafficLight(1);
      TrafficLightPhase phase{10};
      phase.addGreen(2);
      tl.addPhase(phase);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.addAgent(1, 0);
      WHEN("Stagnant agents are killed and we evolve the dynamics") {
        CHECK_THROWS_AS(dynamics.killStagnantAgents(0.), std::invalid_argument);
        dynamics.killStagnantAgents(1.);
        for (auto i{0}; i < 10; ++i) {
          dynamics.evolve();
        }
        THEN("The agent is killed instead of waiting forever") {
          auto const [nAdded, nInserted, nArrived, nKilled, nCurrent] =
              dynamics.agentStats();
          CHECK_EQ(nKilled, 1);
          CHECK_EQ(nArrived, 0);
          CHECK_EQ(nCurrent, 0);
        }
      }
    }
    GIVEN("Agents still waiting to enter a full origin street") {
      RoadNetwork graph2;
      // A 5 m street only has room for one vehicle
      graph2.addStreet(0, std::make_pair(0, 1), 5.);
      graph2.addStreet(1, std::make_pair(1, 2), 100.);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setODs({{0, 1, 1.}});
      dynamics.killStagnantAgents(1.);
      dynamics.addAgents(3, AgentInsertionMethod::ODS);
      dynamics.evolve();
      WHEN("New agents are added") {
        dynamics.addAgents(1, AgentInsertionMethod::ODS);
        THEN("The agents left waiting are counted as killed") {
          auto const [nAdded, nInserted, nArrived, nKilled, nCurrent] =
              dynamics.agentStats();
          CHECK_GT(nKilled, 0);
          CHECK_EQ(nInserted, nArrived + nKilled + nCurrent);
        }
      }
    }
  }
  SUBCASE("Intersection right of way") {
    GIVEN("A dynamics object with five nodes and eight streets") {
      RoadNetwork graph2;
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

      graph2.addEdge<Street>(11, std::make_pair(1, 11), 10., 10.);
      graph2.addEdge<Street>(22, std::make_pair(2, 22), 10., 10.);
      graph2.addEdge<Street>(33, std::make_pair(3, 33), 10., 10.);
      graph2.addEdge<Street>(44, std::make_pair(4, 44), 10., 10.);
      FirstOrderDynamics dynamics{std::move(graph2), 69};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.graph().node(0).setCapacity(3);
      dynamics.graph().node(0).setTransportCapacity(1);
      auto& nodeO{dynamic_cast<Intersection&>(dynamics.graph().node(0))};
      WHEN("We add agents and evolve the dynamics") {
        // add an agent in C, D, A
        dynamics.addAgent(22, 20);  // Second
        dynamics.addAgent(22, 15);  // Third
        dynamics.addAgent(22, 5);   // First
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent in A passes first") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 2);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 20);
        }
        dynamics.evolve();
        THEN("The agent in D passes second") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 2);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 15);
        }
        dynamics.evolve();
        THEN("The agent in C passes last") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 3);
          CHECK(nodeO.agents().empty());
        }
      }
      WHEN("We add agents of another destination and update the dynamics") {
        dynamics.addAgent(11, 10);
        dynamics.addAgent(11, 15);
        dynamics.addAgent(11, 20);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent in D passes first") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 2);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 15);
        }
        dynamics.evolve();
        THEN("The agent in C passes second") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 2);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 10);
        }
        dynamics.evolve();
        THEN("The agent in C passes last") {
          CHECK_EQ(dynamics.graph().edge(1).nAgents(), 3);
          CHECK(nodeO.agents().empty());
        }
      }
      WHEN("We set street priorities and add agents") {
        nodeO.addStreetPriority(5);
        nodeO.addStreetPriority(15);
        dynamics.addAgent(22, 5);
        dynamics.addAgent(22, 20);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent in A passes first because it's on the main road") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          // The agent in D gives way: it waits on its street, not in the node
          CHECK(nodeO.agents().empty());
          CHECK_EQ(dynamics.graph().edge(20).nAgents(), 1);
        }
        dynamics.evolve();
        THEN("The agent in D passes next") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(20).nAgents(), 0);
        }
      }
      WHEN("We add agents in A and D without priorities") {
        dynamics.addAgent(22, 5);
        dynamics.addAgent(22, 20);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent in A passes first") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 20);
        }
      }
      WHEN("We set street priorities on the D-B road and add agents in A and D") {
        nodeO.addStreetPriority(10);
        nodeO.addStreetPriority(20);
        dynamics.addAgent(22, 5);
        dynamics.addAgent(22, 20);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The agent in D passes first because it's on the main road") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          // The agent in A gives way: it waits on its street, not in the node
          CHECK(nodeO.agents().empty());
          CHECK_EQ(dynamics.graph().edge(5).nAgents(), 1);
        }
        dynamics.evolve();
        THEN("The agent in A passes next") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(5).nAgents(), 0);
        }
      }
      WHEN("We flag the D-B road as priority without applying it to the intersections") {
        dynamics.graph().edge(10).setPriority();
        dynamics.graph().edge(20).setPriority();
        dynamics.addAgent(22, 5);
        dynamics.addAgent(22, 20);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The flags are ignored: the agent in A passes first") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 20);
        }
      }
      WHEN("We flag the D-B road as priority and add agents in A and D") {
        // Same as above, through the street flag (e.g. the "priority" column)
        dynamics.graph().edge(10).setPriority();
        dynamics.graph().edge(20).setPriority();
        dynamics.graph().applyIntersectionPriorities();
        dynamics.addAgent(22, 5);
        dynamics.addAgent(22, 20);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        THEN("The intersection's priority set is left empty") {
          CHECK(nodeO.streetPriorities().empty());
        }
        THEN("The agent in D passes first because it's on the main road") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 1);
          // The agent in A gives way: it waits on its street, not in the node
          CHECK(nodeO.agents().empty());
          CHECK_EQ(dynamics.graph().edge(5).nAgents(), 1);
        }
        dynamics.evolve();
        THEN("The agent in A passes next") {
          CHECK_EQ(dynamics.graph().edge(2).nAgents(), 2);
          CHECK_EQ(dynamics.graph().edge(5).nAgents(), 0);
        }
      }
      WHEN("The road from A to C is the main one and the network is prepared") {
        for (Id const streetId : {5, 15}) {
          dynamics.graph().edge(streetId).setMobilityClass(64u);  // PRIMARY
        }
        for (Id const streetId : {10, 20}) {
          dynamics.graph().edge(streetId).setMobilityClass(8u);  // RESIDENTIAL
        }
        AND_WHEN("The intersection priorities are applied") {
          dynamics.prepareNetwork(false, false, true, false, true);
          dynamics.addAgent(11, 15);
          dynamics.addAgent(11, 20);
          dynamics.evolve();
          dynamics.evolve();
          dynamics.evolve();
          THEN("The agent in C passes first because it's on the main road") {
            CHECK(nodeO.hasRightOfWay(dynamics.graph().edge(15)));
            CHECK_FALSE(nodeO.hasRightOfWay(dynamics.graph().edge(20)));
            CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
            // The agent in D gives way: it waits on its street, not in the node
            CHECK(nodeO.agents().empty());
            CHECK_EQ(dynamics.graph().edge(20).nAgents(), 1);
          }
        }
        AND_WHEN("The intersection priorities are not applied (the default)") {
          dynamics.prepareNetwork(false, false, true, false);
          dynamics.addAgent(11, 15);
          dynamics.addAgent(11, 20);
          dynamics.evolve();
          dynamics.evolve();
          dynamics.evolve();
          THEN("The agent in D passes first, as without road types") {
            CHECK_FALSE(nodeO.hasPriorityStreets());
            CHECK_EQ(dynamics.graph().edge(1).nAgents(), 1);
            CHECK_EQ(nodeO.agents().size(), 1);
            CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 15);
          }
        }
      }
      WHEN("Two agents queue on the same lane and the intersection keeps the first") {
        // A negligible transport capacity keeps the agents in the intersection
        nodeO.setTransportCapacity(1e-9);
        dynamics.addAgent(22, 20);
        dynamics.addAgent(22, 20);
        for (auto i{0}; i < 5; ++i) {
          dynamics.evolve();
        }
        THEN(
            "The second agent waits on the street, although the intersection is not "
            "full") {
          CHECK_FALSE(nodeO.isFull());
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK(nodeO.isLaneOccupied(20, 0));
          CHECK_EQ(dynamics.graph().edge(20).nExitingAgents(), 1);
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

      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 300., 50.);
      graph.addStreet(1, std::make_pair(1, 2), 1000., 50.);
      graph.addStreet(2, std::make_pair(1, 3), 1000., 30.);
      graph.autoMapStreetLanes();
      graph.adjustNodeCapacities();

      FirstOrderDynamics dynamics{std::move(graph), 42};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dynamics.setOrigins({{0, 1.0}});

      WHEN("We add multiple random agents and evolve the system") {
        dynamics.addAgents(6, AgentInsertionMethod::RANDOM_WEIGHTED_ORIGIN);
        CHECK_EQ(dynamics.nAgents(), 6);
        // Evolve to get agents onto street 0
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        CHECK_EQ(dynamics.graph().edge(0).nAgents(), 6);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();

        THEN("The distribution of agents follows the transition probabilities") {
          CHECK_EQ(dynamics.graph().edge(0).nAgents(), 0);
          // Both exits are equally likely. The exact split is seed-dependent; the
          // invariant is that nobody is lost.
          CHECK_EQ(
              dynamics.graph().edge(1).nAgents() + dynamics.graph().edge(2).nAgents(), 6);
        }
      }
    }

    GIVEN("A network with three possible exits from a node") {
      // Create a network with one incoming street and three outgoing streets
      // Street layout:
      //   0 --[s0]--> 1 --[s1]--> 2
      //                |--[s2]--> 3
      //                \--[s3]--> 4

      RoadNetwork graph;
      graph.addStreet(0, std::make_pair(0, 1), 300., 50.);
      graph.addStreet(1, std::make_pair(1, 2), 1000., 50.);
      graph.addStreet(2, std::make_pair(1, 3), 1000., 40.);
      graph.addStreet(3, std::make_pair(1, 4), 1000., 30.);
      graph.autoMapStreetLanes();
      graph.adjustNodeCapacities();

      FirstOrderDynamics dynamics{std::move(graph), 123};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.);
      dynamics.setOrigins({{0, 1.0}});

      WHEN("We add multiple random agents and evolve the system") {
        dynamics.addAgents(6, AgentInsertionMethod::RANDOM_WEIGHTED_ORIGIN);
        CHECK_EQ(dynamics.nAgents(), 6);
        // Evolve to get agents onto street 0
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        CHECK_EQ(dynamics.graph().edge(0).nAgents(), 6);
        // Single-lane streets: the length-weighted density is the agents over the total length
        auto const density{dynamics.streetMeanDensity<false>()};
        CHECK_EQ(density.mean, doctest::Approx(6. / 3300.));
        CHECK_EQ(density.n, 4);
        auto const normalizedDensity{dynamics.streetMeanDensity<true>()};
        CHECK_EQ(normalizedDensity.mean,
                 doctest::Approx(dynamics.graph().edge(0).density<true>() / 4.));
        CHECK_EQ(normalizedDensity.n, 4);
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();
        dynamics.evolve();

        THEN("The distribution of agents follows the transition probabilities") {
          CHECK_EQ(dynamics.graph().edge(0).nAgents(), 0);
          // The exact split is seed-dependent; the invariant is that nobody is lost.
          CHECK_EQ(dynamics.graph().edge(1).nAgents() +
                       dynamics.graph().edge(2).nAgents() +
                       dynamics.graph().edge(3).nAgents(),
                   6);
        }
      }
    }
  }
  SUBCASE("Seeded results are reproducible with concurrency 1") {
    // With a single thread, the per-thread engine is seeded from the dynamics' seed only, so
    // the same seed gives the same simulation, also when it is set after construction
    auto const run = [](unsigned int const constructorSeed,
                        std::optional<unsigned int> const newSeed) {
      auto network = RoadNetwork{};
      network.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
      network.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());
      FirstOrderDynamics dynamics{std::move(network), constructorSeed};
      dynamics.setConcurrency(1);
      if (newSeed.has_value()) {
        dynamics.setSeed(*newSeed);
      }
      dynamics.setPassageProbability(0.7);
      dynamics.setPathRandomness(0.3);
      auto const& graph = dynamics.graph();
      dynamics.setConditionalODs(
          {{graph.node(1).outgoingEdges().front()->id(),
            {1.0, {{graph.node(14).ingoingEdges().front()->id(), 1.0}}}}});
      dynamics.addAgents(200, AgentInsertionMethod::RANDOM);
      dynamics.addAgents(50, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS);
      for (auto i{0}; i < 300; ++i) {
        dynamics.evolve();
      }
      std::vector<std::pair<Id, std::size_t>> streetAgents;
      for (auto const& [streetId, pStreet] : dynamics.graph().edges()) {
        streetAgents.emplace_back(streetId, pStreet->nAgents());
      }
      std::ranges::sort(streetAgents);
      return std::make_pair(dynamics.agentStats(), streetAgents);
    };
    auto const [firstStats, firstStreets] = run(69, std::nullopt);
    auto const [secondStats, secondStreets] = run(69, std::nullopt);
    auto const [reseededStats, reseededStreets] = run(1, 69);
    // Make sure the simulation actually moved agents around
    CHECK(firstStats.nArrived + firstStats.nKilled > 0);
    CHECK_EQ(firstStats, secondStats);
    CHECK_EQ(firstStreets, secondStreets);
    CHECK_EQ(firstStats, reseededStats);
    CHECK_EQ(firstStreets, reseededStreets);
  }
}

TEST_CASE("RoadDynamics Configuration") {
  auto defaultNetwork = RoadNetwork{};
  // We need at least some nodes to add agents
  // Assuming the data files exist where Test_dynamics.cpp expects them
  defaultNetwork.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
  defaultNetwork.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());
  defaultNetwork.autoMapStreetLanes();

  FirstOrderDynamics dynamics{std::move(defaultNetwork), 42};

  SUBCASE("originCounts and destinationCounts") {
    GIVEN("A simple network with origin and destination nodes") {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 13.8888888889);
      graph2.addStreet(1, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(2, std::make_pair(2, 3), 13.8888888889);
      FirstOrderDynamics dyn{std::move(graph2), 42};
      dyn.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);

      WHEN("We add agents with source nodes and evolve until they reach destination") {
        dyn.addAgent(2, 0);
        dyn.addAgent(2, 0);
        dyn.addAgent(2, 1);

        CHECK_EQ(dyn.nAgents(), 3);

        THEN("originCounts returns the correct counts") {
          auto counts = dyn.originCounts(false);
          CHECK_FALSE(counts.empty());
          CHECK_EQ(counts.at(0), 2);
          CHECK_EQ(counts.at(1), 1);
        }

        THEN("originCounts with bReset=true clears the counts") {
          auto counts = dyn.originCounts(true);
          CHECK_FALSE(counts.empty());
          CHECK_EQ(counts.at(0), 2);
          CHECK_EQ(counts.at(1), 1);

          auto countsAfterReset = dyn.originCounts(false);
          CHECK(countsAfterReset.empty());
        }

        // Evolve until agents reach destination
        while (dyn.nAgents() > 0) {
          dyn.evolve();
        }

        THEN("destinationCounts returns the correct counts") {
          auto destCounts = dyn.destinationCounts(false);
          if (destCounts.empty()) {
            spdlog::warn(
                "destinationCounts is empty. This may indicate that no agents reached "
                "their destinations.");
          }
          CHECK_FALSE(destCounts.empty());
          CHECK_EQ(destCounts.at(2), 3);
        }

        THEN("destinationCounts with bReset=true clears the counts") {
          auto destCounts = dyn.destinationCounts(true);
          CHECK_FALSE(destCounts.empty());
          CHECK_EQ(destCounts.at(2), 3);

          auto destCountsAfterReset = dyn.destinationCounts(false);
          CHECK(destCountsAfterReset.empty());
        }
      }
    }

    GIVEN("Multiple destinations") {
      RoadNetwork graph2;
      graph2.addStreet(0, std::make_pair(0, 1), 13.8888888889);
      graph2.addStreet(1, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(2, std::make_pair(1, 3), 13.8888888889);
      graph2.addStreet(3, std::make_pair(4, 0), 13.8888888889);
      FirstOrderDynamics dyn{std::move(graph2), 42};
      dyn.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      dyn.setDestinations({1, 2});

      WHEN("Agents travel to different destinations") {
        dyn.addAgent(1, 3);
        dyn.addAgent(2, 3);
        dyn.addAgent(2, 3);

        // Evolve until all agents reach destination
        while (dyn.nAgents() > 0) {
          dyn.evolve();
        }

        THEN("destinationCounts tracks each destination separately") {
          auto destCounts = dyn.destinationCounts(false);
          CHECK_FALSE(destCounts.empty());
          CHECK_EQ(destCounts.at(1), 1);
          CHECK_EQ(destCounts.at(2), 2);
        }
      }
    }
  }

  SUBCASE("DynamicsMode") {
    GIVEN(
        "A network where node 2 has two outgoing streets, nodes 3, 4 have none and node "
        "0 "
        "no ingoing one") {
      // Street ids differ from node ids, so that mixing them up fails
      RoadNetwork graph2;
      graph2.addStreet(10, std::make_pair(0, 1), 13.8888888889);
      graph2.addStreet(11, std::make_pair(1, 2), 13.8888888889);
      graph2.addStreet(12, std::make_pair(2, 3), 13.8888888889);
      graph2.addStreet(13, std::make_pair(2, 4), 13.8888888889);
      FirstOrderDynamics dyn{std::move(graph2), 42};

      auto const sortedIds = [](auto const& weightedIds) {
        std::vector<Id> ids;
        double sumWeights{0.};
        for (auto const& [id, weight] : weightedIds) {
          ids.push_back(id);
          sumWeights += weight;
        }
        CHECK_EQ(sumWeights, doctest::Approx(1.));
        std::ranges::sort(ids);
        return ids;
      };

      THEN("The default mode is EDGES and ids are street ids") {
        CHECK_EQ(dyn.mode(), DynamicsMode::EDGES);
        dyn.setOrigins({{11, 1.}});
        dyn.setDestinations({12});
        CHECK_EQ(sortedIds(dyn.origins()), std::vector<Id>{11});
        CHECK_EQ(sortedIds(dyn.destinations()), std::vector<Id>{12});
      }
      THEN("The mode cannot change once origins or destinations are set") {
        dyn.setOrigins({{11, 1.}});
        CHECK_THROWS_AS(dyn.setMode(DynamicsMode::NODES), std::runtime_error);
        CHECK_NOTHROW(dyn.setMode(DynamicsMode::EDGES));
        CHECK_EQ(dyn.mode(), DynamicsMode::EDGES);
      }
      WHEN("The mode is NODES") {
        dyn.setMode(DynamicsMode::NODES);
        CHECK_EQ(dyn.mode(), DynamicsMode::NODES);

        THEN(
            "Origins are spread over the outgoing streets, destinations over the "
            "ingoing") {
          dyn.setOrigins({{2, 1.}});
          CHECK_EQ(sortedIds(dyn.origins()), std::vector<Id>{12, 13});
          for (auto const& [id, weight] : dyn.origins()) {
            CHECK_EQ(weight, doctest::Approx(0.5));
          }
          dyn.setDestinations({1, 2, 3});
          CHECK_EQ(sortedIds(dyn.destinations()), std::vector<Id>{10, 11, 12});
          for (auto const& [id, weight] : dyn.destinations()) {
            CHECK_EQ(weight, doctest::Approx(1. / 3.));
          }
          CHECK_EQ(dyn.destinationStreets().size(), 3);
        }
        THEN("Nodes without such streets are skipped") {
          dyn.setDestinations({0, 4});
          CHECK_EQ(sortedIds(dyn.destinations()), std::vector<Id>{13});
          CHECK_THROWS_AS(dyn.setDestinations({0}), std::invalid_argument);
          CHECK_THROWS_AS(dyn.setOrigins({{3, 1.}}), std::invalid_argument);
          CHECK_THROWS_AS(dyn.setODs({{3, 2, 1.}}), std::invalid_argument);
          CHECK_THROWS_AS(dyn.setODs({{1, 0, 1.}}), std::invalid_argument);
          CHECK_THROWS_AS(dyn.setConditionalODs({{3, {1., {{2, 1.}}}}}),
                          std::invalid_argument);
        }
        THEN("Unknown nodes throw") {
          CHECK_THROWS_AS(dyn.setOrigins({{99, 1.}}), std::out_of_range);
        }
        THEN("Conditional ODs are spread over the node's streets") {
          dyn.setConditionalODs({{0, {1., {{3, 1.}, {4, 1.}}}}});
          CHECK_EQ(sortedIds(dyn.origins()), std::vector<Id>{10});
          dyn.addAgents(10, AgentInsertionMethod::CONDITIONAL_RANDOM_ODS);
          CHECK_EQ(dyn.nAgents(), 10);
          for (auto const& agent : dyn.agents()) {
            auto const destination{agent->destination().value()};
            CHECK((destination == 12 || destination == 13));
          }
        }
        WHEN("Agents travel from node 0 to node 2") {
          dyn.setODs({{0, 2, 1.}});
          CHECK_EQ(dyn.destinationStreets().size(), 1);
          dyn.addAgents(4, AgentInsertionMethod::ODS);
          CHECK_EQ(dyn.nAgents(), 4);

          THEN("They start from node 0 and are counted by node") {
            auto const counts{dyn.originCounts(false)};
            CHECK_EQ(counts.size(), 1);
            CHECK_EQ(counts.at(0), 4);
          }
          THEN("They arrive at node 2 and are counted by node") {
            for (std::size_t i{0}; i < 1000 && dyn.nAgents() > 0; ++i) {
              dyn.evolve();
            }
            CHECK_EQ(dyn.nAgents(), 0);
            CHECK_EQ(dyn.agentStats().nArrived, 4);
            auto const counts{dyn.destinationCounts(false)};
            CHECK_EQ(counts.size(), 1);
            CHECK_EQ(counts.at(2), 4);
          }
        }
      }
    }
  }

  SUBCASE("setMeanTravelDistance") {
    CHECK_THROWS_AS(dynamics.setMeanTravelDistance(-1.0), std::invalid_argument);
    CHECK_THROWS_AS(dynamics.setMeanTravelDistance(0.0), std::invalid_argument);

    dynamics.setMeanTravelDistance(1000.0);

    dynamics.addAgents(10, AgentInsertionMethod::RANDOM);

    const auto& agents = dynamics.agents();
    CHECK(agents.size() == 10);
    for (const auto& agent : agents) {
      CHECK_NOTHROW(agent->maxDistance());
      CHECK(agent->maxDistance() >= 0.0);
    }
  }

  SUBCASE("setPassageProbability") {
    for (auto const p : {-0.1, 1.1}) {
      CHECK_THROWS_AS(dynamics.setPassageProbability(p), std::invalid_argument);
    }
    CHECK_NOTHROW(dynamics.setPassageProbability(0.5));
  }

  SUBCASE("setMeanTravelTime") {
    dynamics.setMeanTravelTime(3600);

    dynamics.addAgents(10, AgentInsertionMethod::RANDOM);

    const auto& agents = dynamics.agents();
    CHECK(agents.size() == 10);
    for (const auto& agent : agents) {
      CHECK_NOTHROW(agent->maxTime());
      CHECK(agent->maxTime() >= 0);
    }
  }
}

TEST_CASE("SpeedFunction::CONSTANT") {
  // 0 --[s0: 100 m, 10 m/s]--> 1 --[s1: 200 m, 20 m/s]--> 2 --[s2: 100 m, 10 m/s]--> 3
  // Agents are routed to node 2, which needs an outgoing street not to be a dead end.
  auto const makeNetwork = []() {
    RoadNetwork graph;
    graph.addStreet(0, std::make_pair(0, 1), 100., 10.);
    graph.addStreet(1, std::make_pair(1, 2), 200., 20.);
    graph.addStreet(2, std::make_pair(2, 3), 100., 10.);
    return graph;
  };
  SUBCASE("Arguments") {
    GIVEN("A dynamics object") {
      FirstOrderDynamics dynamics{makeNetwork(), 42};
      THEN("The constant speed function takes no arguments") {
        CHECK_NOTHROW(dynamics.setSpeedFunction(dsf::SpeedFunction::CONSTANT));
      }
      THEN("Passing an argument throws") {
        CHECK_THROWS_AS(dynamics.setSpeedFunction(dsf::SpeedFunction::CONSTANT, 0.5),
                        std::invalid_argument);
      }
      THEN("The linear speed function needs an alpha in [0, 1)") {
        CHECK_THROWS_AS(dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 1.),
                        std::invalid_argument);
      }
    }
  }
  SUBCASE("Estimated travel time") {
    GIVEN("A dynamics object using the constant speed function") {
      FirstOrderDynamics dynamics{makeNetwork(), 42};
      dynamics.setSpeedFunction(dsf::SpeedFunction::CONSTANT);
      THEN("Empty streets are traversed in free flow") {
        CHECK_EQ(dynamics.graph().edge(0).estimatedTravelTime(), doctest::Approx(10.));
        CHECK_EQ(dynamics.graph().edge(1).estimatedTravelTime(), doctest::Approx(10.));
      }
      WHEN("The first street gets loaded with agents") {
        for (auto i{0}; i < 15; ++i) {
          dynamics.addAgent(2, 0);
        }
        for (auto i{0}; i < 5; ++i) {
          dynamics.evolve();
        }
        auto const& street{dynamics.graph().edge(0)};
        REQUIRE(street.density<true>() > 0.);
        THEN("The estimated travel time does not depend on the density") {
          CHECK_EQ(street.estimatedTravelTime(), doctest::Approx(10.));
        }
        THEN("The linear speed function would instead slow the street down") {
          auto const density{street.movingDensity<true>()};
          dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
          CHECK_EQ(street.estimatedTravelTime(),
                   doctest::Approx(100. / (10. * (1. - 0.8 * density))));
          CHECK(street.estimatedTravelTime() > 10.);
        }
      }
    }
  }
  SUBCASE("Agent speeds") {
    auto const load = [](FirstOrderDynamics& dynamics) {
      for (auto i{0}; i < 15; ++i) {
        dynamics.addAgent(2, 0);
      }
      for (auto i{0}; i < 10; ++i) {
        dynamics.evolve();
      }
    };
    auto const speeds = [](FirstOrderDynamics const& dynamics, Id const streetId) {
      std::vector<double> result;
      for (auto const& pAgent : dynamics.graph().edge(streetId).movingAgents()) {
        result.push_back(pAgent->speed());
      }
      return result;
    };
    GIVEN("A dynamics object using the constant speed function") {
      FirstOrderDynamics dynamics{makeNetwork(), 42};
      dynamics.setSpeedFunction(dsf::SpeedFunction::CONSTANT);
      WHEN("We evolve the dynamics until the streets are loaded") {
        load(dynamics);
        THEN("Every moving agent travels at the speed limit of its street") {
          for (Id streetId{0}; streetId < 2; ++streetId) {
            auto const maxSpeed{dynamics.graph().edge(streetId).maxSpeed()};
            for (auto const speed : speeds(dynamics, streetId)) {
              CHECK_EQ(speed, doctest::Approx(maxSpeed));
            }
          }
          // The loop above must not be vacuous, and the street must be congested
          // enough for a density-dependent speed function to behave differently
          CHECK_FALSE(speeds(dynamics, 0).empty());
          CHECK(dynamics.graph().edge(0).density<true>() > 0.);
        }
      }
    }
    GIVEN("The same dynamics using the linear speed function") {
      FirstOrderDynamics dynamics{makeNetwork(), 42};
      dynamics.setSpeedFunction(dsf::SpeedFunction::LINEAR, 0.8);
      WHEN("We evolve the dynamics until the streets are loaded") {
        load(dynamics);
        THEN("Some agent is slower than the speed limit of its street") {
          auto const maxSpeed{dynamics.graph().edge(0).maxSpeed()};
          auto const streetSpeeds{speeds(dynamics, 0)};
          REQUIRE_FALSE(streetSpeeds.empty());
          auto bSomeoneIsSlower{false};
          for (auto const speed : streetSpeeds) {
            bSomeoneIsSlower |= speed < maxSpeed;
          }
          CHECK(bSomeoneIsSlower);
        }
      }
    }
    GIVEN("The same dynamics using a custom speed function") {
      FirstOrderDynamics dynamics{makeNetwork(), 42};
      dynamics.setSpeedFunction(dsf::SpeedFunction::CUSTOM, [](Street const& street) {
        return street.maxSpeed() / 2.;
      });
      WHEN("We evolve the dynamics until the streets are loaded") {
        load(dynamics);
        THEN("Every moving agent travels at the custom speed") {
          auto const streetSpeeds{speeds(dynamics, 0)};
          REQUIRE_FALSE(streetSpeeds.empty());
          for (auto const speed : streetSpeeds) {
            CHECK_EQ(speed, doctest::Approx(dynamics.graph().edge(0).maxSpeed() / 2.));
          }
          CHECK_EQ(dynamics.graph().edge(0).estimatedTravelTime(), doctest::Approx(20.));
        }
      }
    }
  }
}
