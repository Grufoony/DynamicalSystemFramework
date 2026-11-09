#include <cstdint>
#include <format>
#include <string>

#include "dsf/mobility/Agent.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using Agent = dsf::mobility::Agent;

TEST_CASE("Agent") {
  SUBCASE("Constructors") {
    GIVEN("An agent and its destination") {
      dsf::Id const destination{5};
      WHEN("The Agent is constructed") {
        Agent agent{0, 0, destination};
        THEN("The agent and its destination are set correctly") {
          CHECK_EQ(agent.destination(), destination);
          CHECK_FALSE(agent.isRandom());
          CHECK_FALSE(agent.streetId().has_value());
          CHECK_FALSE(agent.srcNodeId().has_value());
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.freeTime(), 0);
          CHECK_EQ(agent.spawnTime(), 0);
        }
      }
    }
    GIVEN("An agent, its destination, and a node id") {
      dsf::Id const destination{5};
      dsf::Id srcNodeId{0};
      WHEN("The Agent is constructed") {
        Agent agent{0, 0, destination, srcNodeId};
        THEN("The agent and its destination are set correctly") {
          CHECK_EQ(agent.destination(), destination);
          CHECK_FALSE(agent.streetId().has_value());
          CHECK(agent.srcNodeId().has_value());
          CHECK_EQ(agent.srcNodeId().value(), srcNodeId);
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.freeTime(), 0);
          CHECK_EQ(agent.spawnTime(), 0);
        }
      }
    }
    GIVEN("No destination and no source node id") {
      WHEN("The agent is constructed") {
        auto randomAgent = Agent{0, 0};
        THEN("The agent is a random agent") {
          CHECK(randomAgent.isRandom());
          CHECK_FALSE(randomAgent.destination().has_value());
          CHECK_THROWS_AS(randomAgent.setPath(std::vector<dsf::Id>{1}), std::logic_error);
        }
      }
    }
  }
}

TEST_CASE("Agent methods") {
  Agent agent{0, 0, dsf::Id{100}, 7};
  SUBCASE("setSrcNodeId and srcNodeId") {
    agent.setSrcNodeId(99);
    CHECK(agent.srcNodeId().has_value());
    CHECK_EQ(agent.srcNodeId().value(), 99);
  }
  SUBCASE("setSpawnTime and spawnTime") {
    CHECK_EQ(agent.spawnTime(), 0);
    // The dynamics re-stamps the spawn time when the agent is actually inserted into
    // the network, so that the wait in the pending list is not billed as travel time.
    agent.setSpawnTime(42);
    CHECK_EQ(agent.spawnTime(), 42);
  }
  SUBCASE("setNextStreetId and nextStreetId") {
    agent.setNextStreetId(123);
    CHECK(agent.nextStreetId().has_value());
    CHECK_EQ(agent.nextStreetId().value(), 123);
  }
  SUBCASE("setStreetId with value") {
    agent.setNextStreetId(55);
    CHECK_THROWS_AS(agent.setStreetId(55), std::logic_error);
    CHECK_FALSE(agent.streetId().has_value());
    agent.setStreetId();
    CHECK_EQ(agent.streetId().value(), 55);
    CHECK_FALSE(agent.nextStreetId().has_value());
  }
  SUBCASE("setStreetId with nullopt uses nextStreetId") {
    CHECK_THROWS_AS(agent.setStreetId(), std::logic_error);
    agent.setNextStreetId(77);
    agent.setStreetId();
    CHECK(agent.streetId().has_value());
    CHECK_EQ(agent.streetId().value(), 77);
  }
  SUBCASE("setSpeed and speed") {
    agent.setSpeed(12.5);
    CHECK_EQ(agent.speed(), 12.5);
    CHECK_THROWS_AS(agent.setSpeed(-1.), std::invalid_argument);
  }
  SUBCASE("setFreeTime and freeTime") {
    agent.setFreeTime(1234);
    CHECK_EQ(agent.freeTime(), 1234);
  }
  SUBCASE("incrementDistance and distance") {
    double d0 = agent.distance();
    agent.incrementDistance(5.5);
    CHECK_EQ(agent.distance(), d0 + 5.5);
    CHECK_THROWS_AS(agent.incrementDistance(-1.), std::invalid_argument);
  }
  SUBCASE("Planned route") {
    // The route ends with the destination, which is never planned as a street to take
    CHECK_EQ(agent.nPlannedStreets(), 0);
    CHECK_FALSE(agent.nextPlannedStreetId().has_value());
    agent.setPath(std::vector<dsf::Id>{3, 5, 8});
    CHECK_EQ(agent.nPlannedStreets(), 3);
    CHECK_EQ(agent.nextPlannedStreetId().value(), 3);
    agent.advancePath();
    CHECK_EQ(agent.nextPlannedStreetId().value(), 5);
    agent.advancePath();
    CHECK_EQ(agent.nextPlannedStreetId().value(), 8);
    agent.advancePath();
    CHECK_FALSE(agent.nextPlannedStreetId().has_value());
    CHECK_THROWS_AS(agent.advancePath(), std::logic_error);
    CHECK_EQ(agent.destination(), 100);
    // A new plan replaces the previous one; dropping it keeps the destination
    agent.setPath(std::vector<dsf::Id>{1, 2});
    agent.setPath(std::vector<dsf::Id>{4});
    CHECK_EQ(agent.nPlannedStreets(), 1);
    CHECK_EQ(agent.nextPlannedStreetId().value(), 4);
    agent.dropPath();
    CHECK_EQ(agent.nPlannedStreets(), 0);
    CHECK_EQ(agent.destination(), 100);
  }
  SUBCASE("isIntelligent") {
    Agent agent{0, 0, dsf::Id{5}};
    CHECK_FALSE(agent.isIntelligent());  // agents are not intelligent by default
    agent.setIntelligent(true);
    CHECK(agent.isIntelligent());
    agent.reset(10);
    CHECK(agent.isIntelligent());  // the flag survives reinsertion
    agent.setIntelligent(false);
    CHECK_FALSE(agent.isIntelligent());
  }
  SUBCASE("reset resets state") {
    agent.setSpeed(10.);
    agent.incrementDistance(5.);
    agent.setFreeTime(99);
    agent.setStreetId(88);
    agent.setPath(std::vector<dsf::Id>{88, 89});
    agent.reset(555);
    CHECK_EQ(agent.nPlannedStreets(), 0);
    CHECK_EQ(agent.spawnTime(), 555);
    CHECK_EQ(agent.freeTime(), 0);
    CHECK_EQ(agent.speed(), 0.);
    CHECK_EQ(agent.distance(), 0.);
    CHECK_EQ(agent.destination(), 100);  // the destination is kept
    CHECK_FALSE(agent.streetId().has_value());
  }
  SUBCASE("reset restarts from the first street") {
    agent.setSrcStreetId(3);
    agent.setNextStreetId(4);
    agent.reset(555);
    CHECK_EQ(agent.srcStreetId().value(), 3);
    CHECK_EQ(agent.nextStreetId().value(), 3);
  }
  SUBCASE("reset clears the next street if the first street is unknown") {
    agent.setNextStreetId(4);
    agent.reset(555);
    CHECK_FALSE(agent.srcStreetId().has_value());
    CHECK_FALSE(agent.nextStreetId().has_value());
  }
}

TEST_CASE("Agent formatting") {
  SUBCASE("std::format with complete agent") {
    Agent agent{0, 0, dsf::Id{42}, 7};
    agent.setStreetId(10);
    agent.setNextStreetId(15);
    agent.setSpeed(13.5);
    agent.incrementDistance(100.25);
    agent.setFreeTime(123);

    std::string formatted = std::format("{}", agent);
    CHECK(formatted.find("id: 0") != std::string::npos);
    CHECK(formatted.find("streetId: 10") != std::string::npos);
    CHECK(formatted.find("srcNodeId: 7") != std::string::npos);
    CHECK(formatted.find("nextStreetId: 15") != std::string::npos);
    CHECK(formatted.find("destination: 42") != std::string::npos);
    CHECK(formatted.find("speed: 13.50 m/s") != std::string::npos);
    CHECK(formatted.find("distance: 100.25 m") != std::string::npos);
    CHECK(formatted.find("spawnTime: 0") != std::string::npos);
    CHECK(formatted.find("freeTime: 123") != std::string::npos);
  }

  SUBCASE("std::format with random agent") {
    Agent randomAgent{0, 5};

    std::string formatted = std::format("{}", randomAgent);
    CHECK(formatted.find("id: 0") != std::string::npos);
    CHECK(formatted.find("streetId: N/A") != std::string::npos);
    CHECK(formatted.find("srcNodeId: N/A") != std::string::npos);
    CHECK(formatted.find("nextStreetId: N/A") != std::string::npos);
    CHECK(formatted.find("destination: RANDOM") != std::string::npos);
    CHECK(formatted.find("speed: 0.00 m/s") != std::string::npos);
    CHECK(formatted.find("distance: 0.00 m") != std::string::npos);
    CHECK(formatted.find("spawnTime: 5") != std::string::npos);
    CHECK(formatted.find("freeTime: 0") != std::string::npos);
  }

  SUBCASE("std::format with agent with optional nullopts") {
    Agent agent{0, 10, dsf::Id{99}};

    std::string formatted = std::format("{}", agent);
    CHECK(formatted.find("id: 0") != std::string::npos);
    CHECK(formatted.find("streetId: N/A") != std::string::npos);
    CHECK(formatted.find("srcNodeId: N/A") != std::string::npos);
    CHECK(formatted.find("nextStreetId: N/A") != std::string::npos);
    CHECK(formatted.find("destination: 99") != std::string::npos);
  }
  SUBCASE("hasArrived") {
    Agent agent{0, 10};  // spawnTime = 10
    CHECK(agent.isRandom());
    CHECK_THROWS_AS(agent.setMaxDistance(0.), std::invalid_argument);

    // Test Max Distance
    agent.setMaxDistance(100.0);
    CHECK_FALSE(agent.hasArrived(std::nullopt));

    agent.incrementDistance(99.0);
    CHECK_FALSE(agent.hasArrived(std::nullopt));

    agent.incrementDistance(1.0);
    CHECK(agent.hasArrived(std::nullopt));

    // Test Max Time
    // Reset agent
    agent = Agent{1, 10};
    agent.setMaxTime(50);  // Max duration 50. Should expire at 10+50=60.

    // Before expiration
    CHECK_FALSE(agent.hasArrived(59));
    // At expiration
    CHECK(agent.hasArrived(60));
    // After expiration
    CHECK(agent.hasArrived(61));

    // Agents with a destination arrive there, not by time or distance
    Agent agentWithDestination{2, 10, dsf::Id{1}};
    CHECK_FALSE(agentWithDestination.hasArrived(1000));
  }
}
