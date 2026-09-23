#include <cstdint>
#include <format>
#include <string>

#include "dsf/mobility/Agent.hpp"
#include "dsf/mobility/Itinerary.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using Agent = dsf::mobility::Agent;
using Itinerary = dsf::mobility::Itinerary;

TEST_CASE("Agent") {
  SUBCASE("Constructors") {
    GIVEN("An agent and its itinerary") {
      auto itinerary = std::make_shared<Itinerary>(0, 5);
      WHEN("The Agent is constructed") {
        Agent agent{0, 0, itinerary};
        THEN("The agent and itinerary ids are set correctly") {
          CHECK_EQ(agent.itinerary()->id(), itinerary->id());
          CHECK_FALSE(agent.streetId().has_value());
          CHECK_FALSE(agent.srcNodeId().has_value());
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.freeTime(), 0);
          CHECK_EQ(agent.spawnTime(), 0);
        }
      }
    }
    GIVEN("An agent, its itinerary, and a node id") {
      auto itinerary = std::make_shared<Itinerary>(0, 5);
      dsf::Id srcNodeId{0};
      WHEN("The Agent is constructed") {
        Agent agent{0, 0, itinerary, srcNodeId};
        THEN("The agent and itinerary ids are set correctly") {
          CHECK_EQ(agent.itinerary()->id(), itinerary->id());
          CHECK_FALSE(agent.streetId().has_value());
          CHECK(agent.srcNodeId().has_value());
          CHECK_EQ(agent.srcNodeId().value(), srcNodeId);
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.freeTime(), 0);
          CHECK_EQ(agent.spawnTime(), 0);
        }
      }
    }
    GIVEN("No initinerary ids and no source node id") {
      WHEN("The agent is constructed") {
        auto randomAgent = Agent{0, 0};
        THEN("The agent is a random agent") { CHECK(randomAgent.isRandom()); }
      }
    }
  }
}

TEST_CASE("Agent methods") {
  auto itinerary42 = std::make_shared<Itinerary>(42, 100);
  Agent agent{0, 0, itinerary42, 7};
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
    agent.setNextStreetId(77);
    agent.setStreetId();
    CHECK(agent.streetId().has_value());
    CHECK_EQ(agent.streetId().value(), 77);
  }
  SUBCASE("setSpeed and speed") {
    agent.setSpeed(12.5);
    CHECK_EQ(agent.speed(), 12.5);
  }
  SUBCASE("setFreeTime and freeTime") {
    agent.setFreeTime(1234);
    CHECK_EQ(agent.freeTime(), 1234);
  }
  SUBCASE("incrementDistance and distance") {
    double d0 = agent.distance();
    agent.incrementDistance(5.5);
    CHECK_EQ(agent.distance(), d0 + 5.5);
  }
  SUBCASE("updateItinerary advances index") {
    auto it1 = std::make_shared<Itinerary>(1, 10);
    auto it2 = std::make_shared<Itinerary>(2, 20);
    auto it3 = std::make_shared<Itinerary>(3, 30);
    std::vector<std::shared_ptr<Itinerary>> trip = {it1, it2, it3};
    Agent a2{0, 0, trip, 0};
    CHECK_EQ(a2.itinerary()->id(), 1);
    a2.updateItinerary();
    CHECK_EQ(a2.itinerary()->id(), 2);
    a2.updateItinerary();
    CHECK_EQ(a2.itinerary()->id(), 3);
    a2.updateItinerary();  // Should not go out of bounds
    CHECK_EQ(a2.itinerary()->id(), 3);
  }
  SUBCASE("reset resets state") {
    agent.setSpeed(10.);
    agent.incrementDistance(5.);
    agent.setFreeTime(99);
    agent.setStreetId(88);
    agent.updateItinerary();
    agent.reset(555);
    CHECK_EQ(agent.spawnTime(), 555);
    CHECK_EQ(agent.freeTime(), 0);
    CHECK_EQ(agent.speed(), 0.);
    CHECK_EQ(agent.distance(), 0.);
    CHECK_EQ(agent.itinerary()->id(), 42);  // itinerary index reset
    CHECK_FALSE(agent.streetId().has_value());
    CHECK_EQ(agent.trip().size(), 1);
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
    auto itinerary42 = std::make_shared<Itinerary>(42, 100);
    Agent agent{0, 0, itinerary42, 7};
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
    CHECK(formatted.find("itineraryId: 42") != std::string::npos);
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
    CHECK(formatted.find("itineraryId: RANDOM") != std::string::npos);
    CHECK(formatted.find("speed: 0.00 m/s") != std::string::npos);
    CHECK(formatted.find("distance: 0.00 m") != std::string::npos);
    CHECK(formatted.find("spawnTime: 5") != std::string::npos);
    CHECK(formatted.find("freeTime: 0") != std::string::npos);
  }

  SUBCASE("std::format with agent with optional nullopts") {
    auto itinerary99 = std::make_shared<Itinerary>(99, 200);
    Agent agent{0, 10, itinerary99};

    std::string formatted = std::format("{}", agent);
    CHECK(formatted.find("id: 0") != std::string::npos);
    CHECK(formatted.find("streetId: N/A") != std::string::npos);
    CHECK(formatted.find("srcNodeId: N/A") != std::string::npos);
    CHECK(formatted.find("nextStreetId: N/A") != std::string::npos);
    CHECK(formatted.find("itineraryId: 99") != std::string::npos);
  }
  SUBCASE("hasArrived") {
    Agent agent{0, 10};  // spawnTime = 10
    CHECK(agent.isRandom());

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
  }
}
