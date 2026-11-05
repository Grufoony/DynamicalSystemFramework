#include "dsf/mobility/RoadNetwork.hpp"
#include "dsf/mobility/Router.hpp"
#include "dsf/mobility/Street.hpp"
#include "dsf/geometry/Point.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <filesystem>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf;
using namespace dsf::mobility;

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "data";

namespace {
  // Reference cost of the best route from a street to a node: Dijkstra on the streets,
  // with the network's turn rules
  double bestCost(RoadNetwork const& network,
                  Id const fromStreetId,
                  Id const goalNodeId,
                  std::function<double(Street const&)> const& cost) {
    std::unordered_map<Id, double> distances{{fromStreetId, 0.}};
    using Entry = std::pair<double, Id>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;
    queue.push({0., fromStreetId});
    while (!queue.empty()) {
      auto const [distance, streetId] = queue.top();
      queue.pop();
      if (distance > distances.at(streetId)) {
        continue;
      }
      auto const& street{network.edge(streetId)};
      if (street.target()->id() == goalNodeId) {
        return distance;
      }
      for (auto const* pNext : street.target()->outgoingEdges()) {
        if (!pNext->isActive() || !RoadNetwork::isTurnAllowed(street, *pNext)) {
          continue;
        }
        auto const nextDistance{distance + cost(*pNext)};
        auto const it{distances.find(pNext->id())};
        if (it == distances.cend() || nextDistance < it->second) {
          distances[pNext->id()] = nextDistance;
          queue.push({nextDistance, pNext->id()});
        }
      }
    }
    return std::numeric_limits<double>::infinity();
  }

  // Whether an agent can travel a route: open streets, allowed turns, ending at the goal
  bool isValidRoute(RoadNetwork const& network,
                    std::vector<Id> const& route,
                    Id const goalNodeId) {
    if (route.empty()) {
      return false;
    }
    for (std::size_t i{1}; i < route.size(); ++i) {
      auto const& from{network.edge(route[i - 1])};
      auto const& to{network.edge(route[i])};
      if (from.target() != to.source() || !to.isActive() ||
          !RoadNetwork::isTurnAllowed(from, to)) {
        return false;
      }
    }
    return network.edge(route.back()).target()->id() == goalNodeId;
  }

  double freeFlowTime(Street const& street) {
    return street.length() / street.maxSpeed();
  }

  // Three branches from node 0 to node 2, entered from street 5 (5 -> 0): 0 -> 1 -> 2 and
  // 0 -> 3 -> 2 of 10 m, 0 -> 6 -> 2 of 12 m and 0 -> 7 -> 2 of 20 m. Street 4 leaves node 2.
  RoadNetwork makeBranches() {
    RoadNetwork network;
    network.setEdgeWeight("length");
    network.addStreet(0, std::make_pair(0, 1), 5.);
    network.addStreet(1, std::make_pair(1, 2), 5.);
    network.addStreet(2, std::make_pair(0, 3), 5.);
    network.addStreet(3, std::make_pair(3, 2), 5.);
    network.addStreet(4, std::make_pair(2, 4), 5.);
    network.addStreet(5, std::make_pair(5, 0), 5.);
    network.addStreet(6, std::make_pair(0, 6), 6.);
    network.addStreet(7, std::make_pair(6, 2), 6.);
    network.addStreet(8, std::make_pair(0, 7), 10.);
    network.addStreet(9, std::make_pair(7, 2), 10.);
    return network;
  }
}  // namespace

TEST_CASE("Router") {
  SUBCASE("Routes") {
    GIVEN("A short (0->1->2) and a long (0->3->2) branch, without coordinates") {
      RoadNetwork network;
      network.addStreet(0, std::make_pair(0, 1), 100.);
      network.addStreet(1, std::make_pair(1, 2), 100.);
      network.addStreet(2, std::make_pair(0, 3), 120.);
      network.addStreet(3, std::make_pair(3, 2), 120.);
      network.addStreet(4, std::make_pair(2, 4), 10.);
      network.addStreet(5, std::make_pair(5, 0), 10.);
      Router router;
      CHECK_FALSE(router.isBuilt());
      router.build(network);
      THEN("The router is built without heuristic") {
        CHECK(router.isBuilt());
        CHECK_EQ(router.nEdges(), 6);
        CHECK_FALSE(router.hasHeuristic());
      }
      THEN("A route ends on the first street reaching the goal node") {
        auto const route{router.route(5, 2, RouteWeights::FREE_FLOW)};
        CHECK_EQ(route, std::vector<Id>{5, 0, 1});
        // The cost of the first street, which the agent is on, does not count
        CHECK_EQ(router.routeCost(route, RouteWeights::FREE_FLOW),
                 doctest::Approx(200. / network.edge(0).maxSpeed()));
        CHECK_EQ(router.route(1, 2, RouteWeights::FREE_FLOW), std::vector<Id>{1});
      }
      THEN("A route from a node also chooses the first street") {
        CHECK_EQ(router.routeFromNode(0, 2, RouteWeights::FREE_FLOW),
                 std::vector<Id>{0, 1});
        CHECK_EQ(router.routeFromNode(5, 2, RouteWeights::FREE_FLOW),
                 std::vector<Id>{5, 0, 1});
      }
      THEN("A node which cannot be reached gives an empty route") {
        CHECK(router.route(4, 2, RouteWeights::FREE_FLOW).empty());
        CHECK(router.route(0, 5, RouteWeights::FREE_FLOW).empty());
        CHECK(router.routeFromNode(4, 2, RouteWeights::FREE_FLOW).empty());
      }
      THEN("Unknown streets and nodes throw") {
        CHECK_THROWS_AS(router.route(42, 2, RouteWeights::FREE_FLOW), std::out_of_range);
        CHECK_THROWS_AS(router.route(5, 42, RouteWeights::FREE_FLOW), std::out_of_range);
        CHECK_THROWS_AS(router.routeFromNode(42, 2, RouteWeights::FREE_FLOW),
                        std::out_of_range);
        CHECK_THROWS_AS(router.routeCost(std::vector<Id>{5, 42}, RouteWeights::FREE_FLOW),
                        std::out_of_range);
      }
      THEN("Only the turns of the routing graph are allowed") {
        CHECK(router.isTurnAllowed(5, 0));
        CHECK(router.isTurnAllowed(5, 2));
        CHECK_FALSE(router.isTurnAllowed(5, 1));
        CHECK_FALSE(router.isTurnAllowed(42, 0));
        CHECK_FALSE(router.isTurnAllowed(5, 42));
      }
      WHEN("A street is added") {
        network.addStreet(6, std::make_pair(2, 5), 10.);
        THEN("The router knows it only once rebuilt") {
          CHECK_THROWS_AS(router.route(6, 0, RouteWeights::FREE_FLOW), std::out_of_range);
          router.build(network);
          CHECK_EQ(router.nEdges(), 7);
          CHECK_EQ(router.route(6, 0, RouteWeights::FREE_FLOW), std::vector<Id>{6, 5});
        }
      }
    }
    GIVEN("A ring of three streets") {
      RoadNetwork network;
      network.addStreet(0, std::make_pair(0, 1), 10.);
      network.addStreet(1, std::make_pair(1, 2), 10.);
      network.addStreet(2, std::make_pair(2, 0), 10.);
      Router router;
      router.build(network);
      THEN("The route from a street to its own source node goes round the ring") {
        CHECK_EQ(router.route(0, 0, RouteWeights::FREE_FLOW), std::vector<Id>{0, 1, 2});
      }
    }
  }
  SUBCASE("Turn rules") {
    GIVEN("A direct street (1->2) and a detour (1->3->2) after street 0") {
      RoadNetwork network;
      network.addStreet(0, std::make_pair(0, 1), 10.);
      network.addStreet(1, std::make_pair(1, 2), 10.);
      network.addStreet(2, std::make_pair(1, 3), 10.);
      network.addStreet(3, std::make_pair(3, 2), 10.);
      network.addStreet(4, std::make_pair(2, 4), 10.);
      Router router;
      WHEN("The turn from street 0 into street 1 is forbidden") {
        network.edge(0).addForbiddenTurn(1);
        router.build(network);
        THEN("The route takes the detour") {
          CHECK_FALSE(router.isTurnAllowed(0, 1));
          CHECK_EQ(router.route(0, 2, RouteWeights::FREE_FLOW), std::vector<Id>{0, 2, 3});
        }
      }
      WHEN("The turn from street 1 into the next street 4 is forbidden") {
        network.edge(1).addForbiddenTurn(4);
        router.build(network);
        THEN("The route still arrives at node 2 through street 1") {
          // Agents arrive at the node: the turns out of it do not matter
          CHECK_FALSE(router.isTurnAllowed(1, 4));
          CHECK_EQ(router.route(0, 2, RouteWeights::FREE_FLOW), std::vector<Id>{0, 1});
        }
      }
      WHEN("Street 1 is closed") {
        network.setStreetStatusById(1, RoadStatus::CLOSED);
        router.build(network);
        THEN("The route takes the detour") {
          CHECK_FALSE(router.isTurnAllowed(0, 1));
          CHECK_EQ(router.route(0, 2, RouteWeights::FREE_FLOW), std::vector<Id>{0, 2, 3});
        }
        THEN("An agent on the closed street can leave it") {
          CHECK_EQ(router.route(1, 4, RouteWeights::FREE_FLOW), std::vector<Id>{1, 4});
        }
      }
    }
    GIVEN("A street 0 (0->1) whose only way out of node 1 is the U-turn 1->0") {
      RoadNetwork network;
      network.addStreet(0, std::make_pair(0, 1), 10.);
      network.addStreet(1, std::make_pair(1, 0), 10.);
      Router router;
      router.build(network);
      THEN("The U-turn is allowed at the dead end") {
        CHECK_EQ(router.route(0, 0, RouteWeights::FREE_FLOW), std::vector<Id>{0, 1});
      }
      WHEN("A longer way back (1->2->0) is added") {
        network.addStreet(2, std::make_pair(1, 2), 10.);
        network.addStreet(3, std::make_pair(2, 0), 10.);
        router.build(network);
        THEN("The U-turn is forbidden") {
          CHECK_EQ(router.route(0, 0, RouteWeights::FREE_FLOW), std::vector<Id>{0, 2, 3});
        }
        AND_WHEN("Node 1 becomes a roundabout") {
          network.makeRoundabout(1);
          router.build(network);
          THEN("The U-turn is allowed again") {
            CHECK_EQ(router.route(0, 0, RouteWeights::FREE_FLOW), std::vector<Id>{0, 1});
          }
        }
      }
    }
  }
  SUBCASE("Heuristic") {
    GIVEN("The Manhattan network, with coordinates") {
      RoadNetwork network;
      network.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
      network.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());
      network.setEdgeWeight("length");
      Router router;
      router.build(network);
      std::vector<Id> streetIds, nodeIds;
      for (auto const& [streetId, pStreet] : network.edges()) {
        streetIds.push_back(streetId);
      }
      for (auto const& [nodeId, pNode] : network.nodes()) {
        nodeIds.push_back(nodeId);
      }
      std::ranges::sort(streetIds);
      std::ranges::sort(nodeIds);
      REQUIRE(router.hasHeuristic());
      THEN("Yen's first route is the shortest and the next ones cost no less") {
        for (std::size_t k{0}; k < 30; ++k) {
          auto const fromStreetId{streetIds[(37 * k) % streetIds.size()]};
          auto const goalNodeId{nodeIds[(53 * k + 11) % nodeIds.size()]};
          auto const routes{router.kShortestRoutes(
              fromStreetId, goalNodeId, RouteWeights::FREE_FLOW, 4)};
          auto const best{bestCost(network, fromStreetId, goalNodeId, freeFlowTime)};
          if (best == std::numeric_limits<double>::infinity()) {
            CHECK(routes.empty());
            continue;
          }
          REQUIRE_FALSE(routes.empty());
          CHECK_EQ(routes.front().cost, doctest::Approx(best));
          std::set<std::vector<Id>> distinct;
          for (std::size_t i{0}; i < routes.size(); ++i) {
            CHECK(isValidRoute(network, routes[i].streets, goalNodeId));
            CHECK_EQ(router.routeCost(routes[i].streets, RouteWeights::FREE_FLOW),
                     doctest::Approx(routes[i].cost));
            if (i > 0) {
              CHECK_GE(routes[i].cost, routes[i - 1].cost - 1e-9);
            }
            distinct.insert(routes[i].streets);
          }
          CHECK_EQ(distinct.size(), routes.size());
        }
      }
      THEN("A* finds routes as good as Dijkstra's, for both street costs") {
        auto const length = [](Street const& street) { return street.length(); };
        std::size_t nRoutes{0};
        for (std::size_t k{0}; k < 100; ++k) {
          auto const fromStreetId{streetIds[(37 * k) % streetIds.size()]};
          auto const goalNodeId{nodeIds[(53 * k + 11) % nodeIds.size()]};
          auto const freeFlowCost{
              bestCost(network, fromStreetId, goalNodeId, freeFlowTime)};
          auto const freeFlowRoute{
              router.route(fromStreetId, goalNodeId, RouteWeights::FREE_FLOW)};
          auto const lengthRoute{
              router.route(fromStreetId, goalNodeId, RouteWeights::CURRENT)};
          if (freeFlowCost == std::numeric_limits<double>::infinity()) {
            CHECK(freeFlowRoute.empty());
            CHECK(lengthRoute.empty());
            continue;
          }
          ++nRoutes;
          CHECK(isValidRoute(network, freeFlowRoute, goalNodeId));
          CHECK_EQ(router.routeCost(freeFlowRoute, RouteWeights::FREE_FLOW),
                   doctest::Approx(freeFlowCost));
          CHECK(isValidRoute(network, lengthRoute, goalNodeId));
          CHECK_EQ(router.routeCost(lengthRoute, RouteWeights::CURRENT),
                   doctest::Approx(bestCost(network, fromStreetId, goalNodeId, length)));
        }
        CHECK_GT(nRoutes, 90);
      }
    }
    GIVEN("A cheap route moving away from the goal and a costly straight one") {
      // From street 5 (5 -> 0) to node 2: 0 -> 1 -> 2 is straight but 5 m long, while
      // 0 -> 3 -> 4 -> 2 goes round through (0, 1) and (2, 1) but is 4 m long
      RoadNetwork network;
      network.addNode<Intersection>(5, dsf::geometry::Point(-1., 0.));
      network.addNode<Intersection>(0, dsf::geometry::Point(0., 0.));
      network.addNode<Intersection>(1, dsf::geometry::Point(1., 0.));
      network.addNode<Intersection>(2, dsf::geometry::Point(2., 0.));
      network.addNode<Intersection>(3, dsf::geometry::Point(0., 1.));
      network.addNode<Intersection>(4, dsf::geometry::Point(2., 1.));
      network.addStreet(5, std::make_pair(5, 0), 1.);
      network.addStreet(0, std::make_pair(0, 1), 2.5);
      network.addStreet(1, std::make_pair(1, 2), 2.5);
      network.addStreet(2, std::make_pair(0, 3), 1.);
      network.addStreet(3, std::make_pair(3, 4), 2.);
      network.addStreet(4, std::make_pair(4, 2), 1.);
      Router router;
      router.build(network);
      THEN("The heuristic does not hide the cheaper detour") {
        REQUIRE(router.hasHeuristic());
        CHECK_EQ(router.route(5, 2, RouteWeights::FREE_FLOW),
                 std::vector<Id>{5, 2, 3, 4});
      }
    }
    GIVEN("Coordinates much farther apart than the street lengths") {
      // The direct street 1 -> 2 is the only straight line to node 2, but by far the longest
      RoadNetwork network;
      network.addNode<Intersection>(0, dsf::geometry::Point(0., 0.));
      network.addNode<Intersection>(1, dsf::geometry::Point(1., 0.));
      network.addNode<Intersection>(2, dsf::geometry::Point(2., 0.));
      network.addNode<Intersection>(3, dsf::geometry::Point(1., 1.));
      network.addStreet(0, std::make_pair(0, 1), 10.);
      network.addStreet(1, std::make_pair(1, 2), 1000.);
      network.addStreet(2, std::make_pair(1, 3), 10.);
      network.addStreet(3, std::make_pair(3, 2), 10.);
      // An isolated node without coordinates does not matter
      network.addNode<Intersection>(99);
      Router router;
      router.build(network);
      THEN("The heuristic does not overestimate and the route is optimal") {
        REQUIRE(router.hasHeuristic());
        CHECK_EQ(router.route(0, 2, RouteWeights::FREE_FLOW), std::vector<Id>{0, 2, 3});
      }
      WHEN("A street reaches a node without coordinates") {
        network.addStreet(4, std::make_pair(2, 98), 10.);
        router.build(network);
        THEN("The search runs without heuristic") {
          CHECK_FALSE(router.hasHeuristic());
          CHECK_EQ(router.route(0, 2, RouteWeights::FREE_FLOW), std::vector<Id>{0, 2, 3});
        }
      }
    }
  }
  SUBCASE("Current weights") {
    GIVEN("A network routed on a custom street cost") {
      RoadNetwork network{makeBranches()};
      network.setEdgeWeight("cost");
      // Branch costs: 0 -> 1 -> 2 costs 2, 0 -> 3 -> 2 101, 0 -> 6 -> 2 4, 0 -> 7 -> 2 10
      std::unordered_map<Id, double> const costs{{0, 1.},
                                                 {1, 1.},
                                                 {2, 100.},
                                                 {3, 1.},
                                                 {4, 1.},
                                                 {5, 1.},
                                                 {6, 2.},
                                                 {7, 2.},
                                                 {8, 5.},
                                                 {9, 5.}};
      for (auto const& [streetId, cost] : costs) {
        network.edge(streetId).setAttribute("cost", cost);
      }
      Router router;
      router.build(network);
      THEN("The current weights follow the snapshot taken when the router was built") {
        CHECK_EQ(router.route(5, 2, RouteWeights::CURRENT), std::vector<Id>{5, 0, 1});
      }
      WHEN("The cost of the first branch rises") {
        network.edge(0).setAttribute("cost", 100.);
        THEN("The routes change only once the snapshot is updated") {
          CHECK_EQ(router.route(5, 2, RouteWeights::CURRENT), std::vector<Id>{5, 0, 1});
          router.updateCurrentWeights(network);
          CHECK_EQ(router.route(5, 2, RouteWeights::CURRENT), std::vector<Id>{5, 6, 7});
          // The free-flow routes do not depend on the network's weight
          CHECK_EQ(router.route(5, 2, RouteWeights::FREE_FLOW), std::vector<Id>{5, 0, 1});
        }
      }
    }
  }
  SUBCASE("K shortest routes") {
    GIVEN("Four branches from street 5 to node 2, of 10, 10, 12 and 20 m") {
      auto const network{makeBranches()};
      Router router;
      router.build(network);
      auto const unit{1. / network.edge(0).maxSpeed()};
      THEN("Yen's algorithm finds the shortest ones, in order of cost") {
        auto const routes{router.kShortestRoutes(5, 2, RouteWeights::FREE_FLOW, 3)};
        REQUIRE_EQ(routes.size(), 3);
        CHECK_EQ(routes[0].streets, std::vector<Id>{5, 0, 1});
        CHECK_EQ(routes[1].streets, std::vector<Id>{5, 2, 3});
        CHECK_EQ(routes[2].streets, std::vector<Id>{5, 6, 7});
        CHECK_EQ(routes[0].cost, doctest::Approx(10. * unit));
        CHECK_EQ(routes[1].cost, doctest::Approx(10. * unit));
        CHECK_EQ(routes[2].cost, doctest::Approx(12. * unit));
      }
      THEN("Asking for more routes than there are gives them all") {
        auto const routes{router.kShortestRoutes(5, 2, RouteWeights::FREE_FLOW, 10)};
        REQUIRE_EQ(routes.size(), 4);
        CHECK_EQ(routes[3].streets, std::vector<Id>{5, 8, 9});
        CHECK_EQ(routes[3].cost, doctest::Approx(20. * unit));
      }
      THEN("A single route is the shortest one") {
        auto const routes{router.kShortestRoutes(5, 2, RouteWeights::FREE_FLOW, 1)};
        REQUIRE_EQ(routes.size(), 1);
        CHECK_EQ(routes[0].streets, router.route(5, 2, RouteWeights::FREE_FLOW));
        CHECK(router.kShortestRoutes(5, 2, RouteWeights::FREE_FLOW, 0).empty());
      }
      THEN("From a node, the routes also differ by their first street") {
        auto const routes{
            router.kShortestRoutesFromNode(0, 2, RouteWeights::FREE_FLOW, 4)};
        REQUIRE_EQ(routes.size(), 4);
        std::set<Id> firstStreets;
        for (auto const& route : routes) {
          firstStreets.insert(route.streets.front());
        }
        CHECK_EQ(firstStreets, std::set<Id>{0, 2, 6, 8});
        // From a node, the cost of the first street counts
        CHECK_EQ(routes[0].cost, doctest::Approx(10. * unit));
        CHECK_EQ(routes[3].cost, doctest::Approx(20. * unit));
      }
      THEN("A goal which cannot be reached gives no route") {
        CHECK(router.kShortestRoutes(4, 2, RouteWeights::FREE_FLOW, 3).empty());
        CHECK(router.kShortestRoutesFromNode(4, 2, RouteWeights::FREE_FLOW, 3).empty());
      }
    }
    GIVEN("A dead end (4->5->4) next to the shortest route") {
      RoadNetwork network;
      network.addStreet(0, std::make_pair(0, 1), 10.);
      network.addStreet(1, std::make_pair(1, 2), 10.);
      network.addStreet(2, std::make_pair(2, 3), 10.);
      network.addStreet(3, std::make_pair(1, 4), 10.);
      network.addStreet(4, std::make_pair(4, 1), 10.);
      network.addStreet(5, std::make_pair(4, 5), 10.);
      network.addStreet(6, std::make_pair(5, 4), 10.);
      Router router;
      router.build(network);
      THEN("The alternative passes by nodes twice, but never travels a street twice") {
        auto const routes{router.kShortestRoutes(0, 2, RouteWeights::FREE_FLOW, 3)};
        REQUIRE_EQ(routes.size(), 2);
        CHECK_EQ(routes[0].streets, std::vector<Id>{0, 1});
        CHECK_EQ(routes[1].streets, std::vector<Id>{0, 3, 5, 6, 4, 1});
      }
    }
  }
  SUBCASE("Route choice") {
    std::vector<Route> const routes{{{0}, 10.}, {{1}, 10.}, {{2}, 12.}, {{3}, 20.}};
    THEN("Routes are chosen with probability proportional to exp(-cost / lowest cost)") {
      std::vector<double> weights;
      double total{0.};
      for (auto const& route : routes) {
        weights.push_back(std::exp(-route.cost / 10.));
        total += weights.back();
      }
      constexpr std::size_t N{100000};
      std::vector<std::size_t> counts(routes.size(), 0);
      for (std::size_t j{0}; j < N; ++j) {
        ++counts[Router::chooseRoute(routes, (static_cast<double>(j) + 0.5) / N)];
      }
      for (std::size_t i{0}; i < routes.size(); ++i) {
        CHECK_EQ(static_cast<double>(counts[i]) / N,
                 doctest::Approx(weights[i] / total).epsilon(1e-3));
      }
    }
    THEN("Degenerate cases pick the cheapest route") {
      CHECK_EQ(Router::chooseRoute(std::span<Route const>{routes}.first(1), 0.99), 0);
      CHECK_EQ(Router::chooseRoute({}, 0.5), 0);
      std::vector<Route> const freeRoutes{{{0}, 1.}, {{1}, 0.}};
      CHECK_EQ(Router::chooseRoute(freeRoutes, 0.5), 1);
    }
  }
}
