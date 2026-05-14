#include "dsf/dsf.hpp"
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using unit = uint32_t;

using RoadNetwork = dsf::mobility::RoadNetwork;
using Itinerary = dsf::mobility::Itinerary;
using Dynamics = dsf::mobility::FirstOrderDynamics;

int main() {
  RoadNetwork graph{};
  std::cout << "Importing manhattan_edges.csv...\n";
  graph.importEdges("../test/data/manhattan_edges.csv");
  std::cout << "Number of nodes: " << graph.nNodes() << '\n'
            << "Number of streets: " << graph.nEdges() << '\n';
  for (auto const& pair : graph.edges()) {
    pair.second->setCapacity(100);
    pair.second->setMaxSpeed(10.);
  }
  for (auto const& pair : graph.nodes()) {
    pair.second->setCapacity(10);
  }
  std::cout << "Done.\n";

  std::cout << "Creating dynamics...\n";

  Dynamics dynamics{graph, false, std::nullopt, 0.95};
  dynamics.addItinerary(0, 118);
  dynamics.addItinerary(1, 115);
  dynamics.addItinerary(2, 112);
  dynamics.addItinerary(3, 109);
  dynamics.setErrorProbability(0.3);
  dynamics.updatePaths();

  std::cout << "Done.\n"
            << "Running simulation...\n";

  for (unsigned int i{0}; i < 1000; ++i) {
    if (i < 12e3) {
      if (i % 60 == 0) {
        dynamics.addAgentsUniformly(100);
      }
    }
    dynamics.evolve();
  }
  std::cout << '\n' << "Done.\n";

  return 0;
}
