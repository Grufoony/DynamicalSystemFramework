#include "dsf/dsf.hpp"
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using unit = uint32_t;

using RoadNetwork = dsf::mobility::RoadNetwork;
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
  dynamics.setDestinations({118, 115, 112, 109});
  dynamics.setPathRandomness(0.3);

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
