#include <dsf/dsf.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <format>
#include <filesystem>
namespace fs = std::filesystem;

// uncomment these lines to print densities, flows and speeds
#define PRINT_DENSITIES

using Street = dsf::mobility::Street;
using Roundabout = dsf::mobility::Roundabout;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main(int argc, char** argv) {
  auto const start = std::chrono::high_resolution_clock::now();
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " <SEED> <ERROR_PROBABILITY> <OUT_FOLDER_BASE> <INIT_NAGENTS>\n";
    return 1;
  }

  const int SEED = std::stoi(argv[1]);  // seed for random number generator
  const double ERROR_PROBABILITY{std::stod(argv[2])};
  const std::string BASE_OUT_FOLDER{argv[3]};
  auto nAgents{std::stoul(argv[4])};

  std::cout << "-------------------------------------------------\n";
  std::cout << "Input parameters:\n";
  std::cout << "Seed: " << SEED << '\n';
  std::cout << "Error probability: " << ERROR_PROBABILITY << '\n';
  std::cout << "Base output folder: " << BASE_OUT_FOLDER << '\n';
  std::cout << "Initial number of agents: " << nAgents << '\n';
  std::cout << "-------------------------------------------------\n";

  const std::string OUT_FOLDER{std::format("{}output_scrb_{}_{}/",
                                           BASE_OUT_FOLDER,
                                           ERROR_PROBABILITY,
                                           std::to_string(SEED))};  // output folder
  constexpr auto MAX_TIME{static_cast<unsigned int>(5e4)};  // maximum time of simulation

  // Create output folder if it doesn't exist (preserve existing database)
  if (!fs::exists(BASE_OUT_FOLDER)) {
    fs::create_directory(BASE_OUT_FOLDER);
  }
  if (!fs::exists(OUT_FOLDER)) {
    fs::create_directory(OUT_FOLDER);
  }
  // Starting
  std::cout << "Using dsf version: " << dsf::version() << '\n';
  dsf::mobility::TrafficSimulator simulator{};
  std::cout << "Importing Manhattan-like network...\n";
  simulator.importRoadNetwork("../test/data/manhattan_edges.csv",
                              "../test/data/manhattan_nodes.csv");
  std::cout << "Setting street parameters..." << '\n';
  auto& graph = simulator.dynamics()->graph();

  std::cout << "Number of nodes: " << graph.nNodes() << '\n';
  std::cout << "Number of streets: " << graph.nEdges() << '\n';

  std::cout << "Rounding the simulation...\n";
  for (std::size_t i{0}; i < graph.nNodes(); ++i) {
    graph.makeRoundabout(i);
  }
  std::cout << "Add a coil on every street...\n";
  for (const auto& [streetId, pStreet] : graph.edges()) {
    graph.addCoil(streetId);
  }
  graph.adjustNodeCapacities();
  std::cout << "Done." << std::endl;

  std::cout << "Creating dynamics...\n";

  auto* dynamics = simulator.dynamics();
  dynamics->setSeed(SEED);
  {
    std::vector<dsf::Id> destinationNodes;
    for (auto const& [nodeId, pNode] : dynamics->graph().nodes()) {
      if (pNode->outgoingEdges().size() < 4) {
        destinationNodes.push_back(nodeId);
      }
    }
    dynamics->setDestinationNodes(destinationNodes);
    std::cout << "Number of exits: " << destinationNodes.size() << '\n';
  }
  dynamics->updatePaths();

  dynamics->setErrorProbability(ERROR_PROBABILITY);
  dynamics->setPassageProbability(0.7707);
  dynamics->killStagnantAgents(40.);
  // dynamics->setForcePriorities(true);

  // Connect database for saving data
  simulator.connectDataBase(OUT_FOLDER + "simulation_data.db");

  simulator.setAgentInsertionMethod(dsf::mobility::AgentInsertionMethod::UNIFORM);

  // Configure data saving: interval=10, saveAverageStats=true, saveStreetData=true
#ifdef PRINT_DENSITIES
  simulator.saveData(300, true, true, false);
#else
  simulator.saveData(300, true, false, false);
#endif

  std::cout << "Done." << std::endl;
  std::cout << "Running simulation...\n";

  simulator.setTimeFrame(0, MAX_TIME);
  simulator.run(nAgents, 60, 2400, 1);

  std::cout << "Total elapsed time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count()
            << " milliseconds\n";
  return 0;
}
