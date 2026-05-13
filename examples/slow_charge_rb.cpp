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

#include <thread>
#ifdef __APPLE__
#define thread_t std::thread
#else
#define thread_t std::jthread
#endif
#include <atomic>

std::atomic<unsigned int> progress{0};
std::atomic<bool> bExitFlag{false};

// uncomment these lines to print densities, flows and speeds
#define PRINT_DENSITIES

using RoadNetwork = dsf::mobility::RoadNetwork;
using Dynamics = dsf::mobility::FirstOrderDynamics;
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
  constexpr auto MAX_TIME{static_cast<unsigned int>(5e5)};  // maximum time of simulation

  // Create output folder if it doesn't exist (preserve existing database)
  if (!fs::exists(BASE_OUT_FOLDER)) {
    fs::create_directory(BASE_OUT_FOLDER);
  }
  if (!fs::exists(OUT_FOLDER)) {
    fs::create_directory(OUT_FOLDER);
  }
  // Starting
  std::cout << "Using dsf version: " << dsf::version() << '\n';
  RoadNetwork graph{};
  std::cout << "Importing Manhattan-like network...\n";
  graph.importEdges("../test/data/manhattan_edges.csv");
  graph.importNodeProperties("../test/data/manhattan_nodes.csv");
  std::cout << "Setting street parameters..." << '\n';

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

  Dynamics dynamics{std::move(graph), false, SEED};
  {
    std::vector<dsf::Id> destinationNodes;
    for (auto const& [nodeId, pNode] : dynamics.graph().nodes()) {
      if (pNode->outgoingEdges().size() < 4) {
        destinationNodes.push_back(nodeId);
      }
    }
    dynamics.setDestinationNodes(destinationNodes);
    std::cout << "Number of exits: " << destinationNodes.size() << '\n';
  }
  dynamics.updatePaths();

  dynamics.setErrorProbability(0.05);
  dynamics.setPassageProbability(0.7707);
  // dynamics.setForcePriorities(true);

  // Connect database for saving data
  dynamics.connectDataBase(OUT_FOLDER + "simulation_data.db");

  // Configure data saving: interval=10, saveAverageStats=true, saveStreetData=true
#ifdef PRINT_DENSITIES
  dynamics.saveData(300, true, true, false);
#else
  dynamics.saveData(300, true, false, false);
#endif

  std::cout << "Done." << std::endl;
  std::cout << "Running simulation...\n";

  int deltaAgents{std::numeric_limits<int>::max()};
  int previousAgents{0};
  // std::vector<int> deltas;

  // lauch progress bar
  thread_t t([]() {
    while (progress < MAX_TIME && !bExitFlag) {
      printLoadingBar(progress, MAX_TIME);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
  });
  // dynamics.addAgentsUniformly(20000);
  while (dynamics.time_step() < MAX_TIME) {
    if (dynamics.time_step() < MAX_TIME && dynamics.time_step() % 60 == 0) {
      try {
        dynamics.addAgentsUniformly(nAgents);
      } catch (const std::overflow_error& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Overflow reached. Exiting the simulation..." << std::endl;
        bExitFlag = true;
        break;
      }
    }
    dynamics.evolve();

    if (dynamics.time_step() % 2400 == 0) {
      auto const totalDynamicsAgents{dynamics.nAgents()};
      deltaAgents = totalDynamicsAgents - previousAgents;
      if (deltaAgents < 0) {
        ++nAgents;
        std::cout << "- Now I'm adding " << nAgents << " agents.\n";
        std::cout << "Delta agents: " << deltaAgents << '\n';
        std::cout << "At time: " << dynamics.time_step() << '\n';
      }
      previousAgents = totalDynamicsAgents;
    }

    if (dynamics.time_step() % 300 == 0) {
      // Data is now saved automatically by saveData() configuration
      printLoadingBar(dynamics.time_step(), MAX_TIME);
    }
    // Street densities are now saved automatically by saveData() configuration
    ++progress;
  }
  // std::cout << std::endl;
  // std::map<uint8_t, std::string> turnNames{
  //     {0, "left"}, {1, "straight"}, {2, "right"}, {3, "u-turn"}};
  // const auto prob = dynamics.turnProbabilities();
  // uint8_t i{0};
  // for (auto value: prob) {
  //   std::cout << "Probability of turning " << std::quoted(turnNames[i]) << ": " << value * 100 << "%\n";
  //   ++i;
  // }
  std::cout << '\n';
  std::cout << "Done." << std::endl;

#ifdef __APPLE__
  t.join();
#endif
  std::cout << "Total elapsed time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count()
            << " milliseconds\n";
  return 0;
}
