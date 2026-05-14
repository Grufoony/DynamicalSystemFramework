#include <dsf/dsf.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
using TrafficLight = dsf::mobility::TrafficLight;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main(int argc, char** argv) {
  auto const start = std::chrono::high_resolution_clock::now();
  if (argc != 6) {
    std::cerr
        << "Usage: " << argv[0]
        << " <SEED> <ERROR_PROBABILITY> <OUT_FOLDER_BASE> <OPTIMIZE> <INIT_NAGENTS>\n";
    return 1;
  }

  const int SEED = std::stoi(argv[1]);  // seed for random number generator
  const double ERROR_PROBABILITY{std::stod(argv[2])};
  std::string BASE_OUT_FOLDER{argv[3]};
  const bool OPTIMIZE{std::string(argv[4]) != std::string("0")};
  BASE_OUT_FOLDER += OPTIMIZE ? "_op/" : "/";
  auto nAgents{std::stoul(argv[5])};

  std::cout << "-------------------------------------------------\n";
  std::cout << "Input parameters:\n";
  std::cout << "Seed: " << SEED << '\n';
  std::cout << "Error probability: " << ERROR_PROBABILITY << '\n';
  std::cout << "Base output folder: " << BASE_OUT_FOLDER << '\n';
  std::cout << "Initial number of agents: " << nAgents << '\n';
  if (OPTIMIZE) {
    std::cout << "Traffic light optimization ENABLED.\n";
  }
  std::cout << "-------------------------------------------------\n";
  const std::string OUT_FOLDER{std::format("{}output_sctl_{}_{}/",
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

  // graph.addStreet(Street(100002, std::make_pair(0, 108)));
  // graph.addStreet(Street(100003, std::make_pair(108, 0)));
  // graph.addStreet(Street(100004, std::make_pair(0, 11)));
  // graph.addStreet(Street(100005, std::make_pair(11, 0)));
  // graph.addStreet(Street(100006, std::make_pair(1, 109)));
  // graph.addStreet(Street(100007, std::make_pair(109, 1)));
  // graph.addStreet(Street(100008, std::make_pair(2, 110)));
  // graph.addStreet(Street(100009, std::make_pair(110, 2)));
  // graph.addStreet(Street(100010, std::make_pair(3, 111)));
  // graph.addStreet(Street(100011, std::make_pair(111, 3)));
  // graph.addStreet(Street(100012, std::make_pair(4, 112)));
  // graph.addStreet(Street(100013, std::make_pair(112, 4)));
  // graph.addStreet(Street(100014, std::make_pair(5, 113)));
  // graph.addStreet(Street(100015, std::make_pair(113, 5)));
  // graph.addStreet(Street(100016, std::make_pair(6, 114)));
  // graph.addStreet(Street(100017, std::make_pair(114, 6)));
  // graph.addStreet(Street(100018, std::make_pair(7, 115)));
  // graph.addStreet(Street(100019, std::make_pair(115, 7)));
  // graph.addStreet(Street(100020, std::make_pair(8, 116)));
  // graph.addStreet(Street(100021, std::make_pair(116, 8)));
  // graph.addStreet(Street(100022, std::make_pair(9, 117)));
  // graph.addStreet(Street(100023, std::make_pair(117, 9)));
  // graph.addStreet(Street(100024, std::make_pair(10, 118)));
  // graph.addStreet(Street(100025, std::make_pair(118, 10)));
  // graph.addStreet(Street(100026, std::make_pair(11, 119)));
  // graph.addStreet(Street(100027, std::make_pair(119, 11)));
  // graph.addStreet(Street(100028, std::make_pair(12, 23)));
  // graph.addStreet(Street(100029, std::make_pair(23, 12)));
  // graph.addStreet(Street(100030, std::make_pair(24, 35)));
  // graph.addStreet(Street(100069, std::make_pair(35, 24)));
  // graph.addStreet(Street(100031, std::make_pair(36, 47)));
  // graph.addStreet(Street(100032, std::make_pair(47, 36)));
  // graph.addStreet(Street(100033, std::make_pair(48, 59)));
  // graph.addStreet(Street(100034, std::make_pair(59, 48)));
  // graph.addStreet(Street(100035, std::make_pair(60, 71)));
  // graph.addStreet(Street(100036, std::make_pair(71, 60)));
  // graph.addStreet(Street(100037, std::make_pair(72, 83)));
  // graph.addStreet(Street(100038, std::make_pair(83, 72)));
  // graph.addStreet(Street(100039, std::make_pair(84, 95)));
  // graph.addStreet(Street(100040, std::make_pair(95, 84)));
  // graph.addStreet(Street(100041, std::make_pair(96, 107)));
  // graph.addStreet(Street(100042, std::make_pair(107, 96)));
  // graph.addStreet(Street(100043, std::make_pair(108, 119)));
  // graph.addStreet(Street(100044, std::make_pair(119, 108)));

  std::cout << "Number of nodes: " << graph.nNodes() << '\n';
  std::cout << "Number of streets: " << graph.nEdges() << '\n';

  std::cout << "Traffic Lightning the simulation...\n";
  for (auto const& [nodeId, pNode] : graph.nodes()) {
    if (pNode->outgoingEdges().size() > 3) {
      graph.makeTrafficLight(nodeId, 120);
    }
  }
  std::cout << "Add a coil on every street...\n";
  for (const auto& pair : graph.edges()) {
    graph.addCoil(pair.first);
  }
  graph.adjustNodeCapacities();
  std::cout << "Setting traffic light parameters..." << '\n';
  graph.autoInitTrafficLights();
  std::cout << "Done." << std::endl;

  std::cout << "Creating dynamics...\n";

  auto* dynamics = simulator.dynamics();
  dynamics->setSeed(SEED);
  {
    std::vector<dsf::Id> destinationNodes;
    for (auto const& [nodeId, pNode] : graph.nodes()) {
      if (pNode->outgoingEdges().size() < 4) {
        destinationNodes.push_back(nodeId);
      }
    }
    dynamics->setDestinationNodes(destinationNodes);
    std::cout << "Number of exits: " << destinationNodes.size() << '\n';
  }
  dynamics->updatePaths();

  dynamics->setErrorProbability(ERROR_PROBABILITY);
  dynamics->killStagnantAgents(40.);
  // dynamics->setMaxFlowPercentage(0.69);
  // dynamics->setForcePriorities(false);
  if (OPTIMIZE)
    dynamics->setDataUpdatePeriod(30);  // Store data every 30 time steps

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
