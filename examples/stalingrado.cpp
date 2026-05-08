/**
 * @brief Main function to simulate traffic dynamics over Via Stalingrado, in Bologna.
 */

#include <dsf/dsf.hpp>
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include <thread>
#include <atomic>

std::atomic<unsigned int> progress{0};

using Unit = unsigned int;
using Delay = uint8_t;

using namespace dsf::mobility;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main() {
  // Import input data
  std::ifstream ifs{"./data/stalingrado_input.txt"};
  Unit timeUnit{0};
  ifs >> timeUnit;
  std::vector<Unit> vehiclesToInsert{};
  while (!ifs.eof()) {
    Unit vehicleId{0};
    ifs >> vehicleId;
    vehiclesToInsert.push_back(vehicleId);
  }
  const auto MAX_TIME{static_cast<Unit>(timeUnit * vehiclesToInsert.size())};

  // Create the graph
  RoadNetwork graph;

  // Street(StreetId, Capacity, Length, vMax, (from, to))
  dsf::mobility::Road::setMeanVehicleLength(8.);
  Street s01{1, std::make_pair(0, 1), 2281., 13.9, 2};
  Street s12{7, std::make_pair(1, 2), 118., 13.9, 2};
  Street s23{13, std::make_pair(2, 3), 222., 13.9, 2};
  Street s34{19, std::make_pair(3, 4), 651., 13.9, 2};
  // Viale Aldo Moro
  graph.addNode<TrafficLight>(1, 132);
  auto& tl1 = graph.node<TrafficLight>(1);
  tl1.setCycle(s01.id(), dsf::Direction::ANY, {62, 0});
  // Via Donato Creti
  graph.addNode<TrafficLight>(2, 141);
  auto& tl2 = graph.node<TrafficLight>(2);
  tl2.setCycle(s12.id(), dsf::Direction::ANY, {72, 0});
  // Via del Lavoro
  graph.addNode<TrafficLight>(3, 138);
  auto& tl3 = graph.node<TrafficLight>(3);
  tl3.setCycle(s23.id(), dsf::Direction::ANY, {88, 0});
  // Viali
  graph.addNode<TrafficLight>(4, 131);
  auto& tl4 = graph.node<TrafficLight>(4);
  tl4.setCycle(s34.id(), dsf::Direction::ANY, {81, 0});

  graph.addStreets(s01, s12, s23, s34);
  graph.adjustNodeCapacities();
  graph.addCoil(19);
  auto& coil = graph.edge(19);

  // Create the dynamics
  FirstOrderDynamics dynamics{std::move(graph), false, 69};
  dynamics.addItinerary(4, 4);
  dynamics.updatePaths();

  auto pItinerary = dynamics.itineraries().at(4);

  // lauch progress bar
  std::jthread t([MAX_TIME]() {
    while (progress < MAX_TIME) {
      printLoadingBar(progress, MAX_TIME);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
  });
  // Evolution
  auto it = vehiclesToInsert.begin();
  std::ofstream ofs{"./stalingrado_output.csv"};
  // print two columns, time and vehicles
  ofs << "time;vehicle_flux" << '\n';
  while (progress < MAX_TIME) {
    if (progress % 60 == 0) {
      if (progress != 0) {
        ++it;
      }
      if (progress % 300 == 0) {
        ofs << progress << ';' << coil.counts() << std::endl;
        coil.resetCounter();
      }
      dynamics.addAgents(*it, pItinerary, 0);
    }
    dynamics.evolve(false);
    ++progress;
  }

  return 0;
}
