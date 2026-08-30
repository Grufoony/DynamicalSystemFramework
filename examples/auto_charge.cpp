#include <dsf.hpp>
#include <iostream>

int main() {
  using namespace dsf::mobility;
  try {
    TrafficSimulator sim;
    sim.setName("auto_charge_example");
    // Import a small example network (user must provide path)
    sim.importRoadNetwork("examples/data/edges.csv", "");
    sim.setTimeFrame(0, 600);                  // 600 steps
    sim.saveData(60, true, true, true, true);  // save every 60 steps
    // Run auto-charge: base 5 agents every 30 steps, save interval 60, maxSteps=600
    sim.runAutoCharge(
        5, 30, 60, 600, std::nullopt, 120, 120, 2, true, "auto_charge_events.csv");
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
