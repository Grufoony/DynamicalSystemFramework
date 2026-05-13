#include "dsf/mobility/TrafficSimulator.hpp"

#include <SQLiteCpp/SQLiteCpp.h>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

using namespace dsf;
using namespace dsf::mobility;

namespace {
  std::filesystem::path makeUniquePath(std::string const& prefix,
                                       std::string const& suffix) {
    auto const stamp = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
    return std::filesystem::current_path() /
           std::filesystem::path(prefix + std::to_string(stamp) + suffix);
  }

  std::filesystem::path makeUniqueDirectory(std::string const& prefix) {
    auto const dir = makeUniquePath(prefix, "");
    std::filesystem::create_directories(dir);
    return dir;
  }

  void writeTinyEdgesCsv(std::filesystem::path const& filePath) {
    std::ofstream out(filePath);
    REQUIRE(out.is_open());
    out << "id;source;target;length;maxspeed;name;type;nlanes\n";
    out << "0;0;1;13.8888888889;50;edge_0;residential;1\n";
    out << "1;1;0;13.8888888889;50;edge_1;residential;1\n";
  }

  int rowCount(SQLite::Database& db, std::string const& tableName) {
    SQLite::Statement query(db, "SELECT COUNT(*) FROM " + tableName);
    REQUIRE(query.executeStep());
    return query.getColumn(0).getInt();
  }
}  // namespace

TEST_CASE("TrafficSimulator configuration") {
  TrafficSimulator simulator;

  CHECK(simulator.database() == nullptr);
  CHECK(simulator.dynamics() == nullptr);

  simulator.setTimeFrame(10, 16);
  CHECK_EQ(simulator.initTime(), 10);
  CHECK_EQ(simulator.endTime(), 16);

  simulator.setNAgentsPerTimeStep({1, 2, 3}, 4);
  CHECK_EQ(simulator.agentInsertionDeltaT(), 4);
  CHECK_EQ(simulator.endTime(), 22);
}

TEST_CASE("TrafficSimulator output prefix") {
  auto const outputDir = makeUniqueDirectory("traffic_simulator_output_");
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_prefix_test");
  simulator.setOutputPrefix(outputDir.string());
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
  simulator.dynamics()->updatePaths();

  simulator.saveData(1, true, true, false, false);
  simulator.setTimeFrame(0, 6);
  simulator.setNAgentsPerTimeStep({1, 0, 0, 0, 0, 0});
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(false);

  auto const roadCsv = outputDir / "road_data.csv";
  auto const avgCsv = outputDir / "avg_stats.csv";

  REQUIRE(std::filesystem::exists(roadCsv));
  REQUIRE(std::filesystem::exists(avgCsv));

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(roadCsv);
  std::filesystem::remove(avgCsv);
  std::filesystem::remove(outputDir);
}

TEST_CASE("TrafficSimulator SQL persistence") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  auto const dbPath = makeUniquePath("traffic_simulator_", ".db");
  writeTinyEdgesCsv(edgesPath);
  std::filesystem::remove(dbPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_sql_test");
  simulator.connectDataBase(dbPath.string());
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
  simulator.dynamics()->updatePaths();

  simulator.saveData(1, true, true, false, false);
  simulator.setTimeFrame(0, 6);
  simulator.setNAgentsPerTimeStep({1, 0, 0, 0, 0, 0});
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(false);

  SQLite::Database db(dbPath.string(), SQLite::OPEN_READONLY);
  CHECK(rowCount(db, "edges") == 2);
  CHECK(rowCount(db, "nodes") == 2);
  CHECK(rowCount(db, "road_data") > 0);
  CHECK(rowCount(db, "avg_stats") > 0);

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(dbPath);
  // Remove eventually generated .db-wal and .db-shm files
  std::filesystem::remove(dbPath.string() + "-wal");
  std::filesystem::remove(dbPath.string() + "-shm");
}

TEST_CASE("TrafficSimulator CSV persistence") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_csv_test");
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
  simulator.dynamics()->updatePaths();

  simulator.saveData(1, true, true, false, false);
  simulator.setTimeFrame(0, 6);
  simulator.setNAgentsPerTimeStep({1, 0, 0, 0, 0, 0});
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(false);

  auto const baseName = std::to_string(static_cast<std::uint64_t>(simulator.id())) +
                        "_traffic_simulator_csv_test";
  auto const roadCsv = std::filesystem::current_path() / (baseName + "_road_data.csv");
  auto const avgCsv = std::filesystem::current_path() / (baseName + "_avg_stats.csv");

  REQUIRE(std::filesystem::exists(roadCsv));
  REQUIRE(std::filesystem::exists(avgCsv));

  {
    std::ifstream roadFile(roadCsv);
    REQUIRE(roadFile.is_open());
    std::string header;
    REQUIRE(std::getline(roadFile, header));
    CHECK_EQ(header,
             "datetime;time_step;street_id;coil;density_vpk;avg_speed_kph;std_speed_kph;"
             "n_observations;counts;queue_length");
  }

  {
    std::ifstream avgFile(avgCsv);
    REQUIRE(avgFile.is_open());
    std::string header;
    REQUIRE(std::getline(avgFile, header));
    CHECK_EQ(header,
             "datetime;time_step;n_ghost_agents;n_agents;mean_speed_kph;std_speed_kph;"
             "mean_density_vpk;std_density_vpk;mean_travel_time_s;mean_queue_length");
  }

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(roadCsv);
  std::filesystem::remove(avgCsv);
}