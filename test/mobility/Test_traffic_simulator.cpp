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
}

TEST_CASE("TrafficSimulator JSON config parameters") {
  // Prepare unique input and output directories
  auto const inputDir = makeUniqueDirectory("traffic_simulator_cfg_input_");
  auto const outputDir = makeUniqueDirectory("traffic_simulator_cfg_output_");

  // Write tiny edges CSV inside input folder
  auto const edgesPath = inputDir / "edges.csv";
  writeTinyEdgesCsv(edgesPath);
  // Write minimal nodes properties CSV so importNodeProperties can be called safely
  auto const nodesPath = inputDir / "nodes.csv";
  {
    std::ofstream nout(nodesPath);
    REQUIRE(nout.is_open());
    nout << "id;type;geometry\n";
    nout << "0;normal;\n";
    nout << "1;normal;\n";
  }

  // Create JSON configuration file
  auto const jsonPath = makeUniquePath("traffic_simulator_config_", ".json");
  {
    std::ofstream out(jsonPath);
    REQUIRE(out.is_open());
    out << "{\n";
    out << "  \"general\": {\n";
    out << "    \"input_folder\": \"" << inputDir.string() << "\",\n";
    out << "    \"output_folder\": \"" << outputDir.string() << "\",\n";
    out << "    \"output_basename\": \"mybase_\",\n";
    out << "    \"name\": \"cfg_test\",\n";
    out << "    \"init_time\": 10,\n";
    out << "    \"end_time\": 16\n";
    out << "  },\n";
    out << "  \"road_network\": {\n";
    out << "    \"edges_file\": \"edges.csv\",\n";
    out << "    \"node_properties_file\": \"nodes.csv\",\n";
    out << "    \"set_edge_weight\": { \"weight\": \"length\", \"threshold\": 1.0 }\n";
    out << "  },\n";
    out << "  \"dynamics\": {\n";
    out << "    \"agent_insertion_method\": \"ODS\"\n";
    out << "  }\n";
    out << "}\n";
  }

  TrafficSimulator simulator;
  simulator.importConfig(jsonPath.string());

  // init_time and end_time must be read from config
  CHECK_EQ(simulator.initTime(), 10);
  CHECK_EQ(simulator.endTime(), 16);

  // Configure saving and run to flush CSVs using the output_basename
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
  simulator.dynamics()->updatePaths();

  simulator.saveData(1, true, true, false, false);
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0, 0});

  // Expect CSV files named using the output_basename inside the output folder
  auto const roadCsv = outputDir / "mybase_road_data.csv";
  auto const avgCsv = outputDir / "mybase_avg_stats.csv";

  REQUIRE(std::filesystem::exists(roadCsv));
  REQUIRE(std::filesystem::exists(avgCsv));

  // Cleanup
  std::filesystem::remove(edgesPath);
  std::filesystem::remove(nodesPath);
  std::filesystem::remove(jsonPath);
  std::filesystem::remove(roadCsv);
  std::filesystem::remove(avgCsv);
  std::filesystem::remove(inputDir);
  std::filesystem::remove(outputDir);
}

TEST_CASE("TrafficSimulator - dynamic ODs") {
  SUBCASE("TrafficSimulator - dynamic ODs") {
    auto DATA_FOLDER = std::filesystem::current_path().parent_path() / "test" / "data";
    GIVEN("A TrafficSimulator with a valid road network config") {
      WHEN("JSON config contains a valid dynamic_ods array starting at time 0") {
        // Write a minimal JSON config to a temp file
        auto const configPath = (DATA_FOLDER / "dynamic_ods_valid.json").string();
        TrafficSimulator simulator;
        CHECK_NOTHROW(simulator.importConfig(configPath));
      }

      WHEN("JSON config has dynamic_ods whose first entry time is not 0") {
        auto const configPath = (DATA_FOLDER / "dynamic_ods_bad_time.json").string();
        TrafficSimulator simulator;
        simulator.importConfig(configPath);
        // The throw happens at run-time, not at config-import time
        THEN("Running the simulation throws a std::runtime_error") {
          CHECK_THROWS_AS(simulator.run({10}), std::runtime_error);
        }
      }

      WHEN("JSON config has both dynamic_ods and importODsFromCSV") {
        auto const configPath = (DATA_FOLDER / "dynamic_ods_and_static.json").string();
        TrafficSimulator simulator;
        // Should not throw — dynamic_ods wins but a warning is logged
        CHECK_NOTHROW(simulator.importConfig(configPath));
      }

      WHEN("dynamic_ods array is missing the 'time' field") {
        auto const configPath = (DATA_FOLDER / "dynamic_ods_missing_time.json").string();
        TrafficSimulator simulator;
        THEN("importConfig throws a std::runtime_error") {
          CHECK_THROWS_AS(simulator.importConfig(configPath), std::runtime_error);
        }
      }

      WHEN("dynamic_ods array is missing the 'file' field") {
        auto const configPath = (DATA_FOLDER / "dynamic_ods_missing_file.json").string();
        TrafficSimulator simulator;
        THEN("importConfig throws a std::runtime_error") {
          CHECK_THROWS_AS(simulator.importConfig(configPath), std::runtime_error);
        }
      }

      WHEN("dynamic_ods contains two updates and we run enough steps to trigger both") {
        // Config schedules:
        //   time 0  → ods_phase1.csv  (origin 0 → dest 2)
        //   time 5  → ods_phase2.csv  (origin 1 → dest 14)
        // Total simulation: 10 steps, 1 agent per step
        auto const configPath = (DATA_FOLDER / "dynamic_ods_two_phases.json").string();
        TrafficSimulator simulator;
        simulator.importConfig(configPath);

        CHECK_NOTHROW(simulator.run({1, 1, 1, 1, 1, 1, 1, 1, 1, 1}));

        THEN("The simulator runs to completion without error") {
          // Structural: if we get here both swaps executed cleanly
          CHECK(true);
        }
      }

      WHEN("The OD file referenced in dynamic_ods does not exist") {
        auto const configPath = (DATA_FOLDER / "dynamic_ods_bad_file.json").string();
        TrafficSimulator simulator;
        simulator.importConfig(configPath);

        THEN("Running throws when the missing CSV is loaded") {
          CHECK_THROWS(simulator.run({1, 1, 1}));
        }
      }
    }
  }
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
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0, 0});

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
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0, 0});

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
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0, 0});

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

TEST_CASE("TrafficSimulator agent insertion interval uses timestep conversion") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.importRoadNetwork(edgesPath.string());

  REQUIRE(simulator.dynamics() != nullptr);

  simulator.dynamics()->setTimestepsToSeconds(2.0);

  simulator.setTimeFrame(0, 12);
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);

  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
  simulator.dynamics()->updatePaths();

  // insert one agent every 4 physical seconds
  simulator.run({1, 1, 1}, 4);

  auto counts = simulator.dynamics()->originCounts(false);

  CHECK_EQ(counts.at(0), 3);

  std::filesystem::remove(edgesPath);
}

TEST_CASE("TrafficSimulator CSV datetime respects timestepsToSeconds") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_datetime_test");
  simulator.importRoadNetwork(edgesPath.string());

  REQUIRE(simulator.dynamics() != nullptr);

  simulator.dynamics()->setTimestepsToSeconds(2.0);

  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);

  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});

  simulator.dynamics()->updatePaths();

  simulator.saveData(1, true, false, false, false);

  simulator.setTimeFrame(0, 10);
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);

  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0});

  auto const baseName = std::to_string(static_cast<std::uint64_t>(simulator.id())) +
                        "_traffic_simulator_datetime_test";

  auto const avgCsv = std::filesystem::current_path() / (baseName + "_avg_stats.csv");

  REQUIRE(std::filesystem::exists(avgCsv));

  std::ifstream file(avgCsv);
  REQUIRE(file.is_open());

  std::string line;

  // header
  REQUIRE(std::getline(file, line));

  // first data row
  REQUIRE(std::getline(file, line));

  std::stringstream ss(line);

  std::string datetimeStr;
  std::string timestepStr;

  std::getline(ss, datetimeStr, ';');
  std::getline(ss, timestepStr, ';');

  auto timeToStr = [](std::time_t const time) {
#ifdef __APPLE__
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    return oss.str();
#else
    return std::format(
        "{:%Y-%m-%d %H:%M:%S}",
        std::chrono::floor<std::chrono::seconds>(std::chrono::current_zone()->to_local(
            std::chrono::system_clock::from_time_t(time))));
#endif
  };

  CHECK_EQ(datetimeStr, timeToStr(0));
  CHECK_EQ(std::stoull(timestepStr), 0);

  // second row
  REQUIRE(std::getline(file, line));

  std::stringstream ss2(line);

  std::getline(ss2, datetimeStr, ';');
  std::getline(ss2, timestepStr, ';');

  CHECK_EQ(datetimeStr, timeToStr(2));
  CHECK_EQ(std::stoull(timestepStr), 1);

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(avgCsv);
}