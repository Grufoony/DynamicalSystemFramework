#include "dsf/mobility/TrafficSimulator.hpp"

#include <SQLiteCpp/SQLiteCpp.h>
#include <csv.hpp>

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
    out << "2;1;2;13.8888888889;50;edge_2;residential;1\n";
    out << "3;2;1;13.8888888889;50;edge_3;residential;1\n";
  }

  void writeFile(std::filesystem::path const& filePath, std::string const& content) {
    std::ofstream out(filePath);
    REQUIRE(out.is_open());
    out << content;
  }

  std::string firstLine(std::filesystem::path const& filePath) {
    std::ifstream in(filePath);
    REQUIRE(in.is_open());
    std::string line;
    REQUIRE(std::getline(in, line));
    return line;
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

TEST_CASE("TrafficSimulator JSON config - transition matrix") {
  auto const inputDir = makeUniqueDirectory("traffic_simulator_tm_input_");
  auto const outputDir = makeUniqueDirectory("traffic_simulator_tm_output_");

  auto const edgesPath = inputDir / "edges.csv";
  writeTinyEdgesCsv(edgesPath);
  auto const nodesPath = inputDir / "nodes.csv";
  {
    std::ofstream nout(nodesPath);
    REQUIRE(nout.is_open());
    nout << "id;type;geometry\n";
    nout << "0;normal;\n";
    nout << "1;normal;\n";
  }
  // Street 0 goes 0 -> 1; the only non-U-turn transition at node 1 is street 2 (1 -> 2)
  auto const matrixPath = inputDir / "transition_matrix.json";
  {
    std::ofstream mout(matrixPath);
    REQUIRE(mout.is_open());
    mout << "{ \"0\": { \"2\": 0.8 } }\n";
  }

  auto const jsonPath = makeUniquePath("traffic_simulator_tm_config_", ".json");
  {
    std::ofstream out(jsonPath);
    REQUIRE(out.is_open());
    out << "{\n";
    out << "  \"general\": {\n";
    out << "    \"input_folder\": \"" << inputDir.string() << "\",\n";
    out << "    \"output_folder\": \"" << outputDir.string() << "\",\n";
    out << "    \"name\": \"tm_test\"\n";
    out << "  },\n";
    out << "  \"road_network\": {\n";
    out << "    \"edges_file\": \"edges.csv\",\n";
    out << "    \"node_properties_file\": \"nodes.csv\",\n";
    out << "    \"set_edge_weight\": { \"weight\": \"length\", \"threshold\": 1.0 }\n";
    out << "  },\n";
    out << "  \"dynamics\": {\n";
    out << "    \"agent_insertion_method\": \"RANDOM\",\n";
    out << "    \"importTransitionMatrixFromJSON\": { \"file\": "
           "\"transition_matrix.json\" }\n";
    out << "  }\n";
    out << "}\n";
  }

  TrafficSimulator simulator;
  simulator.importConfig(jsonPath.string());

  REQUIRE(simulator.dynamics() != nullptr);
  auto const& matrix = simulator.dynamics()->transitionMatrix();
  REQUIRE(matrix.contains(0));
  CHECK_EQ(matrix.size(), 1);
  CHECK_EQ(matrix.at(0).size(), 1);
  CHECK_EQ(matrix.at(0).at(2), doctest::Approx(0.8));

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(nodesPath);
  std::filesystem::remove(matrixPath);
  std::filesystem::remove(jsonPath);
  std::filesystem::remove(inputDir);
  std::filesystem::remove(outputDir);
}

TEST_CASE("TrafficSimulator JSON config - all options") {
  auto const inputDir = makeUniqueDirectory("traffic_simulator_all_input_");
  auto const outputDir = makeUniqueDirectory("traffic_simulator_all_output_");

  // Street 0 carries a coil and node 1 is a roundabout, so both end up in the dump
  writeFile(inputDir / "edges.csv",
            "id;source;target;length;maxspeed;name;type;nlanes;coilcode\n"
            "0;0;1;13.8888888889;50;edge_0;residential;1;C0\n"
            "1;1;0;13.8888888889;50;edge_1;residential;1;\n"
            "2;1;2;13.8888888889;50;edge_2;residential;1;\n"
            "3;2;1;13.8888888889;50;edge_3;residential;1;\n");
  writeFile(inputDir / "nodes.csv",
            "id;type;geometry\n"
            "0;normal;POINT (0 0)\n"
            "1;roundabout;POINT (1 0)\n"
            "2;normal;POINT (2 0)\n");
  // Compat format: an empty probability means the street is not an origin/destination
  writeFile(inputDir / "ods.csv", "id,o_prob,d_prob\n0,1.0,\n3,,1.0\n");

  auto const jsonPath = makeUniquePath("traffic_simulator_all_config_", ".json");
  writeFile(jsonPath,
            R"({
  "general": {
    "input_folder": ")" +
                inputDir.string() +
                R"(",
    "output_folder": ")" +
                outputDir.string() +
                R"(",
    "name": "all options",
    "database": "all_options.db",
    "init_time": "20240101",
    "end_time": "20240101 000020",
    "update_paths": { "interval": 5, "throw_on_empty": false,
                      "intelligent_fraction": 0.5 },
    "save_data": { "interval": 1, "avg": true, "road": true, "travel": true,
                   "agent": true, "turn_counts": true }
  },
  "road_network": {
    "edges_file": "edges.csv",
    "node_properties_file": "nodes.csv",
    "set_edge_weight": { "weight": "length", "threshold": 1.0 }
  },
  "dynamics": {
    "seed": 42,
    "max_concurrency": 2,
    "agent_insertion_method": "RANDOM_ODS",
    "error_probability": 0.05,
    "kill_stagnant_agents": 10.0,
    "mean_travel_distance": 1000.0,
    "mean_travel_time": 600,
    "importODsFromCSV": { "file": "ods.csv", "separator": ",", "edges": true }
  }
})");

  TrafficSimulator simulator{jsonPath.string()};
  CHECK_EQ(simulator.name(), "all options");
  CHECK_EQ(simulator.safeName(), "all_options");
  CHECK_EQ(simulator.strInitTime(), "2024-01-01 00:00:00");
  CHECK_EQ(simulator.strEndTime(), "2024-01-01 00:00:20");
  REQUIRE(simulator.database() != nullptr);
  REQUIRE(simulator.dynamics() != nullptr);
  CHECK_EQ(simulator.dynamics()->origins().size(), 1);
  CHECK_EQ(simulator.dynamics()->destinations().size(), 1);
  CHECK_EQ(simulator.dynamics()->intelligentAgentsFraction(), 0.5);

  simulator.run(std::vector<std::size_t>{2, 2, 2, 2});
  CHECK_EQ(simulator.dynamics()->freeflowItineraries().size(),
           simulator.dynamics()->itineraries().size());

  auto const dbPath = outputDir / "all_options.db";
  {
    SQLite::Database db(dbPath.string(), SQLite::OPEN_READONLY);
    CHECK_GT(rowCount(db, "travel_data"), 0);
    CHECK_GT(rowCount(db, "agent_data"), 0);
    CHECK_GT(rowCount(db, "turn_counts"), 0);
    SQLite::Statement coil(db, "SELECT coilcode FROM edges WHERE id = 0");
    REQUIRE(coil.executeStep());
    CHECK_EQ(coil.getColumn(0).getString(), "c0");
    SQLite::Statement node(db, "SELECT type, geometry FROM nodes WHERE id = 1");
    REQUIRE(node.executeStep());
    CHECK_EQ(node.getColumn(0).getString(), "roundabout");
    CHECK_FALSE(node.getColumn(1).isNull());
  }

  std::filesystem::remove_all(inputDir);
  std::filesystem::remove_all(outputDir);
  std::filesystem::remove(jsonPath);
}

TEST_CASE("TrafficSimulator JSON config errors") {
  auto const inputDir = makeUniqueDirectory("traffic_simulator_err_input_");
  auto const outputDir = makeUniqueDirectory("traffic_simulator_err_output_");
  auto const jsonPath = makeUniquePath("traffic_simulator_err_config_", ".json");
  writeTinyEdgesCsv(inputDir / "edges.csv");
  writeFile(inputDir / "nodes.csv", "id;type;geometry\n0;normal;\n");

  // Builds a config from the body of the "general" section and the other sections
  auto const makeConfig = [&](std::string const& generalExtra,
                              std::string const& otherSections) {
    return R"({ "general": { "input_folder": ")" + inputDir.string() +
           R"(", "output_folder": ")" + outputDir.string() + R"(", "name": "err")" +
           generalExtra + "}" + otherSections + "}";
  };
  std::string const roadNetwork{
      R"(, "road_network": { "edges_file": "edges.csv", "node_properties_file": )"
      R"("nodes.csv", "set_edge_weight": { "weight": "length", "threshold": 1.0 } })"};
  auto const importConfig = [&](std::string const& content) {
    writeFile(jsonPath, content);
    TrafficSimulator simulator;
    simulator.importConfig(jsonPath.string());
  };

  SUBCASE("The file does not exist") {
    CHECK_THROWS_AS(TrafficSimulator{(inputDir / "missing.json").string()},
                    std::runtime_error);
  }
  SUBCASE("The root is not an object") {
    CHECK_THROWS_AS(importConfig("[1, 2]"), std::runtime_error);
  }
  SUBCASE("A required section or field is missing") {
    CHECK_THROWS_AS(importConfig("{}"), std::runtime_error);
    CHECK_THROWS_AS(importConfig(R"({ "general": {} })"), std::runtime_error);
    CHECK_THROWS_AS(importConfig(makeConfig("", "")), std::runtime_error);
    CHECK_THROWS_AS(importConfig(makeConfig("", roadNetwork)), std::runtime_error);
  }
  SUBCASE("A time field is malformed") {
    CHECK_THROWS_AS(importConfig(makeConfig(R"(, "init_time": true)", "")),
                    std::invalid_argument);
    CHECK_THROWS_AS(importConfig(makeConfig(R"(, "init_time": "2024")", "")),
                    std::invalid_argument);
  }
  SUBCASE("dynamic_ods is malformed") {
    CHECK_THROWS_AS(importConfig(makeConfig(R"(, "dynamic_ods": {})", "")),
                    std::runtime_error);
    CHECK_THROWS_AS(importConfig(makeConfig(R"(, "dynamic_ods": [1])", "")),
                    std::runtime_error);
  }
  SUBCASE("The agent insertion method is unknown") {
    CHECK_THROWS_AS(
        importConfig(makeConfig(
            "", roadNetwork + R"(, "dynamics": { "agent_insertion_method": "FOO" })")),
        std::runtime_error);
  }
  SUBCASE("The intelligent fraction is out of range") {
    CHECK_THROWS_AS(
        importConfig(makeConfig(
            R"(, "update_paths": { "interval": 1, "intelligent_fraction": 1.5 })",
            roadNetwork + R"(, "dynamics": { "agent_insertion_method": "ODS" })")),
        std::invalid_argument);
  }
  SUBCASE("Every other agent insertion method is accepted") {
    for (auto const* method : {"CONDITIONAL_RANDOM_ODS", "UNIFORM"}) {
      CHECK_NOTHROW(importConfig(
          makeConfig("",
                     roadNetwork + R"(, "dynamics": { "agent_insertion_method": ")" +
                         method + R"(" })")));
    }
  }

  std::filesystem::remove_all(inputDir);
  std::filesystem::remove_all(outputDir);
  std::filesystem::remove(jsonPath);
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
          CHECK_THROWS_AS(simulator.run(1, 1, 1), std::runtime_error);
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
        CHECK_EQ(simulator.dynamics()->time_step(), 10);
      }

      WHEN("dynamic_ods contains two updates and we run a slow charge") {
        auto const configPath = (DATA_FOLDER / "dynamic_ods_two_phases.json").string();
        TrafficSimulator simulator;
        simulator.importConfig(configPath);
        simulator.setTimeFrame(0, 10);

        CHECK_NOTHROW(simulator.run(1, 1, 5));
        CHECK_EQ(simulator.dynamics()->time_step(), 10);
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
  CHECK(rowCount(db, "edges") == 4);
  CHECK(rowCount(db, "nodes") == 3);
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

TEST_CASE("TrafficSimulator CSV writer semantics") {
  // The CSV artifacts are written with csv::DelimWriter over an appended stream, so
  // two properties matter: the header must be written once for the whole run, and a
  // field holding the separator must be escaped instead of corrupting the row.
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_writer_test");
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
  simulator.dynamics()->updatePaths();
  simulator.dynamics()->graph().edge(0).enableCounter("coil;with;separators");

  // Saving every step means several appends to the same file.
  simulator.saveData(1, false, true, false, false);
  simulator.setTimeFrame(0, 6);
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0, 0});

  auto const baseName = std::to_string(static_cast<std::uint64_t>(simulator.id())) +
                        "_traffic_simulator_writer_test";
  auto const roadCsv = std::filesystem::current_path() / (baseName + "_road_data.csv");
  REQUIRE(std::filesystem::exists(roadCsv));

  std::string const expectedHeader{
      "datetime;time_step;street_id;coil;density_vpk;avg_speed_kph;std_speed_kph;"
      "n_observations;counts;queue_length"};

  SUBCASE("The header is written exactly once for the whole run") {
    std::ifstream file(roadCsv);
    REQUIRE(file.is_open());
    std::size_t nHeaders{0}, nLines{0};
    std::string line;
    while (std::getline(file, line)) {
      if (line == expectedHeader) {
        ++nHeaders;
      }
      ++nLines;
    }
    CHECK_EQ(nHeaders, 1);
    // One header plus at least one flush of the four streets.
    CHECK_GT(nLines, 4);
  }

  SUBCASE("A field containing the separator is quoted and round-trips") {
    csv::CSVFormat format;
    format.delimiter(';');
    csv::CSVReader reader(roadCsv.string(), format);
    std::size_t nCoilRows{0};
    for (auto const& row : reader) {
      // Escaping is what keeps the row parseable: without it the coil name would
      // spill into the neighbouring columns.
      REQUIRE_EQ(row.size(), 10);
      auto const coil = row["coil"].get<std::string>();
      if (!coil.empty()) {
        CHECK_EQ(coil, "coil;with;separators");
        CHECK_EQ(row["street_id"].get<Id>(), 0);
        ++nCoilRows;
      }
    }
    CHECK_GT(nCoilRows, 0);
  }

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(roadCsv);
}

TEST_CASE("TrafficSimulator SQL turn counts persistence") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  auto const dbPath = makeUniquePath("traffic_simulator_", ".db");
  writeTinyEdgesCsv(edgesPath);
  std::filesystem::remove(dbPath);

  TrafficSimulator simulator;
  spdlog::set_level(spdlog::level::trace);
  simulator.setName("traffic_simulator_turn_counts_sql_test");
  simulator.connectDataBase(dbPath.string());
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
  // Route agents from node 0 to node 2: they must cross both edges.
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 3, 1.0}});
  simulator.dynamics()->updatePaths();

  // 4th flag = save turn counts
  simulator.saveData(1, false, false, false, false, true);
  simulator.setTimeFrame(0, 10);
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  // Insert one agent at t=0, then idle.
  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  spdlog::set_level(spdlog::level::info);

  SQLite::Database db(dbPath.string(), SQLite::OPEN_READONLY);
  spdlog::info("Turn counts table row count: {}", rowCount(db, "turn_counts"));
  spdlog::info(dbPath.string());

  // Table must exist.
  CHECK(db.tableExists("turn_counts"));
  CHECK(rowCount(db, "turn_counts") > 0);

  // At least one turn event: edge 0 → edge 1.
  // Take all rows with source_edge_id = 0 and target_edge_id = 1, sum counts.
  {
    SQLite::Statement q(db,
                        "SELECT SUM(counts) FROM turn_counts WHERE source_edge_id = 0 "
                        "AND target_edge_id = 2");
    REQUIRE(q.executeStep());
    CHECK(q.getColumn(0).getInt64() == 1);
  }

  // Every row must reference valid, distinct edge IDs.
  {
    SQLite::Statement q(db, "SELECT source_edge_id, target_edge_id FROM turn_counts");
    while (q.executeStep()) {
      CHECK_NE(q.getColumn(0).getInt64(), q.getColumn(1).getInt64());
    }
  }

  // Counts must be positive.
  {
    SQLite::Statement q(db, "SELECT counts FROM turn_counts");
    while (q.executeStep()) {
      CHECK(q.getColumn(0).getInt64() > 0);
    }
  }

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(dbPath);
  std::filesystem::remove(dbPath.string() + "-wal");
  std::filesystem::remove(dbPath.string() + "-shm");
}

TEST_CASE("TrafficSimulator CSV turn counts persistence") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_turn_counts_csv_test");
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
  // Route agents from edge 0 to edge 3 so that they actually turn (0 -> 2). With
  // destination edge 1 the agent arrives at that edge's source node and is removed
  // without ever turning, so no turn event would be recorded at all.
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 3, 1.0}});
  simulator.dynamics()->updatePaths();

  // 4th flag = save turn counts
  simulator.saveData(1, false, false, false, false, true);
  simulator.setTimeFrame(0, 10);
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  simulator.run(std::vector<std::size_t>{1, 0, 0, 0, 0, 0, 0, 0, 0, 0});

  auto const baseName = std::to_string(static_cast<std::uint64_t>(simulator.id())) +
                        "_traffic_simulator_turn_counts_csv_test";
  auto const turnCountsCsv =
      std::filesystem::current_path() / (baseName + "_turn_counts.csv");

  // The linear network must have generated at least one turn event.
  REQUIRE(std::filesystem::exists(turnCountsCsv));

  {
    std::ifstream tcFile(turnCountsCsv);
    REQUIRE(tcFile.is_open());

    // Header row.
    std::string header;
    REQUIRE(std::getline(tcFile, header));
    CHECK_EQ(header, "datetime;time_step;source_edge_id;target_edge_id;counts");

    // At least one data row must follow.
    std::string dataRow;
    REQUIRE(std::getline(tcFile, dataRow));
    CHECK_FALSE(dataRow.empty());

    // The data row must contain 5 semicolon-separated fields.
    auto fieldCount = std::count(dataRow.begin(), dataRow.end(), ';');
    CHECK_EQ(fieldCount, 4);  // 4 delimiters → 5 fields
  }

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(turnCountsCsv);
}
TEST_CASE("TrafficSimulator run validation") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  SUBCASE("Running without a road network throws") {
    TrafficSimulator simulator;
    CHECK_THROWS_AS(simulator.run(std::vector<std::size_t>{1}), std::runtime_error);
    CHECK_THROWS_AS(simulator.run(1, 1, 1), std::runtime_error);
  }
  SUBCASE("Invalid schedules throw") {
    TrafficSimulator simulator;
    simulator.importRoadNetwork(edgesPath.string());
    simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
    CHECK_THROWS_AS(simulator.run(std::vector<std::size_t>{}), std::runtime_error);
    CHECK_THROWS_AS(simulator.run(std::vector<std::size_t>{1}, 0), std::invalid_argument);
    CHECK_THROWS_AS(simulator.run(1, 0, 1), std::invalid_argument);
    CHECK_THROWS_AS(simulator.run(1, 1, 0), std::invalid_argument);

    simulator.setTimeFrame(10, 16);
    // An end time not after the init time is ignored, leaving end (16) < init (20)
    simulator.setTimeFrame(20, 20);
    CHECK_EQ(simulator.endTime(), 16);
    CHECK_THROWS_AS(simulator.run(std::vector<std::size_t>{1}), std::runtime_error);
    CHECK_THROWS_AS(simulator.run(1, 1, 1), std::runtime_error);
  }

  std::filesystem::remove(edgesPath);
}

TEST_CASE("TrafficSimulator run schedule") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_schedule_test");
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 1, 1.0}});
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);

  SUBCASE("An explicit insertion delta time overrides the end time") {
    simulator.setTimeFrame(0, 100);
    simulator.run(std::vector<std::size_t>{1, 1}, 3);
    CHECK_EQ(simulator.endTime(), 6);
    CHECK_EQ(simulator.dynamics()->time_step(), 6);
  }
  SUBCASE("A saving interval of 0 saves a single snapshot") {
    // 7 steps for 2 insertions: the delta time is 3 and step 6 exceeds the schedule
    simulator.setTimeFrame(0, 7);
    simulator.saveData(0, true);
    simulator.run(std::vector<std::size_t>{1, 1});
    CHECK_EQ(simulator.dynamics()->time_step(), 7);

    auto const avgCsv = std::filesystem::current_path() /
                        (std::to_string(static_cast<std::uint64_t>(simulator.id())) +
                         "_traffic_simulator_schedule_test_avg_stats.csv");
    REQUIRE(std::filesystem::exists(avgCsv));
    std::ifstream avgFile(avgCsv);
    std::size_t nLines{0};
    for (std::string line; std::getline(avgFile, line);) {
      ++nLines;
    }
    CHECK_EQ(nLines, 2);  // header + one snapshot
    avgFile.close();
    std::filesystem::remove(avgCsv);
  }

  std::filesystem::remove(edgesPath);
}

TEST_CASE("TrafficSimulator slow charge with travel and agent data") {
  auto const edgesPath = makeUniquePath("traffic_simulator_edges_", ".csv");
  writeTinyEdgesCsv(edgesPath);

  TrafficSimulator simulator;
  simulator.setName("traffic_simulator_slow_charge_test");
  simulator.importRoadNetwork(edgesPath.string());
  REQUIRE(simulator.dynamics() != nullptr);
  simulator.dynamics()->setODs(std::vector<std::tuple<Id, Id, double>>{{0, 3, 1.0}});

  simulator.saveData(5, false, false, true, true);
  simulator.setTimeFrame(0, 20);
  simulator.setAgentInsertionMethod(AgentInsertionMethod::ODS);
  // One agent at t=0, then insertions every 2 steps, checked every 4 steps
  simulator.run(1, 2, 4);

  CHECK_EQ(simulator.dynamics()->time_step(), 20);
  auto const [nAdded, nInserted, nArrived, nKilled, nRemaining] =
      simulator.dynamics()->agentStats();
  CHECK_GE(nAdded, 10);
  CHECK_GT(nArrived, 0);

  auto const baseName = std::to_string(static_cast<std::uint64_t>(simulator.id())) +
                        "_traffic_simulator_slow_charge_test";
  auto const travelCsv =
      std::filesystem::current_path() / (baseName + "_travel_data.csv");
  auto const agentCsv = std::filesystem::current_path() / (baseName + "_agent_data.csv");
  REQUIRE(std::filesystem::exists(travelCsv));
  REQUIRE(std::filesystem::exists(agentCsv));
  CHECK_EQ(firstLine(travelCsv), "datetime;time_step;distance_m;travel_time_s");
  CHECK_EQ(firstLine(agentCsv), "agent_id;edge_id;time_step_in;time_step_out");

  std::filesystem::remove(edgesPath);
  std::filesystem::remove(travelCsv);
  std::filesystem::remove(agentCsv);
}
