#include "TrafficSimulator.hpp"

#include <simdjson.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>

static constexpr char CSV_SEPARATOR = ';';

namespace dsf::mobility {
  TrafficSimulator::TrafficSimulator() {
    // Take the current time and set id as YYYYMMDDHHMMSS
    auto const now = std::chrono::system_clock::now();
#ifdef __APPLE__
    std::time_t const t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y%m%d%H%M%S");
    m_id = std::stoull(oss.str());
#else
    m_id = std::stoull(std::format(
        "{:%Y%m%d%H%M%S}",
        std::chrono::floor<std::chrono::seconds>(
            std::chrono::current_zone()->to_local(std::chrono::system_clock::from_time_t(
                std::chrono::system_clock::to_time_t(now))))));
#endif
  }
  // TrafficSimulator::TrafficSimulator(std::string_view const jsonConfigPath) {
  //   simdjson::dom::parser parser;
  //   simdjson::dom::element root;
  //   auto error = parser.load(jsonConfigPath).get(root);
  //   if (error) {
  //     throw std::runtime_error(std::format(
  //         "Failed to load JSON configuration file '{}': {}", jsonConfigPath, simdjson::error_message(error)));
  //   }
  //   if (!root.is_object()) {
  //     throw std::runtime_error(std::format(
  //         "Invalid JSON configuration file '{}': root element is not an object",
  //         jsonConfigPath));
  //   }
  //   // Road network config
  //   auto roadNetworkConfig = root["road_network"];
  //   if (roadNetworkConfig.error()) {
  //     throw std::runtime_error(std::format(
  //         "Missing 'road_network' configuration in JSON file '{}'", jsonConfigPath));
  //   }
  //   importRoadNetwork(roadNetworkConfig["edges_file"].get_string().value(),
  //                     roadNetworkConfig["node_properties_file"].get_string().value());
  //   // Dynamics config
  //   auto dynamicsConfig = root["dynamics"];
  //   if (dynamicsConfig.error()) {
  //     throw std::runtime_error(std::format(
  //         "Missing 'dynamics' configuration in JSON file '{}'", jsonConfigPath));
  //   }
  //   if (dynamicsConfig["seed"].error()) {
  //     m_dynamics->setSeed(dynamicsConfig["seed"].get_uint64().value());
  //   }
  //   if (dynamicsConfig["agent_insertion_method"].get_string().value() == "random") {
  //     m_agentInsertionMethod = AgentInsertionMethod::RANDOM;
  //   } else if (dynamicsConfig["agent_insertion_method"].get_string().value() == "ods") {
  //     m_agentInsertionMethod = AgentInsertionMethod::ODS;
  //   } else if (dynamicsConfig["agent_insertion_method"].get_string().value() ==
  //              "random_ods") {
  //     m_agentInsertionMethod = AgentInsertionMethod::RANDOM_ODS;
  //   } else {
  //     throw std::runtime_error(
  //         std::format("Invalid 'agent_insertion_method' value in JSON file '{}': {}",
  //                     jsonConfigPath,
  //                     dynamicsConfig["agent_insertion_method"].get_string().value()));
  //   }
  //   // Database config
  //   auto dbConfig = root["database"];
  //   if (dbConfig.error()) {
  //     throw std::runtime_error(std::format(
  //         "Missing 'database' configuration in JSON file '{}'", jsonConfigPath));
  //   }
  //   connectDataBase(dbConfig["path"].get_string().value());
  // }

  void TrafficSimulator::connectDataBase(std::string_view const dbPath,
                                         std::string_view const queries) {
    m_database = std::make_unique<SQLite::Database>(
        dbPath, SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
    m_database->exec(std::string(queries));
    m_preparePersistence();
  }

  void TrafficSimulator::importRoadNetwork(std::string_view const edgesFile,
                                           std::string_view const nodePropertiesFile) {
    RoadNetwork network;
    network.importEdges(std::string(edgesFile));
    if (!nodePropertiesFile.empty()) {
      network.importNodeProperties(std::string(nodePropertiesFile));
    }
    m_dynamics = std::make_unique<FirstOrderDynamics>(std::move(network));
    spdlog::info("Road network imported successfully with {} nodes and {} edges.",
                 m_dynamics->graph().nNodes(),
                 m_dynamics->graph().nEdges());
    m_preparePersistence();
  }

  void TrafficSimulator::updatePaths(std::time_t const deltaT) {
    m_updatePathDeltaT = deltaT;
  }

  void TrafficSimulator::saveData(std::time_t const savingInterval,
                                  bool const saveAverageStats,
                                  bool const saveStreetData,
                                  bool const saveTravelData,
                                  bool const saveAgentData) {
    m_savingInterval = savingInterval;
    m_saveAverageStats = saveAverageStats;
    m_saveStreetData = saveStreetData;
    m_saveTravelData = saveTravelData;
    m_saveAgentData = saveAgentData;
    m_preparePersistence();
    spdlog::info(
        "Data saving configured: interval={}s, avg_stats={}, street_data={}, "
        "travel_data={}, agent_data={}",
        savingInterval,
        saveAverageStats,
        saveStreetData,
        saveTravelData,
        saveAgentData);
  }

  void TrafficSimulator::setName(const std::string& name) {
    m_name = name;
    m_safeName = name;
    std::ranges::replace(m_safeName, ' ', '_');
  }

  void TrafficSimulator::setTimeFrame(std::time_t const initTime,
                                      std::optional<std::time_t> const endTime) {
    m_initTime = initTime;
    if (endTime.has_value()) {
      if (*endTime <= m_initTime) {
        spdlog::warn(
            "End time ({}) is not greater than initial time ({}). End time will be "
            "ignored.",
            *endTime,
            m_initTime);
      } else {
        m_endTime = *endTime;
      }
    }
    if (m_endTime != 0 && !m_nAgentsPerTimeStep.empty()) {
      m_agentInsertionDeltaT = (m_endTime - m_initTime) / m_nAgentsPerTimeStep.size();
    }
  }

  void TrafficSimulator::setNAgentsPerTimeStep(
      std::vector<std::size_t> const& nAgentsPerTimeStep,
      std::optional<std::time_t> const deltaT) {
    m_nAgentsPerTimeStep = nAgentsPerTimeStep;
    if (deltaT.has_value()) {
      if (m_endTime == 0) {
        spdlog::warn(
            "Delta time for agent insertion is set to {} seconds, but no end time is "
            "currently set. The end time will be ignored for agent insertion timing.",
            deltaT.value());
      }
      m_agentInsertionDeltaT = deltaT.value();
      m_endTime = m_initTime + m_nAgentsPerTimeStep.size() * m_agentInsertionDeltaT;
    }
  }

  void TrafficSimulator::m_initStreetTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS road_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "street_id INTEGER NOT NULL, "
        "coil TEXT, "
        "density_vpk REAL, "
        "avg_speed_kph REAL, "
        "std_speed_kph REAL, "
        "n_observations INTEGER, "
        "counts INTEGER, "
        "queue_length INTEGER)");
    spdlog::info("Initialized road_data table in the database.");
  }

  void TrafficSimulator::m_saveStreetDataSQL(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const {
    if (streetDataRecords.empty()) {
      spdlog::debug("No street data records to save for time step {}.", time_step);
      return;
    }
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO road_data (datetime, time_step, simulation_id, street_id, "
        "coil, density_vpk, avg_speed_kph, std_speed_kph, n_observations, counts, "
        "queue_length) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

    for (auto const& [streetId, record] : streetDataRecords) {
      insertStmt.bind(1, datetime);
      insertStmt.bind(2, time_step);
      insertStmt.bind(3, simulation_id);
      insertStmt.bind(4, static_cast<std::int64_t>(streetId));
      if (record.coilName.has_value()) {
        insertStmt.bind(5, record.coilName.value());
      } else {
        insertStmt.bind(5);
      }
      insertStmt.bind(6, record.density);
      if (record.avgSpeed.has_value()) {
        insertStmt.bind(7, record.avgSpeed.value());
        insertStmt.bind(8, record.stdSpeed.value());
      } else {
        insertStmt.bind(7);
        insertStmt.bind(8);
      }
      insertStmt.bind(9, static_cast<std::int64_t>(record.nObservations.value_or(0)));
      if (record.counts.has_value()) {
        insertStmt.bind(10, static_cast<std::int64_t>(record.counts.value()));
      } else {
        insertStmt.bind(10);
      }
      insertStmt.bind(11, static_cast<std::int64_t>(record.queueLength));
      insertStmt.exec();
      insertStmt.reset();
    }
  }

  void TrafficSimulator::m_saveStreetDataCSV(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const {
    if (streetDataRecords.empty()) {
      spdlog::debug("No street data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{std::format("{}_{}_road_data.csv", simulation_id, m_safeName)};
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing street data.", filename);
      return;
    }
    if (!fileExists) {
      outFile << "datetime" << CSV_SEPARATOR << "time_step" << CSV_SEPARATOR
              << "street_id" << CSV_SEPARATOR << "coil" << CSV_SEPARATOR << "density_vpk"
              << CSV_SEPARATOR << "avg_speed_kph" << CSV_SEPARATOR << "std_speed_kph"
              << CSV_SEPARATOR << "n_observations" << CSV_SEPARATOR << "counts"
              << CSV_SEPARATOR << "queue_length" << "\n";
    }
    for (auto const& [streetId, record] : streetDataRecords) {
      outFile << datetime << CSV_SEPARATOR << time_step << CSV_SEPARATOR << streetId
              << CSV_SEPARATOR;
      if (record.coilName.has_value()) {
        outFile << record.coilName.value();
      }
      outFile << CSV_SEPARATOR << record.density << CSV_SEPARATOR;
      if (record.avgSpeed.has_value()) {
        outFile << record.avgSpeed.value() << CSV_SEPARATOR << record.stdSpeed.value()
                << CSV_SEPARATOR;
      } else {
        outFile << CSV_SEPARATOR << CSV_SEPARATOR;
      }
      outFile << record.nObservations.value_or(0) << CSV_SEPARATOR;
      if (record.counts.has_value()) {
        outFile << record.counts.value();
      }
      outFile << CSV_SEPARATOR << record.queueLength << "\n";
    }
    outFile.flush();
    outFile.close();
    spdlog::debug("Saved street data for time step {} to file {}.", time_step, filename);
  }

  void TrafficSimulator::m_initAvgStatsTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS avg_stats ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "n_ghost_agents INTEGER NOT NULL, "
        "n_agents INTEGER NOT NULL, "
        "mean_speed_kph REAL, "
        "std_speed_kph REAL, "
        "mean_density_vpk REAL NOT NULL, "
        "std_density_vpk REAL NOT NULL, "
        "mean_travel_time_s REAL, "
        "mean_queue_length REAL NOT NULL)");
    spdlog::info("Initialized avg_stats table in the database.");
  }

  void TrafficSimulator::m_saveAvgStatsSQL(const std::string& datetime,
                                           const std::int64_t time_step,
                                           const std::int64_t simulation_id,
                                           const AverageStatsRecord& averageStats) const {
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO avg_stats ("
        "simulation_id, datetime, time_step, n_ghost_agents, n_agents, "
        "mean_speed_kph, std_speed_kph, mean_density_vpk, std_density_vpk, "
        "mean_travel_time_s, mean_queue_length) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
    insertStmt.bind(1, simulation_id);
    insertStmt.bind(2, datetime);
    insertStmt.bind(3, time_step);
    insertStmt.bind(4, static_cast<std::int64_t>(m_dynamics->ghostAgents()));
    insertStmt.bind(5, static_cast<std::int64_t>(m_dynamics->nAgents()));

    if (averageStats.nValidEdges > 0) {
      insertStmt.bind(6, averageStats.meanSpeed);
      insertStmt.bind(7, averageStats.stdSpeed);
      insertStmt.bind(10, averageStats.meanTravelTime);
    } else {
      insertStmt.bind(6);
      insertStmt.bind(7);
      insertStmt.bind(10);
    }
    insertStmt.bind(8, averageStats.meanDensity);
    insertStmt.bind(9, averageStats.stdDensity);
    insertStmt.bind(11, averageStats.meanQueueLength);
    insertStmt.exec();
  }

  void TrafficSimulator::m_saveAvgStatsCSV(const std::string& datetime,
                                           const std::int64_t time_step,
                                           const std::int64_t simulation_id,
                                           const AverageStatsRecord& averageStats) const {
    auto const filename{std::format("{}_{}_avg_stats.csv", simulation_id, m_safeName)};
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing average stats.", filename);
      return;
    }
    if (!fileExists) {
      outFile << "datetime" << CSV_SEPARATOR << "time_step" << CSV_SEPARATOR
              << "n_ghost_agents" << CSV_SEPARATOR << "n_agents" << CSV_SEPARATOR
              << "mean_speed_kph" << CSV_SEPARATOR << "std_speed_kph" << CSV_SEPARATOR
              << "mean_density_vpk" << CSV_SEPARATOR << "std_density_vpk" << CSV_SEPARATOR
              << "mean_travel_time_s" << CSV_SEPARATOR << "mean_queue_length" << "\n";
    }
    outFile << datetime << CSV_SEPARATOR << time_step << CSV_SEPARATOR
            << m_dynamics->ghostAgents() << CSV_SEPARATOR << m_dynamics->nAgents()
            << CSV_SEPARATOR;
    if (averageStats.nValidEdges > 0) {
      outFile << averageStats.meanSpeed << CSV_SEPARATOR << averageStats.stdSpeed
              << CSV_SEPARATOR << averageStats.meanDensity << CSV_SEPARATOR
              << averageStats.stdDensity << CSV_SEPARATOR << averageStats.meanTravelTime
              << CSV_SEPARATOR;
    } else {
      outFile << CSV_SEPARATOR << CSV_SEPARATOR << averageStats.meanDensity
              << CSV_SEPARATOR << averageStats.stdDensity << CSV_SEPARATOR
              << CSV_SEPARATOR;
    }
    outFile << averageStats.meanQueueLength << "\n";
    outFile.flush();
    outFile.close();
    spdlog::debug(
        "Saved average stats for time step {} to file {}.", time_step, filename);
  }

  void TrafficSimulator::m_initTravelDataTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS travel_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "distance_m REAL NOT NULL, "
        "travel_time_s REAL NOT NULL)");
    spdlog::info("Initialized travel_data table in the database.");
  }

  void TrafficSimulator::m_saveTravelDataSQL(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_vector<std::pair<double, double>> travelDTs) const {
    if (travelDTs.empty()) {
      spdlog::debug("No travel data records to save for time step {}.", time_step);
      return;
    }
    SQLite::Statement insertStmt(*this->database(),
                                 "INSERT INTO travel_data (datetime, time_step, "
                                 "simulation_id, distance_m, travel_time_s) "
                                 "VALUES (?, ?, ?, ?, ?)");

    for (auto const& [distance, time] : travelDTs) {
      insertStmt.bind(1, datetime);
      insertStmt.bind(2, time_step);
      insertStmt.bind(3, simulation_id);
      insertStmt.bind(4, distance);
      insertStmt.bind(5, time);
      insertStmt.exec();
      insertStmt.reset();
    }
  }

  void TrafficSimulator::m_saveTravelDataCSV(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_vector<std::pair<double, double>> travelDTs) const {
    if (travelDTs.empty()) {
      spdlog::debug("No travel data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{std::format("{}_{}_travel_data.csv", simulation_id, m_safeName)};
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing travel data.", filename);
      return;
    }
    if (!fileExists) {
      outFile << "datetime" << CSV_SEPARATOR << "time_step" << CSV_SEPARATOR
              << "distance_m" << CSV_SEPARATOR << "travel_time_s" << "\n";
    }
    for (auto const& [distance, time] : travelDTs) {
      outFile << datetime << CSV_SEPARATOR << time_step << CSV_SEPARATOR << distance
              << CSV_SEPARATOR << time << "\n";
    }
    outFile.flush();
    outFile.close();
    spdlog::debug("Saved travel data for time step {} to file {}.", time_step, filename);
  }

  void TrafficSimulator::m_initAgentDataTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS agent_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "agent_id INTEGER NOT NULL, "
        "edge_id INTEGER NOT NULL, "
        "time_step_in INTEGER NOT NULL, "
        "time_step_out INTEGER NOT NULL)");
    spdlog::info("Initialized agent_data table in the database.");
  }

  void TrafficSimulator::m_saveAgentDataSQL(
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_unordered_map<Id,
                                    std::vector<std::tuple<Id, std::time_t, std::time_t>>>
          agentDataRecords) const {
    if (agentDataRecords.empty()) {
      spdlog::debug("No agent data records to save for time step {}.", time_step);
      return;
    }
    SQLite::Statement insertStmt(*this->database(),
                                 "INSERT INTO agent_data (simulation_id, "
                                 "agent_id, edge_id, time_step_in, time_step_out)"
                                 "VALUES (?, ?, ?, ?, ?)");
    for (auto const& [edge_id, data] : agentDataRecords) {
      for (auto const& [agent_id, ts_in, ts_out] : data) {
        insertStmt.bind(1, simulation_id);
        insertStmt.bind(2, static_cast<std::int64_t>(agent_id));
        insertStmt.bind(3, static_cast<std::int64_t>(edge_id));
        insertStmt.bind(4, static_cast<std::int64_t>(ts_in));
        insertStmt.bind(5, static_cast<std::int64_t>(ts_out));
        insertStmt.exec();
        insertStmt.reset();
      }
    }
  }

  void TrafficSimulator::m_saveAgentDataCSV(
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_unordered_map<Id,
                                    std::vector<std::tuple<Id, std::time_t, std::time_t>>>
          agentDataRecords) const {
    if (agentDataRecords.empty()) {
      spdlog::debug("No agent data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{std::format("{}_{}_agent_data.csv", simulation_id, m_safeName)};
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing agent data.", filename);
      return;
    }
    if (!fileExists) {
      outFile << "simulation_id" << CSV_SEPARATOR << "agent_id" << CSV_SEPARATOR
              << "edge_id" << CSV_SEPARATOR << "time_step_in" << CSV_SEPARATOR
              << "time_step_out" << "\n";
    }
    for (auto const& [edge_id, data] : agentDataRecords) {
      for (auto const& [agent_id, ts_in, ts_out] : data) {
        outFile << simulation_id << CSV_SEPARATOR << agent_id << CSV_SEPARATOR << edge_id
                << CSV_SEPARATOR << ts_in << CSV_SEPARATOR << ts_out << "\n";
      }
    }
    outFile.flush();
    outFile.close();
    spdlog::debug("Saved agent data for time step {} to file {}.", time_step, filename);
  }

  void TrafficSimulator::m_dumpNetwork() const {
    if (!this->database()) {
      return;
    }
    SQLite::Statement edgesQuery(
        *this->database(),
        "SELECT name FROM sqlite_master WHERE type='table' AND name='edges';");
    SQLite::Statement nodesQuery(
        *this->database(),
        "SELECT name FROM sqlite_master WHERE type='table' AND name='nodes';");
    bool edgesTableExists = edgesQuery.executeStep();
    bool nodesTableExists = nodesQuery.executeStep();
    if (edgesTableExists && nodesTableExists) {
      spdlog::info(
          "Edges and nodes tables already exist in the database. Skipping network dump.");
      return;
    }

    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS edges ("
        "id INTEGER PRIMARY KEY, "
        "source INTEGER NOT NULL, "
        "target INTEGER NOT NULL, "
        "length REAL NOT NULL, "
        "maxspeed REAL NOT NULL, "
        "name TEXT, "
        "nlanes INTEGER NOT NULL, "
        "coilcode TEXT, "
        "geometry TEXT NOT NULL)");
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS nodes ("
        "id INTEGER PRIMARY KEY, "
        "type TEXT, "
        "geometry TEXT)");

    SQLite::Statement insertEdgeStmt(*this->database(),
                                     "INSERT INTO edges (id, source, target, length, "
                                     "maxspeed, name, nlanes, coilcode, geometry) "
                                     "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);");
    for (const auto& [edgeId, pEdge] : m_dynamics->graph().edges()) {
      insertEdgeStmt.bind(1, static_cast<std::int64_t>(edgeId));
      insertEdgeStmt.bind(2, static_cast<std::int64_t>(pEdge->source()));
      insertEdgeStmt.bind(3, static_cast<std::int64_t>(pEdge->target()));
      insertEdgeStmt.bind(4, pEdge->length());
      insertEdgeStmt.bind(5, pEdge->maxSpeed());
      insertEdgeStmt.bind(6, pEdge->name());
      insertEdgeStmt.bind(7, pEdge->nLanes());
      auto const& counterName{pEdge->counterName()};
      if (counterName != "N/A") {
        insertEdgeStmt.bind(8, counterName);
      } else {
        insertEdgeStmt.bind(8);
      }
      insertEdgeStmt.bind(9, std::format("{}", pEdge->geometry()));
      insertEdgeStmt.exec();
      insertEdgeStmt.reset();
    }

    SQLite::Statement insertNodeStmt(
        *this->database(), "INSERT INTO nodes (id, type, geometry) VALUES (?, ?, ?);");
    for (const auto& [nodeId, pNode] : m_dynamics->graph().nodes()) {
      insertNodeStmt.bind(1, static_cast<std::int64_t>(nodeId));
      if (pNode->isTrafficLight()) {
        insertNodeStmt.bind(2, "traffic_light");
      } else if (pNode->isRoundabout()) {
        insertNodeStmt.bind(2, "roundabout");
      } else {
        insertNodeStmt.bind(2);
      }
      if (pNode->geometry().has_value()) {
        insertNodeStmt.bind(3, std::format("{}", *pNode->geometry()));
      } else {
        insertNodeStmt.bind(3);
      }
      insertNodeStmt.exec();
      insertNodeStmt.reset();
    }
  }

  void TrafficSimulator::m_preparePersistence() {
    if (m_dynamics == nullptr) {
      return;
    }
    if (m_saveAgentData) {
      Street::acquireAgentData();
    }
    if (m_database == nullptr) {
      return;
    }
    if (m_saveStreetData) {
      m_initStreetTable();
    }
    if (m_saveAverageStats) {
      m_initAvgStatsTable();
    }
    if (m_saveTravelData) {
      m_initTravelDataTable();
    }
    if (m_saveAgentData) {
      m_initAgentDataTable();
    }
    m_dumpNetwork();
  }

  void TrafficSimulator::m_flushStepData(StepDataResult&& stepData) {
    if (!stepData.streetData.has_value() && !stepData.travelData.has_value() &&
        !stepData.averageStats.has_value() && !stepData.agentData.has_value()) {
      return;
    }

    auto const datetime = m_timeToStr(m_initTime + stepData.timeStep);
    auto const timeStep = static_cast<std::int64_t>(stepData.timeStep);
    auto const simulationId = static_cast<std::int64_t>(m_id);

    if (m_database != nullptr) {
      std::optional<SQLite::Transaction> transaction;
      transaction.emplace(*m_database);
      if (stepData.streetData.has_value()) {
        m_saveStreetDataSQL(
            datetime, timeStep, simulationId, std::move(*stepData.streetData));
      }
      if (stepData.travelData.has_value()) {
        m_saveTravelDataSQL(
            datetime, timeStep, simulationId, std::move(*stepData.travelData));
      }
      if (stepData.agentData.has_value()) {
        m_saveAgentDataSQL(timeStep, simulationId, std::move(*stepData.agentData));
      }
      if (stepData.averageStats.has_value()) {
        m_saveAvgStatsSQL(datetime, timeStep, simulationId, *stepData.averageStats);
      }
      transaction->commit();
      return;
    }

    if (stepData.streetData.has_value()) {
      m_saveStreetDataCSV(
          datetime, timeStep, simulationId, std::move(*stepData.streetData));
    }
    if (stepData.travelData.has_value()) {
      m_saveTravelDataCSV(
          datetime, timeStep, simulationId, std::move(*stepData.travelData));
    }
    if (stepData.agentData.has_value()) {
      m_saveAgentDataCSV(timeStep, simulationId, std::move(*stepData.agentData));
    }
    if (stepData.averageStats.has_value()) {
      m_saveAvgStatsCSV(datetime, timeStep, simulationId, *stepData.averageStats);
    }
  }

  void TrafficSimulator::run(bool const reinsertAgents) {
    if (m_dynamics == nullptr) {
      throw std::runtime_error(
          "Cannot run the simulation without imported road network dynamics.");
    }
    if (m_nAgentsPerTimeStep.empty()) {
      throw std::runtime_error(
          "Cannot run the simulation without an agent insertion schedule.");
    }

    auto const totalTimeSteps = m_endTime - m_initTime;

    if (m_agentInsertionDeltaT == 0) {
      if (m_endTime > m_initTime) {
        m_agentInsertionDeltaT =
            totalTimeSteps / static_cast<std::time_t>(m_nAgentsPerTimeStep.size());
        if (totalTimeSteps % m_nAgentsPerTimeStep.size() != 0) {
          spdlog::warn(
              "Total simulation time ({} seconds) is not perfectly divisible by the "
              "number of agent insertion steps ({}). The last agent insertion step "
              "will occur at time {}.",
              totalTimeSteps,
              m_nAgentsPerTimeStep.size(),
              m_timeToStr(m_initTime +
                          m_agentInsertionDeltaT * m_nAgentsPerTimeStep.size()));
        }
      }
      if (m_agentInsertionDeltaT == 0) {
        m_agentInsertionDeltaT = 1;
      }
    }
    if (m_endTime == 0) {
      m_endTime = m_initTime + static_cast<std::time_t>(m_agentInsertionDeltaT *
                                                        m_nAgentsPerTimeStep.size());
    }

    m_preparePersistence();

    spdlog::info(
        "Starting simulation run from {} to {} ({} time steps) with agent insertion "
        "every {} seconds.",
        m_timeToStr(m_initTime),
        m_timeToStr(m_endTime),
        totalTimeSteps,
        m_agentInsertionDeltaT);
    for (auto currentTime = m_initTime; currentTime < m_endTime; ++currentTime) {
      auto currentStep = m_dynamics->time_step();
      if (m_updatePathDeltaT > 0 && currentStep % m_updatePathDeltaT == 0) {
        m_dynamics->updatePaths();
      }
      if (currentStep % m_agentInsertionDeltaT == 0) {
        auto const insertionIndex =
            static_cast<std::size_t>(currentStep / m_agentInsertionDeltaT);
        if (insertionIndex < m_nAgentsPerTimeStep.size()) {
          auto const nAgents = m_nAgentsPerTimeStep.at(insertionIndex);
          if (nAgents > 0) {
            m_dynamics->addAgents(nAgents, m_agentInsertionMethod);
          }
        } else {
          spdlog::warn(
              "Current time step {} exceeds the agent insertion schedule. No more "
              "agents will be inserted.",
              currentStep);
        }
      }

      bool const shouldSave =
          m_savingInterval.has_value() &&
          (m_savingInterval.value() == 0 || currentStep % m_savingInterval.value() == 0);
      auto stepData = m_dynamics->evolve(reinsertAgents,
                                         shouldSave ? StepDataRequest{m_saveAverageStats,
                                                                      m_saveStreetData,
                                                                      m_saveTravelData,
                                                                      m_saveAgentData}
                                                    : StepDataRequest{});

      if (shouldSave) {
        m_pendingStepData.push_back(std::move(stepData));
        m_flushStepData(std::move(m_pendingStepData.back()));
        m_pendingStepData.clear();
        if (m_savingInterval.value() == 0) {
          m_savingInterval.reset();
          m_saveAverageStats = false;
          m_saveStreetData = false;
          m_saveTravelData = false;
          m_saveAgentData = false;
        }
      }
    }
    spdlog::info("Simulation run completed.");
  }
}  // namespace dsf::mobility
