#include "TrafficSimulator.hpp"
#include "../utility/progress_bar.hpp"

#include <csv.hpp>
#include <simdjson.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>

namespace dsf::mobility {
  /// @brief Simple CSV writer for batch data persistence
  class CSVWriter {
  private:
    std::ofstream m_file;
    char m_separator;
    bool m_headerWritten = false;

  public:
    /// @brief Construct a CSV writer and open the file
    /// @param filename The output file path
    /// @param separator The field separator character
    CSVWriter(std::string_view const filename, char separator = ';')
        : m_separator(separator) {
      m_file.open(std::string(filename), std::ios::app);
      if (!m_file.is_open()) {
        throw std::runtime_error(
            std::format("Failed to open CSV file for writing: {}", filename));
      }
      m_headerWritten = std::filesystem::file_size(filename) > 0;
    }

    /// @brief Write header row
    template <typename... Args>
    void writeHeader(Args&&... headers) {
      if (m_headerWritten)
        return;
      writeRow(std::forward<Args>(headers)...);
      m_headerWritten = true;
    }

    /// @brief Write a data row
    template <typename... Args>
    void writeRow(Args&&... fields) {
      bool first = true;
      (..., ([this, &first](auto const& field) {
         if (!first)
           m_file << m_separator;
         if constexpr (std::is_same_v<std::decay_t<decltype(field)>, std::nullopt_t>) {
           // Write nothing for nullopt
         } else {
           m_file << field;
         }
         first = false;
       }(fields)));
      m_file << "\n";
    }

    /// @brief Flush and close the file
    void close() {
      if (m_file.is_open()) {
        m_file.flush();
        m_file.close();
      }
    }

    /// @brief Destructor automatically closes the file
    ~CSVWriter() { close(); }
  };

  std::string TrafficSimulator::m_generateCSVfilename(
      std::string_view const tableName) const {
    if (m_outputPrefix.empty()) {
      return std::format("{}_{}_{}.csv", m_id, m_safeName, tableName);
    }
    return std::format("{}{}.csv", m_outputPrefix, tableName);
  }

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
  TrafficSimulator::TrafficSimulator(std::string_view const jsonConfigPath) {
    importConfig(jsonConfigPath);
  }

  void TrafficSimulator::importConfig(std::string_view const jsonConfigFile) {
    simdjson::dom::parser parser;
    simdjson::dom::element root;
    auto error = parser.load(jsonConfigFile).get(root);
    if (error) {
      throw std::runtime_error(
          std::format("Failed to load JSON configuration file '{}': {}",
                      jsonConfigFile,
                      simdjson::error_message(error)));
    }
    if (!root.is_object()) {
      throw std::runtime_error(std::format(
          "Invalid JSON configuration file '{}': root element is not an object",
          jsonConfigFile));
    }
    auto require_field = [&](auto const& object,
                             std::string_view const section,
                             std::string_view const field) {
      auto fieldValue = object[field];
      if (fieldValue.error()) {
        throw std::runtime_error(
            std::format("Missing JSON field '{}.{}' in configuration file '{}'",
                        section,
                        field,
                        jsonConfigFile));
      }
      return fieldValue;
    };
    // General settings
    auto generalConfig = root["general"];
    if (generalConfig.error()) {
      throw std::runtime_error(std::format(
          "Missing 'general' configuration in JSON file '{}'", jsonConfigFile));
    }
    auto const input_folder = std::filesystem::path(
        require_field(generalConfig, "general", "input_folder").get_string().value());
    auto const output_folder = std::filesystem::path(
        require_field(generalConfig, "general", "output_folder").get_string().value());
    std::filesystem::create_directories(output_folder);
    setOutputPrefix(output_folder.string());
    {
      auto const name =
          require_field(generalConfig, "general", "name").get_string().value();
      setName(name);
    }
    // Road network config
    {
      auto roadNetworkConfig = root["road_network"];
      if (roadNetworkConfig.error()) {
        throw std::runtime_error(std::format(
            "Missing 'road_network' configuration in JSON file '{}'", jsonConfigFile));
      }
      auto const edgesFile =
          input_folder /
          std::filesystem::path(
              require_field(roadNetworkConfig, "road_network", "edges_file")
                  .get_string()
                  .value());
      auto const nodePropertiesFile =
          input_folder /
          std::filesystem::path(
              require_field(roadNetworkConfig, "road_network", "node_properties_file")
                  .get_string()
                  .value());
      importRoadNetwork(edgesFile.string(), nodePropertiesFile.string());
      auto const setEdgeWeightParams =
          require_field(roadNetworkConfig, "road_network", "set_edge_weight");
      m_dynamics->graph().setEdgeWeight(
          require_field(setEdgeWeightParams, "road_network.set_edge_weight", "weight")
              .get_string()
              .value(),
          require_field(setEdgeWeightParams, "road_network.set_edge_weight", "threshold")
              .get_double()
              .value());
    }
    // Dynamics config
    {
      auto dynamicsConfig = root["dynamics"];
      if (dynamicsConfig.error()) {
        throw std::runtime_error(std::format(
            "Missing 'dynamics' configuration in JSON file '{}'", jsonConfigFile));
      }
      if (!dynamicsConfig["seed"].error()) {
        m_dynamics->setSeed(dynamicsConfig["seed"].get_uint64().value());
      }
      auto const agentInsertionMethod =
          require_field(dynamicsConfig, "dynamics", "agent_insertion_method")
              .get_string()
              .value();
      if (agentInsertionMethod == "RANDOM") {
        m_agentInsertionMethod = AgentInsertionMethod::RANDOM;
      } else if (agentInsertionMethod == "ODS") {
        m_agentInsertionMethod = AgentInsertionMethod::ODS;
      } else if (agentInsertionMethod == "RANDOM_ODS") {
        m_agentInsertionMethod = AgentInsertionMethod::RANDOM_ODS;
      } else if (agentInsertionMethod == "UNIFORM") {
        m_agentInsertionMethod = AgentInsertionMethod::UNIFORM;
      } else {
        throw std::runtime_error(
            std::format("Invalid 'agent_insertion_method' value in JSON file '{}': {}",
                        jsonConfigFile,
                        agentInsertionMethod));
      }
      if (!dynamicsConfig["error_probability"].error()) {
        m_dynamics->setErrorProbability(
            dynamicsConfig["error_probability"].get_double().value());
      }
      if (!dynamicsConfig["kill_stagnant_agents"].error()) {
        m_dynamics->killStagnantAgents(
            dynamicsConfig["kill_stagnant_agents"].get_double().value());
      }
      if (!dynamicsConfig["importODsFromCSV"].error()) {
        auto const importODsFromCSVConfig = dynamicsConfig["importODsFromCSV"];
        auto const odsFile =
            input_folder /
            std::filesystem::path(
                require_field(importODsFromCSVConfig, "importODsFromCSV", "file")
                    .get_string()
                    .value());
        if (!importODsFromCSVConfig["separator"].error()) {
          auto const sep =
              require_field(importODsFromCSVConfig, "importODsFromCSV", "separator")
                  .get_string()
                  .value()[0];
          m_dynamics->importODsFromCSV(odsFile.string(), sep);
        } else {
          m_dynamics->importODsFromCSV(odsFile.string());
        }
      }
    }
    // Connect DB
    {
      if (!generalConfig["database"].error()) {
        spdlog::info("Database configuration found. Connecting to database...");
        auto const databasePath =
            output_folder /
            std::filesystem::path(
                require_field(generalConfig, "general", "database").get_string().value());
        connectDataBase(databasePath.string());
      }
    }
    // Update paths
    {
      auto const updatePathsConfig = generalConfig["update_paths"];
      if (!updatePathsConfig.error()) {
        updatePaths(require_field(updatePathsConfig, "general.update_paths", "interval")
                        .get_uint64()
                        .value(),
                    updatePathsConfig["throw_on_empty"].get_bool().has_value()
                        ? updatePathsConfig["throw_on_empty"].get_bool().value()
                        : true);
      }
    }
    // Save Data
    {
      auto const save_data = generalConfig["save_data"];
      if (!save_data.error()) {
        saveData(
            require_field(save_data, "general.save_data", "interval").get_uint64().value(),
            save_data["avg"].get_bool().has_value() ? save_data["avg"].get_bool().value()
                                                    : false,
            save_data["road"].get_bool().has_value()
                ? save_data["road"].get_bool().value()
                : false,
            save_data["travel"].get_bool().has_value()
                ? save_data["travel"].get_bool().value()
                : false,
            save_data["agent"].get_bool().has_value()
                ? save_data["agent"].get_bool().value()
                : false);
      }
    }
  }

  void TrafficSimulator::m_runDefault(std::vector<std::size_t> const& nAgentsPerTimeStep,
                                      std::optional<std::time_t> const deltaT) {
    if (deltaT.has_value()) {
      if (m_endTime == 0) {
        spdlog::warn(
            "Delta time for agent insertion is set to {} seconds, but no end time is "
            "currently set. The end time will be ignored for agent insertion timing.",
            deltaT.value());
      }
      m_endTime = m_initTime + nAgentsPerTimeStep.size() * deltaT.value();
    }
    std::time_t agentInsertionDeltaT = deltaT.value_or(0);

    if (nAgentsPerTimeStep.empty()) {
      throw std::runtime_error(
          "Cannot run the simulation without an agent insertion schedule.");
    }

    auto const totalTimeSteps = static_cast<std::size_t>(m_endTime - m_initTime);

    if (agentInsertionDeltaT == 0) {
      if (m_endTime > m_initTime) {
        agentInsertionDeltaT =
            totalTimeSteps / static_cast<std::time_t>(nAgentsPerTimeStep.size());
        if (totalTimeSteps % nAgentsPerTimeStep.size() != 0) {
          spdlog::warn(
              "Total simulation time ({} seconds) is not perfectly divisible by the "
              "number of agent insertion steps ({}). The last agent insertion step "
              "will occur at time {} instead of the end time {}.",
              totalTimeSteps,
              nAgentsPerTimeStep.size(),
              m_timeToStr(m_initTime + agentInsertionDeltaT * nAgentsPerTimeStep.size()),
              m_timeToStr(m_endTime));
        }
      }
      if (agentInsertionDeltaT == 0) {
        agentInsertionDeltaT = 1;
      }
    }
    if (m_endTime == 0) {
      m_endTime = m_initTime + static_cast<std::time_t>(agentInsertionDeltaT *
                                                        nAgentsPerTimeStep.size());
    }

    m_preparePersistence();

    spdlog::info(
        "Starting simulation run from {} to {} ({} time steps) with agent insertion "
        "every {} seconds.",
        m_timeToStr(m_initTime),
        m_timeToStr(m_endTime),
        totalTimeSteps,
        agentInsertionDeltaT);
    auto pbar = dsf::utility::default_progress_bar("Running simulation", totalTimeSteps);
    for (std::size_t currentStep{0}; currentStep < totalTimeSteps; ++currentStep) {
      if ((m_updatePathDeltaT > 0 && currentStep % m_updatePathDeltaT == 0) ||
          (currentStep == 0)) {
        m_dynamics->updatePaths();
      }
      if (currentStep % agentInsertionDeltaT == 0) {
        auto const insertionIndex =
            static_cast<std::size_t>(currentStep / agentInsertionDeltaT);
        if (insertionIndex < nAgentsPerTimeStep.size()) {
          auto const nAgents = nAgentsPerTimeStep.at(insertionIndex);
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
      auto stepData = m_dynamics->evolve(shouldSave ? StepDataRequest{m_saveAverageStats,
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
      pbar->update();
    }
  }
  void TrafficSimulator::m_runSlowCharge(std::size_t const nInitialAgents,
                                         std::time_t const agentInsertionDeltaT,
                                         std::time_t const checkDeltaT,
                                         std::size_t const agentIncrement) {
    if (m_endTime < m_initTime) {
      throw std::runtime_error(
          "End time must be greater than or equal to initial time for the simulation.");
    }
    auto const totalTimeSteps = static_cast<std::size_t>(m_endTime - m_initTime);

    m_preparePersistence();

    spdlog::info(
        "Starting slow charge run from {} to {} ({} time steps) with agent insertion "
        " check every {} seconds.",
        m_timeToStr(m_initTime),
        m_timeToStr(m_endTime),
        totalTimeSteps,
        checkDeltaT);
    auto pbar = dsf::utility::default_progress_bar("Running simulation", totalTimeSteps);
    std::size_t currentAgents{nInitialAgents};
    std::size_t previousAgents{0};
    for (std::size_t currentStep{0}; currentStep < totalTimeSteps; ++currentStep) {
      if ((m_updatePathDeltaT > 0 && currentStep % m_updatePathDeltaT == 0) ||
          (currentStep == 0)) {
        m_dynamics->updatePaths();
      }
      if (currentStep > 0 && currentStep % checkDeltaT == 0) {
        if (m_dynamics->nAgents() < previousAgents) {
          currentAgents += agentIncrement;
        }
        previousAgents = m_dynamics->nAgents();
      }
      if (currentStep % agentInsertionDeltaT == 0) {
        m_dynamics->addAgents(currentAgents, m_agentInsertionMethod);
      }

      bool const shouldSave =
          m_savingInterval.has_value() &&
          (m_savingInterval.value() == 0 || currentStep % m_savingInterval.value() == 0);
      auto stepData = m_dynamics->evolve(shouldSave ? StepDataRequest{m_saveAverageStats,
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
      pbar->update();
    }
  }

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

  void TrafficSimulator::updatePaths(std::time_t const deltaT,
                                     bool const throw_on_empty) {
    m_updatePathDeltaT = deltaT;
    m_dynamics->setUpdatePathsThrowOnEmpty(throw_on_empty);
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

  void TrafficSimulator::setName(std::string_view const name) {
    m_name = std::string(name);
    m_safeName = std::string(name);
    std::ranges::replace(m_safeName, ' ', '_');
  }
  void TrafficSimulator::setOutputPrefix(std::string_view const prefix) {
    // Check if prefix is a path to an existing directory. If so, be sure it has a trailing separator
    std::filesystem::path prefixPath(prefix);
    if (std::filesystem::exists(prefixPath) &&
        std::filesystem::is_directory(prefixPath)) {
      std::string prefixStr = prefixPath.string();
      if (prefixStr.empty() ||
          prefixStr.back() !=
              static_cast<char>(std::filesystem::path::preferred_separator)) {
        m_outputPrefix =
            prefixStr + static_cast<char>(std::filesystem::path::preferred_separator);
      } else {
        m_outputPrefix = prefixStr;
      }
    } else {
      m_outputPrefix = std::string(prefix);
    }
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
  }

  void TrafficSimulator::m_dumpSimInfoSQL() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS simulation_info ("
        "id INTEGER PRIMARY KEY, "
        "name TEXT, "
        "init_time TEXT, "
        "end_time TEXT, "
        "agent_insertion_method TEXT)");
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT OR REPLACE INTO simulation_info (id, name, init_time, end_time, "
        "agent_insertion_method) VALUES (?, ?, ?, ?, ?)");
    insertStmt.bind(1, static_cast<std::int64_t>(m_id));
    insertStmt.bind(2, m_name);
    insertStmt.bind(3, strInitTime());
    insertStmt.bind(4, strEndTime());
    std::string insertionMethodStr;
    switch (m_agentInsertionMethod) {
      case AgentInsertionMethod::RANDOM:
        insertionMethodStr = "random";
        break;
      case AgentInsertionMethod::ODS:
        insertionMethodStr = "ods";
        break;
      case AgentInsertionMethod::RANDOM_ODS:
        insertionMethodStr = "random_ods";
        break;
      default:
        insertionMethodStr = "unknown";
    }
    insertStmt.bind(5, insertionMethodStr);
    insertStmt.exec();
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
      tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const {
    if (streetDataRecords.empty()) {
      spdlog::debug("No street data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{m_generateCSVfilename("road_data")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      CSVWriter writer(filename);
      if (!fileExists) {
        writer.writeHeader("datetime",
                           "time_step",
                           "street_id",
                           "coil",
                           "density_vpk",
                           "avg_speed_kph",
                           "std_speed_kph",
                           "n_observations",
                           "counts",
                           "queue_length");
      }
      for (auto const& [streetId, record] : streetDataRecords) {
        auto avgSpeed =
            record.avgSpeed.has_value() ? std::format("{}", record.avgSpeed.value()) : "";
        auto stdSpeed =
            record.avgSpeed.has_value() ? std::format("{}", record.stdSpeed.value()) : "";
        writer.writeRow(
            datetime,
            time_step,
            streetId,
            record.coilName.has_value() ? record.coilName.value() : "",
            record.density,
            avgSpeed,
            stdSpeed,
            record.nObservations.value_or(0),
            record.counts.has_value() ? std::to_string(record.counts.value()) : "",
            record.queueLength);
      }
      spdlog::debug(
          "Saved street data for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write street data CSV: {}", e.what());
    }
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
                                           const AverageStatsRecord& averageStats) const {
    auto const filename{m_generateCSVfilename("avg_stats")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      CSVWriter writer(filename);
      if (!fileExists) {
        writer.writeHeader("datetime",
                           "time_step",
                           "n_ghost_agents",
                           "n_agents",
                           "mean_speed_kph",
                           "std_speed_kph",
                           "mean_density_vpk",
                           "std_density_vpk",
                           "mean_travel_time_s",
                           "mean_queue_length");
      }
      auto meanSpeed =
          averageStats.nValidEdges > 0 ? std::format("{}", averageStats.meanSpeed) : "";
      auto stdSpeed =
          averageStats.nValidEdges > 0 ? std::format("{}", averageStats.stdSpeed) : "";
      auto meanTravelTime = averageStats.nValidEdges > 0
                                ? std::format("{}", averageStats.meanTravelTime)
                                : "";
      writer.writeRow(datetime,
                      time_step,
                      m_dynamics->ghostAgents(),
                      m_dynamics->nAgents(),
                      meanSpeed,
                      stdSpeed,
                      averageStats.meanDensity,
                      averageStats.stdDensity,
                      meanTravelTime,
                      averageStats.meanQueueLength);
      spdlog::debug(
          "Saved average stats for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write average stats CSV: {}", e.what());
    }
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
      tbb::concurrent_vector<std::pair<double, double>> travelDTs) const {
    if (travelDTs.empty()) {
      spdlog::debug("No travel data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{m_generateCSVfilename("travel_data")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      CSVWriter writer(filename);
      if (!fileExists) {
        writer.writeHeader("datetime", "time_step", "distance_m", "travel_time_s");
      }
      for (auto const& [distance, time] : travelDTs) {
        writer.writeRow(datetime, time_step, distance, time);
      }
      spdlog::debug(
          "Saved travel data for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write travel data CSV: {}", e.what());
    }
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
      tbb::concurrent_unordered_map<Id,
                                    std::vector<std::tuple<Id, std::time_t, std::time_t>>>
          agentDataRecords) const {
    if (agentDataRecords.empty()) {
      spdlog::debug("No agent data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{m_generateCSVfilename("agent_data")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      CSVWriter writer(filename);
      if (!fileExists) {
        writer.writeHeader("agent_id", "edge_id", "time_step_in", "time_step_out");
      }
      for (auto const& [edge_id, data] : agentDataRecords) {
        for (auto const& [agent_id, ts_in, ts_out] : data) {
          writer.writeRow(agent_id, edge_id, ts_in, ts_out);
        }
      }
      spdlog::debug("Saved agent data for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write agent data CSV: {}", e.what());
    }
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
          "Edges and nodes tables already exist in the database. Skipping network "
          "dump.");
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
    m_dumpSimInfoSQL();
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
      m_saveStreetDataCSV(datetime, timeStep, std::move(*stepData.streetData));
    }
    if (stepData.travelData.has_value()) {
      m_saveTravelDataCSV(datetime, timeStep, std::move(*stepData.travelData));
    }
    if (stepData.agentData.has_value()) {
      m_saveAgentDataCSV(timeStep, std::move(*stepData.agentData));
    }
    if (stepData.averageStats.has_value()) {
      m_saveAvgStatsCSV(datetime, timeStep, *stepData.averageStats);
    }
  }
}  // namespace dsf::mobility
