#pragma once

#include <SQLiteCpp/SQLiteCpp.h>

#include "FirstOrderDynamics.hpp"
#include "RoadNetwork.hpp"

#include <chrono>
#include <deque>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef __APPLE__
#include <algorithm>
#include <iomanip>
#include <sstream>
#endif

namespace dsf::mobility {
  class TrafficSimulator {
    std::unique_ptr<FirstOrderDynamics> m_dynamics;
    std::unique_ptr<SQLite::Database> m_database;
    Id m_id;
    std::string m_name = "simulation";
    std::string m_safeName = "simulation";
    std::string m_outputPrefix = std::string();
    AgentInsertionMethod m_agentInsertionMethod{AgentInsertionMethod::RANDOM};
    std::time_t m_initTime = 0;
    std::time_t m_endTime = 0;
    std::time_t m_updatePathDeltaT = 0;
    std::optional<std::time_t> m_savingInterval{std::nullopt};
    bool m_saveAverageStats{false};
    bool m_saveStreetData{false};
    bool m_saveTravelData{false};
    bool m_saveAgentData{false};
    std::deque<StepDataResult> m_pendingStepData;

    /// @brief Convert a time_t to a string in the datetime format "YYYY-MM-DD HH:MM:SS"
    /// @param time The time_t to convert
    /// @return std::string The converted string
    inline std::string m_timeToStr(std::time_t const time) const {
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

    std::string m_generateCSVfilename(std::string_view const tableName) const;

    void m_dumpSimInfoSQL() const;
    /// @brief Initialize the street data table.
    /// This table contains the data of each street. Columns are:
    /// - id: The entry id (auto-incremented)
    /// - simulation_id: The simulation id
    /// - datetime: The datetime of the data entry
    /// - time_step: The time step of the data entry
    /// - street_id: The id of the street
    /// - coil: The name of the coil on the street (can be null)
    /// - density_vpk: The density in vehicles per kilometer
    /// - avg_speed_kph: The average speed in kilometers per hour
    /// - std_speed_kph: The standard deviation of the speed in kilometers per hour
    /// - n_observations: The number of speed observations used to compute avg/std (0 if none)
    /// - counts: The counts of the coil sensor (can be null)
    /// - queue_length: The length of the queue on the street
    void m_initStreetTable() const;
    /// @brief Save street data to the database using a batch insert.
    /// @param datetime The datetime of the data entry
    /// @param time_step The time step of the data entry
    /// @param simulation_id The id of the simulation
    /// @param streetDataRecords A map of StreetDataRecord containing the data to be saved
    void m_saveStreetDataSQL(
        const std::string& datetime,
        const std::int64_t time_step,
        const std::int64_t simulation_id,
        tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const;
    void m_saveStreetDataCSV(
        const std::string& datetime,
        const std::int64_t time_step,
        tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const;
    /// @brief Initialize the average stats table.
    /// This table contains the average stats of the simulation at each time step. Columns are:
    /// - id: The entry id (auto-incremented)
    /// - simulation_id: The simulation id
    /// - datetime: The datetime of the data entry
    /// - time_step: The time step of the data entry
    /// - n_ghost_agents: The number of ghost agents
    /// - n_agents: The number of agents
    /// - mean_speed_kph: The mean speed of the agents in kilometers per hour
    /// - std_speed_kph: The standard deviation of the speed of the agents in kilometers per hour
    /// - mean_density_vpk: The mean density of the streets in vehicles per kilometer
    /// - std_density_vpk: The standard deviation of the density of the streets in vehicles per kilometer
    void m_initAvgStatsTable() const;
    /// @brief Save average stats to the database.
    /// @param datetime The datetime of the data entry
    /// @param time_step The time step of the data entry
    /// @param simulation_id The id of the simulation
    /// @param n_valid_edges The number of valid edges (i.e. edges with speed observations)
    /// @param mean_speed The mean speed of the agents in kilometers per hour
    /// @param std_speed The standard deviation of the speed of the agents in kilometers per hour
    /// @param mean_density The mean density of the streets in vehicles per kilometer
    /// @param std_density The standard deviation of the density of the streets in vehicles per kilometer
    /// @param mean_traveltime The mean travel time of the agents in seconds
    /// @param meanQueueLength The mean queue length of the streets
    void m_saveAvgStatsSQL(const std::string& datetime,
                           const std::int64_t time_step,
                           const std::int64_t simulation_id,
                           const AverageStatsRecord& averageStats) const;
    /// @brief Save average stats to a CSV file.
    /// @param datetime The datetime of the data entry
    /// @param time_step The time step of the data entry
    /// @param n_valid_edges The number of valid edges (i.e. edges with speed observations)
    /// @param mean_speed The mean speed of the agents in kilometers per hour
    /// @param std_speed The standard deviation of the speed of the agents in kilometers per hour
    /// @param mean_density The mean density of the streets in vehicles per kilometer
    /// @param std_density The standard deviation of the density of the streets in vehicles per kilometer
    /// @param mean_traveltime The mean travel time of the agents in seconds
    /// @param meanQueueLength The mean queue length of the streets
    void m_saveAvgStatsCSV(const std::string& datetime,
                           const std::int64_t time_step,
                           const AverageStatsRecord& averageStats) const;
    /// @brief Initialize the travel data table.
    /// This table contains the travel data of the agents. Columns are:
    /// - id: The entry id (auto-incremented)
    /// - simulation_id: The simulation id
    /// - datetime: The datetime of the data entry
    /// - time_step: The time step of the data entry
    /// - distance_m: The distance travelled by the agent in meters
    /// - travel_time_s: The travel time of the agent in seconds
    void m_initTravelDataTable() const;
    /// @brief Save travel data to the database using a batch insert.
    /// @param datetime The datetime of the data entry
    /// @param time_step The time step of the data entry
    /// @param simulation_id The id of the simulation
    /// @param travelDTs A vector of pairs containing the distance travelled by the agent in meters and the travel time of the agent in seconds
    void m_saveTravelDataSQL(
        const std::string& datetime,
        const std::int64_t time_step,
        const std::int64_t simulation_id,
        tbb::concurrent_vector<std::pair<double, double>> travelDTs) const;
    /// @brief Save travel data to a CSV file.
    /// @param datetime The datetime of the data entry
    /// @param time_step The time step of the data entry
    /// @param travelDTs A vector of pairs containing the distance travelled by the agent in meters and the travel time of the agent in seconds
    void m_saveTravelDataCSV(
        const std::string& datetime,
        const std::int64_t time_step,
        tbb::concurrent_vector<std::pair<double, double>> travelDTs) const;
    /// @brief Initialize the agent data table.
    /// This table contains the agent data of the agents. Columns are:
    /// - id: The entry id (auto-incremented)
    /// - simulation_id: The simulation id
    /// - agent_id: The id of the agent
    /// - edge_id: The id of the edge
    /// - time_step_in: The time step of the data entry
    /// - time_step_out: The time step of the data entry
    void m_initAgentDataTable() const;
    /// @brief Save agent data to the database using a batch insert.
    /// @param time_step The time step of the data entry
    /// @param simulation_id The id of the simulation
    /// @param agentData A concurrent unordered map containing the agent data to be saved, where the key is the edge id and the value is a vector of tuples containing the edge id, the time step in and the time step out
    void m_saveAgentDataSQL(
        const std::int64_t time_step,
        const std::int64_t simulation_id,
        tbb::concurrent_unordered_map<
            Id,
            std::vector<std::tuple<Id, std::time_t, std::time_t>>> agentData) const;
    /// @brief Save agent data to a CSV file.
    /// @param time_step The time step of the data entry
    /// @param agentData A concurrent unordered map containing the agent data to be saved, where the key is the edge id and the value is a vector of tuples containing the edge id, the time step in and the time step out
    void m_saveAgentDataCSV(
        const std::int64_t time_step,
        tbb::concurrent_unordered_map<
            Id,
            std::vector<std::tuple<Id, std::time_t, std::time_t>>> agentData) const;
    void m_dumpNetwork() const;
    void m_preparePersistence();
    void m_flushStepData(StepDataResult&& stepData);

    void m_runDefault(std::vector<std::size_t> const& nAgentsPerTimeStep,
                      std::optional<std::time_t> const deltaT = std::nullopt);
    void m_runSlowCharge(std::size_t const nInitialAgents,
                         std::time_t const agentInsertionDeltaT,
                         std::time_t const checkDeltaT,
                         std::size_t const agentIncrement = 1);

  public:
    /// @brief Construct a new TrafficSimulator with a generated simulation id.
    TrafficSimulator();
    /// @brief Construct a new TrafficSimulator and import its configuration from JSON.
    /// @param jsonConfigFile The path to the JSON configuration file.
    explicit TrafficSimulator(std::string_view const jsonConfigFile);

    /// @brief Import a JSON configuration file and apply it to the simulator.
    /// @param jsonConfigFile The path to the JSON configuration file.
    void importConfig(std::string_view const jsonConfigFile);
    /// @brief Connect to a SQLite database, creating it if it doesn't exist, and executing optional initialization queries
    /// @param dbPath The path to the SQLite database file
    /// @param queries Optional SQL queries to execute upon connecting to the database (default is a set of pragmas for performance optimization : "PRAGMA busy_timeout = 5000;PRAGMA journal_mode = WAL;PRAGMA synchronous=NORMAL;PRAGMA temp_store=MEMORY;PRAGMA cache_size=-20000;")
    void connectDataBase(
        std::string_view const dbPath,
        std::string_view const queries =
            "PRAGMA busy_timeout = 5000;PRAGMA journal_mode = WAL;PRAGMA "
            "synchronous=NORMAL;PRAGMA temp_store=MEMORY;PRAGMA cache_size=-20000;");
    /// @brief Import a road network and build the simulation dynamics.
    /// @param edgesFile The edges CSV file.
    /// @param nodePropertiesFile Optional node-properties CSV file.
    void importRoadNetwork(std::string_view const edgesFile,
                           std::string_view const nodePropertiesFile = std::string_view());

    /// @brief Configure the path-update cadence forwarded to the dynamics engine.
    /// @param deltaT The update cadence in time steps.
    /// @param throw_on_empty Whether an empty itinerary path should throw.
    void updatePaths(std::time_t const deltaT = 0, bool const throw_on_empty = true);

    /// @brief Configure the data-saving behavior.
    void saveData(std::time_t const savingInterval,
                  bool const saveAverageStats = false,
                  bool const saveStreetData = false,
                  bool const saveTravelData = false,
                  bool const saveAgentData = false);

    /// @brief Set the name of the simulation
    /// @param name The name of the simulation
    void setName(std::string_view const name);

    /// @brief Set the output prefix used for generated CSV files and database paths.
    /// @param prefix The prefix or directory path.
    void setOutputPrefix(std::string_view const prefix);

    /// @brief Set the simulation time frame.
    /// @param initTime The simulation start time.
    /// @param endTime Optional simulation end time.
    void setTimeFrame(std::time_t const initTime,
                      std::optional<std::time_t> const endTime = std::nullopt);
    /// @brief Set the strategy used when inserting new agents.
    /// @param insertionMethod The insertion method to use.
    void setAgentInsertionMethod(AgentInsertionMethod const insertionMethod) noexcept {
      m_agentInsertionMethod = insertionMethod;
    }

    /// @brief Run the simulation until the configured end time.
    inline void run(std::vector<std::size_t> const& nAgentsPerTimeStep,
                    std::optional<std::time_t> const deltaT = std::nullopt) {
      if (m_dynamics == nullptr) {
        throw std::runtime_error(
            "Cannot run the simulation without imported road network dynamics.");
      }
      m_dynamics->prepareNetwork();
      m_runDefault(nAgentsPerTimeStep, deltaT);
    }

    inline void run(std::size_t const nInitialAgents,
                    std::time_t const agentInsertionDeltaT,
                    std::time_t const checkDeltaT,
                    std::size_t const agentIncrement = 1) {
      if (m_dynamics == nullptr) {
        throw std::runtime_error(
            "Cannot run the simulation without imported road network dynamics.");
      }
      m_dynamics->prepareNetwork();
      m_runSlowCharge(nInitialAgents, agentInsertionDeltaT, checkDeltaT, agentIncrement);
    }

    /// @brief Get the database connection (const version)
    /// @return const SQLite::Database const*, The database connection
    inline auto* database() const { return m_database.get(); }

    /// @brief Get the dynamics engine managed by this simulator.
    inline auto const* dynamics() const { return m_dynamics.get(); }
    /// @brief Get the mutable dynamics engine managed by this simulator.
    inline auto* dynamics() { return m_dynamics.get(); }
    /// @brief Get the id of the simulation
    /// @return Id, The id of the simulation
    inline auto id() const { return m_id; };
    /// @brief Get the simulation start time.
    inline auto initTime() const { return m_initTime; }
    /// @brief Get the formatted simulation start time.
    inline auto strInitTime() const { return m_timeToStr(m_initTime); }
    /// @brief Get the simulation end time.
    inline auto endTime() const { return m_endTime; }
    /// @brief Get the formatted simulation end time.
    inline auto strEndTime() const { return m_timeToStr(m_endTime); }
    /// @brief Get the name of the simulation
    /// @return const std::string&, The name of the simulation
    inline auto const& name() const { return m_name; };
    /// @brief Get a safe name string for filenames (spaces replaced by underscores)
    /// @return std::string, The safe name string
    inline auto const& safeName() const { return m_safeName; };
  };
}  // namespace dsf::mobility
