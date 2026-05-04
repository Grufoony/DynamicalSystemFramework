#pragma once

#include <SQLiteCpp/SQLiteCpp.h>

#include "FirstOrderDynamics.hpp"
#include "RoadNetwork.hpp"

namespace dsf::mobility {
    class TrafficSimulator {
        std::unique_ptr<FirstOrderDynamics> m_dynamics;
        std::unique_ptr<SQLite::Database> m_database;
        std::string m_databasePath;
        std::string m_databaseQueries;
        std::vector<std::size_t> m_nAgentsPerTimeStep;
        AgentInsertionMethod m_agentInsertionMethod{AgentInsertionMethod::RANDOM};
        std::time_t m_initTime = 0;
        std::time_t m_endTime = 0;
        std::time_t m_agentInsertionDeltaT = 0;
        std::optional<std::time_t> m_savingInterval{std::nullopt};
        bool m_saveAverageStats{false};
        bool m_saveStreetData{false};
        bool m_saveTravelData{false};
        bool m_saveAgentData{false};
        std::deque<StepDataResult> m_pendingStepData;

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

        inline std::string m_safeName() const {
            if (m_dynamics == nullptr) {
                return "unnamed_simulation";
            }
            std::string safeName = m_dynamics->name();
            std::replace(safeName.begin(), safeName.end(), ' ', '_');
            return safeName;
        }

        void m_initStreetTable() const;
        void m_saveStreetDataSQL(const std::string& datetime,
                                 const std::int64_t time_step,
                                 const std::int64_t simulation_id,
                                 tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const;
        void m_saveStreetDataCSV(const std::string& datetime,
                                 const std::int64_t time_step,
                                 const std::int64_t simulation_id,
                                 tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const;
        void m_initAvgStatsTable() const;
        void m_saveAvgStatsSQL(const std::string& datetime,
                               const std::int64_t time_step,
                               const std::int64_t simulation_id,
                               const AverageStatsRecord& averageStats) const;
        void m_saveAvgStatsCSV(const std::string& datetime,
                               const std::int64_t time_step,
                               const std::int64_t simulation_id,
                               const AverageStatsRecord& averageStats) const;
        void m_initTravelDataTable() const;
        void m_saveTravelDataSQL(const std::string& datetime,
                                 const std::int64_t time_step,
                                 const std::int64_t simulation_id,
                                 tbb::concurrent_vector<std::pair<double, double>> travelDTs) const;
        void m_saveTravelDataCSV(const std::string& datetime,
                                 const std::int64_t time_step,
                                 const std::int64_t simulation_id,
                                 tbb::concurrent_vector<std::pair<double, double>> travelDTs) const;
        void m_initAgentDataTable() const;
        void m_saveAgentDataSQL(
            const std::int64_t time_step,
            const std::int64_t simulation_id,
            tbb::concurrent_unordered_map<Id, std::vector<std::tuple<Id, std::time_t, std::time_t>>> agentData) const;
        void m_saveAgentDataCSV(
            const std::int64_t time_step,
            const std::int64_t simulation_id,
            tbb::concurrent_unordered_map<Id, std::vector<std::tuple<Id, std::time_t, std::time_t>>> agentData) const;
        void m_dumpNetwork() const;
        void m_preparePersistence();
        void m_flushStepData(StepDataResult&& stepData);
        

    public:
        TrafficSimulator() = default;

        /// @brief Connect to a SQLite database, creating it if it doesn't exist, and executing optional initialization queries
        /// @param dbPath The path to the SQLite database file
        /// @param queries Optional SQL queries to execute upon connecting to the database (default is a set of pragmas for performance optimization : "PRAGMA busy_timeout = 5000;PRAGMA journal_mode = WAL;PRAGMA synchronous=NORMAL;PRAGMA temp_store=MEMORY;PRAGMA cache_size=-20000;")
        void connectDataBase(
            std::string_view const dbPath,
            std::string_view const queries =
                "PRAGMA busy_timeout = 5000;PRAGMA journal_mode = WAL;PRAGMA "
                "synchronous=NORMAL;PRAGMA temp_store=MEMORY;PRAGMA cache_size=-20000;");
        void importRoadNetwork(std::string_view const edgesFile, std::string_view const nodePropertiesFile = std::string_view());

        /// @brief Configure the data-saving behavior.
        void saveData(std::time_t const savingInterval,
                      bool const saveAverageStats = false,
                      bool const saveStreetData = false,
                      bool const saveTravelData = false,
                      bool const saveAgentData = false);

        void setTimeFrame(std::time_t const initTime, std::optional<std::time_t> const endTime = std::nullopt);
        void setNAgentsPerTimeStep(std::vector<std::size_t> const& nAgentsPerTimeStep, std::optional<std::time_t> const deltaT = std::nullopt);
        void setAgentInsertionMethod(AgentInsertionMethod const insertionMethod) noexcept { m_agentInsertionMethod = insertionMethod; }

        /// @brief Run the simulation until the configured end time.
        void run(bool const reinsertAgents = false);

        /// @brief Get the database connection (const version)
        /// @return const SQLite::Database const*, The database connection
        inline auto* database() const { return m_database.get(); }

        inline auto const* dynamics() const { return m_dynamics.get(); }
        inline auto* dynamics() { return m_dynamics.get(); }

        inline auto initTime() const { return m_initTime; }
        inline auto strInitTime() const { return m_timeToStr(m_initTime); }
        inline auto endTime() const { return m_endTime; }
        inline auto strEndTime() const { return m_timeToStr(m_endTime); }
        inline auto agentInsertionDeltaT() const { return m_agentInsertionDeltaT; }
    };
}