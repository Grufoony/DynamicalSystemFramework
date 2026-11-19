#include "TrafficSimulator.hpp"
#include "../utility/csv_writer.hpp"
#include "../utility/progress_bar.hpp"
#include "../dsf.hpp"

#include <simdjson.h>
#include <sqlite3.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace dsf::mobility {
  namespace {
    /// @brief Percentage of @p value over @p total, or 0 when @p total is 0.
    constexpr double percentOf(std::size_t const value,
                               std::size_t const total) noexcept {
      return total == 0 ? 0.
                        : static_cast<double>(value) * 100. / static_cast<double>(total);
    }

    // ----- SQLite layout, version DB_SCHEMA_VERSION -----
    // Per-step rows carry no datetime (init_time + time_step, see the *_view views) and no
    // data repeated from the network tables. Rows are keyed by simulation_id, and the
    // networks by network_id (see networkFingerprint), so one database holds any number of
    // simulations on any number of networks.
    struct TableSchema {
      std::string_view name;
      std::string_view columns;
      /// @brief The columns of the <name>_simulation_idx index, empty for none
      std::string_view indexColumns;
      /// @brief The SELECT of the <name>_view view, empty for none
      std::string_view view;
    };

    /// @brief One row per simulation id (the last configuration written for it)
    constexpr TableSchema kSimulationInfo{
        "simulation_info",
        "id INTEGER PRIMARY KEY, name TEXT, init_time TEXT, end_time TEXT, "
        "agent_insertion_method TEXT, network_id INTEGER, dsf_version TEXT",
        "",
        ""};
    /// @brief The streets of every network stored, coilcode is NULL without a counter
    constexpr TableSchema kEdges{
        "edges",
        "network_id INTEGER NOT NULL, id INTEGER NOT NULL, source INTEGER NOT NULL, "
        "target INTEGER NOT NULL, length REAL NOT NULL, maxspeed REAL NOT NULL, "
        "name TEXT, nlanes INTEGER NOT NULL, coilcode TEXT, geometry TEXT NOT NULL, "
        "PRIMARY KEY (network_id, id)",
        "",
        ""};
    /// @brief The nodes of every network stored, type is traffic_light, roundabout or NULL
    constexpr TableSchema kNodes{"nodes",
                                 "network_id INTEGER NOT NULL, id INTEGER NOT NULL, "
                                 "type TEXT, geometry TEXT, PRIMARY KEY (network_id, id)",
                                 "",
                                 ""};
    /// @brief Street data. avg/std speed are NULL without observations, counts without a
    ///   counter
    constexpr TableSchema kRoadData{
        "road_data",
        "id INTEGER PRIMARY KEY, simulation_id INTEGER NOT NULL, "
        "time_step INTEGER NOT NULL, street_id INTEGER NOT NULL, density_vpk REAL, "
        "avg_speed_kph REAL, std_speed_kph REAL, n_observations INTEGER, counts INTEGER, "
        "queue_length INTEGER",
        "simulation_id, time_step",
        "SELECT t.*, datetime(s.init_time, '+' || t.time_step || ' seconds') AS "
        "datetime, "
        "e.coilcode AS coil FROM road_data t "
        "JOIN simulation_info s ON s.id = t.simulation_id "
        "LEFT JOIN edges e ON e.network_id = s.network_id AND e.id = t.street_id"};
    /// @brief Network averages. Speeds and travel time are NULL without observations
    constexpr TableSchema kAvgStats{
        "avg_stats",
        "id INTEGER PRIMARY KEY, simulation_id INTEGER NOT NULL, "
        "time_step INTEGER NOT NULL, n_ghost_agents INTEGER NOT NULL, "
        "n_agents INTEGER NOT NULL, mean_speed_kph REAL, std_speed_kph REAL, "
        "mean_density_vpk REAL NOT NULL, std_density_vpk REAL NOT NULL, "
        "mean_travel_time_s REAL, mean_queue_length REAL NOT NULL",
        "simulation_id, time_step",
        "SELECT t.*, datetime(s.init_time, '+' || t.time_step || ' seconds') AS datetime "
        "FROM avg_stats t JOIN simulation_info s ON s.id = t.simulation_id"};
    /// @brief One row per trip ended in the saving interval
    constexpr TableSchema kTravelData{
        "travel_data",
        "id INTEGER PRIMARY KEY, simulation_id INTEGER NOT NULL, "
        "time_step INTEGER NOT NULL, distance_m REAL NOT NULL, "
        "travel_time_s REAL NOT NULL",
        "simulation_id, time_step",
        "SELECT t.*, datetime(s.init_time, '+' || t.time_step || ' seconds') AS datetime "
        "FROM travel_data t JOIN simulation_info s ON s.id = t.simulation_id"};
    /// @brief One row per agent and street travelled
    constexpr TableSchema kAgentData{
        "agent_data",
        "id INTEGER PRIMARY KEY, simulation_id INTEGER NOT NULL, "
        "agent_id INTEGER NOT NULL, edge_id INTEGER NOT NULL, "
        "time_step_in INTEGER NOT NULL, time_step_out INTEGER NOT NULL",
        "simulation_id, time_step_in",
        ""};
    /// @brief Turn counts from source to target street, zero counts are not stored
    constexpr TableSchema kTurnCounts{
        "turn_counts",
        "id INTEGER PRIMARY KEY, simulation_id INTEGER NOT NULL, "
        "time_step INTEGER NOT NULL, source_edge_id INTEGER NOT NULL, "
        "target_edge_id INTEGER NOT NULL, counts INTEGER NOT NULL",
        "simulation_id, time_step",
        "SELECT t.*, datetime(s.init_time, '+' || t.time_step || ' seconds') AS datetime "
        "FROM turn_counts t JOIN simulation_info s ON s.id = t.simulation_id"};
    /// @brief The scheduled street status changes, when they were made (status is open or
    ///   closed)
    constexpr TableSchema kStreetStatus{
        "street_status",
        "id INTEGER PRIMARY KEY, simulation_id INTEGER NOT NULL, "
        "time_step INTEGER NOT NULL, street_id INTEGER NOT NULL, status TEXT NOT NULL",
        "simulation_id, time_step",
        "SELECT t.*, datetime(s.init_time, '+' || t.time_step || ' seconds') AS datetime "
        "FROM street_status t JOIN simulation_info s ON s.id = t.simulation_id"};

    constexpr std::array kAllTables{&kSimulationInfo,
                                    &kEdges,
                                    &kNodes,
                                    &kRoadData,
                                    &kAvgStats,
                                    &kTravelData,
                                    &kAgentData,
                                    &kTurnCounts,
                                    &kStreetStatus};

    void createTable(SQLite::Database& db, TableSchema const& table) {
      db.exec(
          std::format("CREATE TABLE IF NOT EXISTS {} ({})", table.name, table.columns));
      if (!table.indexColumns.empty()) {
        db.exec(std::format("CREATE INDEX IF NOT EXISTS {0}_simulation_idx ON {0} ({1})",
                            table.name,
                            table.indexColumns));
      }
      if (!table.view.empty()) {
        db.exec(std::format(
            "CREATE VIEW IF NOT EXISTS {}_view AS {}", table.name, table.view));
      }
    }

    // ----- Network rows and fingerprint -----
    struct EdgeRow {
      std::int64_t id, source, target;
      double length, maxSpeed;
      std::string name;
      std::int64_t nLanes;
      std::optional<std::string> coilCode;
      std::string geometry;
    };
    struct NodeRow {
      std::int64_t id;
      std::optional<std::string> type, geometry;
    };
    struct NetworkRows {
      std::vector<EdgeRow> edges;  // Sorted by id
      std::vector<NodeRow> nodes;  // Sorted by id
    };

    /// @brief The rows of the edges and nodes tables for a network, sorted by id
    NetworkRows networkRows(RoadNetwork const& network) {
      NetworkRows rows;
      rows.edges.reserve(network.nEdges());
      for (auto const& [edgeId, pEdge] : network.edges()) {
        auto const counterName{pEdge->counterName()};
        rows.edges.push_back(
            EdgeRow{.id = static_cast<std::int64_t>(edgeId),
                    .source = static_cast<std::int64_t>(pEdge->source()->id()),
                    .target = static_cast<std::int64_t>(pEdge->target()->id()),
                    .length = pEdge->length(),
                    .maxSpeed = pEdge->maxSpeed(),
                    .name = pEdge->name(),
                    .nLanes = static_cast<std::int64_t>(pEdge->nLanes()),
                    .coilCode = counterName != "N/A"
                                    ? std::make_optional(std::string(counterName))
                                    : std::nullopt,
                    .geometry = std::format("{}", pEdge->geometry())});
      }
      rows.nodes.reserve(network.nNodes());
      for (auto const& [nodeId, pNode] : network.nodes()) {
        std::optional<std::string> type;
        if (pNode->isTrafficLight()) {
          type = "traffic_light";
        } else if (pNode->isRoundabout()) {
          type = "roundabout";
        }
        rows.nodes.push_back(NodeRow{
            .id = static_cast<std::int64_t>(nodeId),
            .type = std::move(type),
            .geometry = pNode->geometry().has_value()
                            ? std::make_optional(std::format("{}", *pNode->geometry()))
                            : std::nullopt});
      }
      // The network maps are unordered, and their order differs between standard libraries
      std::ranges::sort(rows.edges, {}, &EdgeRow::id);
      std::ranges::sort(rows.nodes, {}, &NodeRow::id);
      return rows;
    }

    /// @brief The rows of edges and nodes tables of the version 1 layout (no network_id)
    NetworkRows legacyNetworkRows(SQLite::Database& db) {
      auto optionalText = [](SQLite::Column const& column) {
        return column.isNull() ? std::nullopt : std::make_optional(column.getString());
      };
      NetworkRows rows;
      SQLite::Statement edges(
          db,
          "SELECT id, source, target, length, maxspeed, name, nlanes, "
          "coilcode, geometry FROM edges ORDER BY id");
      while (edges.executeStep()) {
        rows.edges.push_back(EdgeRow{.id = edges.getColumn(0).getInt64(),
                                     .source = edges.getColumn(1).getInt64(),
                                     .target = edges.getColumn(2).getInt64(),
                                     .length = edges.getColumn(3).getDouble(),
                                     .maxSpeed = edges.getColumn(4).getDouble(),
                                     .name = edges.getColumn(5).getString(),
                                     .nLanes = edges.getColumn(6).getInt64(),
                                     .coilCode = optionalText(edges.getColumn(7)),
                                     .geometry = edges.getColumn(8).getString()});
      }
      if (db.tableExists("nodes")) {
        SQLite::Statement nodes(db, "SELECT id, type, geometry FROM nodes ORDER BY id");
        while (nodes.executeStep()) {
          rows.nodes.push_back(NodeRow{.id = nodes.getColumn(0).getInt64(),
                                       .type = optionalText(nodes.getColumn(1)),
                                       .geometry = optionalText(nodes.getColumn(2))});
        }
      }
      return rows;
    }

    /// @brief A 64-bit FNV-1a hash of the network rows, cleared of its sign bit
    /// @details Stable across platforms and runs: every value is hashed with a type tag,
    ///   integers and doubles by their bits (little-endian), strings with their length.
    ///   std::hash is not used, as it differs between standard libraries
    std::int64_t networkFingerprint(NetworkRows const& rows) {
      std::uint64_t hash{14695981039346656037ULL};
      auto byte = [&hash](std::uint8_t const value) {
        hash ^= value;
        hash *= 1099511628211ULL;
      };
      auto word = [&byte](std::uint64_t const value) {
        for (int shift{0}; shift < 64; shift += 8) {
          byte(static_cast<std::uint8_t>(value >> shift));
        }
      };
      auto integer = [&](std::int64_t const value) {
        byte('i');
        word(std::bit_cast<std::uint64_t>(value));
      };
      auto real = [&](double const value) {
        byte('d');
        word(std::bit_cast<std::uint64_t>(value));
      };
      auto text = [&](std::string_view const value) {
        byte('s');
        word(value.size());
        for (auto const c : value) {
          byte(static_cast<std::uint8_t>(c));
        }
      };
      auto optionalText = [&](std::optional<std::string> const& value) {
        if (value.has_value()) {
          text(*value);
        } else {
          byte('n');
        }
      };
      for (auto const& edge : rows.edges) {
        byte('E');
        integer(edge.id);
        integer(edge.source);
        integer(edge.target);
        real(edge.length);
        real(edge.maxSpeed);
        text(edge.name);
        integer(edge.nLanes);
        optionalText(edge.coilCode);
        text(edge.geometry);
      }
      for (auto const& node : rows.nodes) {
        byte('N');
        integer(node.id);
        optionalText(node.type);
        optionalText(node.geometry);
      }
      return static_cast<std::int64_t>(hash & 0x7FFF'FFFF'FFFF'FFFFULL);
    }

    // ----- Schema migrations -----
    /// @brief Rebuild a table with its current layout, keeping the columns it shares with
    ///   the old one, and setting network_id to the given value if the new layout has it
    void rebuildTable(SQLite::Database& db,
                      TableSchema const& table,
                      std::optional<std::int64_t> const networkId = std::nullopt) {
      if (!db.tableExists(std::string(table.name))) {
        return;
      }
      auto const oldName{std::format("{}_old", table.name)};
      // An index keeps its name when its table is renamed: drop it, or it would stop the
      // new table from getting its own
      db.exec(std::format("DROP INDEX IF EXISTS {}_simulation_idx", table.name));
      db.exec(std::format("ALTER TABLE {} RENAME TO {}", table.name, oldName));
      createTable(db, table);
      auto columnNames = [&db](std::string_view const tableName) {
        std::vector<std::string> names;
        SQLite::Statement query(db, std::format("PRAGMA table_info({})", tableName));
        while (query.executeStep()) {
          names.push_back(query.getColumn(1).getString());
        }
        return names;
      };
      auto const oldColumns{columnNames(oldName)};
      std::string columns, values;
      for (auto const& column : columnNames(table.name)) {
        if (column == "network_id" && networkId.has_value()) {
          columns += columns.empty() ? "network_id" : ", network_id";
          values += values.empty() ? "?" : ", ?";
        } else if (std::ranges::find(oldColumns, column) != oldColumns.end()) {
          columns += columns.empty() ? column : ", " + column;
          values += values.empty() ? column : ", " + column;
        }
      }
      SQLite::Statement copy(db,
                             std::format("INSERT INTO {} ({}) SELECT {} FROM {}",
                                         table.name,
                                         columns,
                                         values,
                                         oldName));
      if (networkId.has_value()) {
        copy.bind(1, *networkId);
      }
      auto const nRows{copy.exec()};
      db.exec(std::format("DROP TABLE {}", oldName));
      spdlog::info("Migrated {} rows of table {}.", nRows, table.name);
    }

    /// @brief Version 1 (the layout before versioning) to version 2
    void migrateV1ToV2(SQLite::Database& db) {
      // The stored network gets the id a dump of the same network computes now
      std::optional<std::int64_t> networkId;
      if (db.tableExists("edges")) {
        try {
          networkId = networkFingerprint(legacyNetworkRows(db));
        } catch (SQLite::Exception const& e) {
          spdlog::warn("The network tables have an unknown layout ({}).", e.what());
        }
      }
      if (!networkId.has_value()) {
        // Without an id their rows cannot be keyed: keep them aside
        for (auto const* name : {"edges", "nodes"}) {
          if (db.tableExists(name)) {
            spdlog::warn("Table {0} is kept as {0}_legacy, without a network id.", name);
            db.exec(std::format("ALTER TABLE {0} RENAME TO {0}_legacy", name));
          }
        }
      }
      for (auto const* pTable : kAllTables) {
        auto const isNetworkTable{pTable == &kEdges || pTable == &kNodes};
        rebuildTable(db, *pTable, isNetworkTable ? networkId : std::nullopt);
      }
      if (networkId.has_value() && db.tableExists("simulation_info")) {
        SQLite::Statement update(db, "UPDATE simulation_info SET network_id = ?");
        update.bind(1, *networkId);
        update.exec();
        // Version 1 kept the first network only: later simulations on other networks
        // can no longer be told apart
        spdlog::info(
            "Simulations stored before the migration are assigned the stored network "
            "({}).",
            *networkId);
      }
    }

    /// @brief Bring the database to DB_SCHEMA_VERSION, within the caller's transaction
    /// @throws std::runtime_error if the database is newer than DB_SCHEMA_VERSION
    void migrateSchema(SQLite::Database& db) {
      // migrations[v - 1] brings version v to v + 1
      constexpr std::array<void (*)(SQLite::Database&), DB_SCHEMA_VERSION - 1> migrations{
          &migrateV1ToV2};
      auto version{db.execAndGet("PRAGMA user_version").getInt()};
      if (version > DB_SCHEMA_VERSION) {
        throw std::runtime_error(
            std::format("The database has layout version {}, newer than the one of this "
                        "dsf version ({}): use a newer dsf version or another database.",
                        version,
                        DB_SCHEMA_VERSION));
      }
      if (version == DB_SCHEMA_VERSION) {
        return;
      }
      if (version == 0) {
        // Before versioning, or a new database
        version = std::ranges::any_of(kAllTables,
                                      [&db](auto const* pTable) {
                                        return db.tableExists(std::string(pTable->name));
                                      })
                      ? 1
                      : DB_SCHEMA_VERSION;
      }
      if (version < DB_SCHEMA_VERSION) {
        spdlog::info("Migrating the database from layout version {} to {}.",
                     version,
                     DB_SCHEMA_VERSION);
        // Views name the tables they read: drop them before tables are renamed, they are
        // created again with the tables
        for (auto const* pTable : kAllTables) {
          db.exec(std::format("DROP VIEW IF EXISTS {}_view", pTable->name));
        }
        for (; version < DB_SCHEMA_VERSION; ++version) {
          migrations[static_cast<std::size_t>(version - 1)](db);
        }
      }
      db.exec(std::format("PRAGMA user_version = {}", DB_SCHEMA_VERSION));
    }
  }  // namespace

  void TrafficSimulator::m_logRunSummary(std::size_t const nAdded,
                                         std::size_t const nInserted,
                                         std::size_t const nArrived,
                                         std::size_t const nKilled,
                                         std::size_t const nRemaining) const {
    // Every inserted agent should be arrived, killed (stagnant pending agents included)
    // or remaining, so ghosts should be 0. Do the subtraction in a signed type anyway,
    // so an imbalance shows up as a negative number instead of wrapping around to ~1.8e19.
    auto const accounted = static_cast<std::int64_t>(nArrived) +
                           static_cast<std::int64_t>(nKilled) +
                           static_cast<std::int64_t>(nRemaining);
    auto const nGhosts = static_cast<std::int64_t>(nInserted) - accounted;
    spdlog::info(
        "Simulation completed. Total agents added: {}\n\tInserted: {} "
        "({:.2f}%)\n\tArrived: {} ({:.2f}%)\n\tKilled: {} ({:.2f}%)\n\tGhosts: {} "
        "({:.2f}%)\n\tRemaining: {} ({:.2f}%).",
        nAdded,
        nInserted,
        percentOf(nInserted, nAdded),
        nArrived,
        percentOf(nArrived, nInserted),
        nKilled,
        percentOf(nKilled, nInserted),
        nGhosts,
        nInserted == 0
            ? 0.
            : static_cast<double>(nGhosts) * 100. / static_cast<double>(nInserted),
        nRemaining,
        percentOf(nRemaining, nInserted));
  }

  void TrafficSimulator::m_createId() {
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

  std::string TrafficSimulator::m_generateCSVfilename(
      std::string_view const tableName) const {
    if (m_outputPrefix.empty()) {
      return std::format("{}_{}_{}.csv", m_id, m_safeName, tableName);
    }
    return std::format("{}{}.csv", m_outputPrefix, tableName);
  }

  TrafficSimulator::TrafficSimulator() { m_createId(); }
  TrafficSimulator::TrafficSimulator(std::string_view const jsonConfigPath) {
    m_createId();
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
    if (!generalConfig["output_basename"].error()) {
      setOutputPrefix(
          (output_folder /
           std::filesystem::path(generalConfig["output_basename"].get_string().value()))
              .string());
    } else {
      setOutputPrefix(output_folder.string());
    }
    {
      auto const name =
          require_field(generalConfig, "general", "name").get_string().value();
      setName(name);
    }
    // Init and End times
    {
      // Helper lambda to parse "YYYYMMDD hhmmss" or "YYYYMMDD" string into time_t
      auto parseTimeString = [](std::string_view s) -> std::time_t {
        // Pad with "000000" if hhmmss is omitted
        std::string normalized{s};
        if (normalized.size() == 8) {
          normalized += " 000000";
        }

        if (normalized.size() != 15) {
          throw std::invalid_argument("Invalid time string format: " + std::string(s));
        }

        std::tm tm{};
        tm.tm_year = std::stoi(normalized.substr(0, 4)) - 1900;
        tm.tm_mon = std::stoi(normalized.substr(4, 2)) - 1;
        tm.tm_mday = std::stoi(normalized.substr(6, 2));
        tm.tm_hour = std::stoi(normalized.substr(9, 2));
        tm.tm_min = std::stoi(normalized.substr(11, 2));
        tm.tm_sec = std::stoi(normalized.substr(13, 2));
        tm.tm_isdst = -1;  // let mktime determine DST

        std::time_t result = std::mktime(&tm);
        if (result == -1) {
          throw std::invalid_argument("mktime failed for: " + std::string(s));
        }
        return result;
      };
      auto parseTimeValue =
          [&parseTimeString](simdjson::dom::element elem) -> std::time_t {
        if (elem.is_uint64()) {
          return static_cast<std::time_t>(elem.get_uint64().value());
        }
        if (elem.is_string()) {
          return parseTimeString(elem.get_string().value());
        }
        throw std::invalid_argument("Time field must be a uint64 or a string");
      };

      std::time_t initTime{0};
      std::optional<std::time_t> endTime{std::nullopt};

      if (!generalConfig["init_time"].error()) {
        initTime = parseTimeValue(generalConfig["init_time"].value());
      }
      if (!generalConfig["end_time"].error()) {
        endTime = parseTimeValue(generalConfig["end_time"].value());
      }
      setTimeFrame(initTime, endTime);
    }
    bool hasDynamicODs{false};
    {
      auto const dynamicODsConfig = generalConfig["dynamic_ods"];
      if (!dynamicODsConfig.error()) {
        hasDynamicODs = true;
        // Expect a list of {"time": <time>, "file": <file_path>} objects
        auto const dynamicODsArray = dynamicODsConfig.get_array();
        if (dynamicODsArray.error()) {
          throw std::runtime_error(std::format(
              "'dynamic_ods' field must be an array in JSON file '{}'", jsonConfigFile));
        }
        for (auto const& item : dynamicODsArray) {
          if (!item.is_object()) {
            throw std::runtime_error(std::format(
                "Each item in 'dynamic_ods' array must be an object in JSON file '{}'",
                jsonConfigFile));
          }
          auto const timeField = item["time"];
          auto const fileField = item["file"];
          if (timeField.error() || fileField.error()) {
            throw std::runtime_error(
                std::format("Each item in 'dynamic_ods' array must have 'time' and "
                            "'file' fields in JSON file '{}'",
                            jsonConfigFile));
          }
          m_dynamicODsUpdate.push(
              ODsUpdate{.time = static_cast<std::time_t>(timeField.get_uint64().value()),
                        .file = std::string(fileField.get_string().value())});
        }
      }
    }
    {
      auto const roadEventsConfig = generalConfig["road_events"];
      if (!roadEventsConfig.error()) {
        // Expect a list of {"time": <step>, "status": "open"|"closed", "street_ids": [...]}
        // objects, or with "street_name": <name> instead of "street_ids"
        auto const roadEventsArray = roadEventsConfig.get_array();
        if (roadEventsArray.error()) {
          throw std::runtime_error(std::format(
              "'road_events' field must be an array in JSON file '{}'", jsonConfigFile));
        }
        auto const invalidEvent = [&](std::string_view const reason) {
          return std::runtime_error(
              std::format("Each item in 'road_events' array must {} in JSON file '{}'",
                          reason,
                          jsonConfigFile));
        };
        for (auto const& item : roadEventsArray) {
          if (!item.is_object()) {
            throw invalidEvent("be an object");
          }
          auto const timeField = item["time"];
          if (timeField.error() || !timeField.is_uint64() ||
              timeField.get_uint64().value() >
                  static_cast<std::uint64_t>(std::numeric_limits<std::time_t>::max())) {
            throw invalidEvent("have a non-negative integer 'time' field");
          }
          auto const statusField = item["status"];
          if (statusField.error() || !statusField.is_string()) {
            throw invalidEvent("have a 'status' field ('open' or 'closed')");
          }
          std::string statusStr{statusField.get_string().value()};
          std::ranges::transform(statusStr, statusStr.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
          });
          RoadStatus status{RoadStatus::OPEN};
          if (statusStr == "closed") {
            status = RoadStatus::CLOSED;
          } else if (statusStr == "open") {
            status = RoadStatus::OPEN;
          } else {
            throw invalidEvent("have a 'status' field ('open' or 'closed')");
          }
          auto const time{static_cast<std::time_t>(timeField.get_uint64().value())};
          auto const idsField = item["street_ids"];
          auto const nameField = item["street_name"];
          bool const bHasIds{!idsField.error()};
          bool const bHasName{!nameField.error()};
          if (bHasIds == bHasName) {
            throw invalidEvent(
                "have exactly one of the 'street_ids' and 'street_name' fields");
          }
          if (bHasName) {
            if (!nameField.is_string()) {
              throw invalidEvent("have a string 'street_name' field");
            }
            scheduleStreetStatusByName(
                time, std::string(nameField.get_string().value()), status);
            continue;
          }
          auto const idsArray = idsField.get_array();
          if (idsArray.error()) {
            throw invalidEvent("have an array 'street_ids' field");
          }
          std::vector<Id> streetIds;
          for (auto const& idElement : idsArray) {
            if (!idElement.is_uint64()) {
              throw invalidEvent("have non-negative integer street ids");
            }
            streetIds.push_back(static_cast<Id>(idElement.get_uint64().value()));
          }
          scheduleStreetStatusById(time, std::move(streetIds), status);
        }
      }
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
      // Must be set before the import: it defines the capacity of the streets without one
      if (!roadNetworkConfig["mean_vehicle_length"].error()) {
        Road::setMeanVehicleLength(
            roadNetworkConfig["mean_vehicle_length"].get_double().value());
      }
      importRoadNetwork(edgesFile.string(), nodePropertiesFile.string());
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
      if (!dynamicsConfig["max_concurrency"].error()) {
        m_dynamics->setConcurrency(dynamicsConfig["max_concurrency"].get_int64().value());
      }
      if (!dynamicsConfig["allow_uturns"].error()) {
        // before any transition matrix: it decides how the matrix's U-turn entries are read
        m_dynamics->setAllowRandomUTurns(
            dynamicsConfig["allow_uturns"].get_bool().value());
      }
      if (!dynamicsConfig["edge_weight"].error()) {
        // The distance cost is used by "datt"
        auto const distanceCost{
            dynamicsConfig["distance_cost"].error()
                ? 0.1
                : dynamicsConfig["distance_cost"].get_double().value()};
        m_dynamics->setEdgeWeight(dynamicsConfig["edge_weight"].get_string().value(),
                                  distanceCost);
      }
      if (!dynamicsConfig["path_randomness"].error()) {
        m_dynamics->setPathRandomness(
            dynamicsConfig["path_randomness"].get_double().value());
      }
      if (!dynamicsConfig["route_algorithm"].error()) {
        auto const algorithm{dynamicsConfig["route_algorithm"].get_string().value()};
        if (algorithm == "ASTAR") {
          m_dynamics->setRouteAlgorithm(RouteAlgorithm::ASTAR);
        } else if (algorithm == "DIJKSTRA") {
          m_dynamics->setRouteAlgorithm(RouteAlgorithm::DIJKSTRA);
        } else {
          throw std::runtime_error(
              std::format("Invalid 'route_algorithm' value in JSON file '{}': {}",
                          jsonConfigFile,
                          algorithm));
        }
      }
      if (!dynamicsConfig["intelligent_fraction"].error()) {
        m_dynamics->setIntelligentAgentsFraction(
            dynamicsConfig["intelligent_fraction"].get_double().value());
      }
      if (!dynamicsConfig["mode"].error()) {
        auto const mode =
            require_field(dynamicsConfig, "dynamics", "mode").get_string().value();
        if (mode == "EDGES") {
          m_dynamics->setMode(DynamicsMode::EDGES);
        } else if (mode == "NODES") {
          m_dynamics->setMode(DynamicsMode::NODES);
        } else {
          throw std::runtime_error(std::format(
              "Invalid 'mode' value in JSON file '{}': {}", jsonConfigFile, mode));
        }
      }
      auto const agentInsertionMethod =
          require_field(dynamicsConfig, "dynamics", "agent_insertion_method")
              .get_string()
              .value();
      if (agentInsertionMethod == "RANDOM") {
        m_agentInsertionMethod = AgentInsertionMethod::RANDOM;
      } else if (agentInsertionMethod == "RANDOM_WEIGHTED_ORIGIN") {
        m_agentInsertionMethod = AgentInsertionMethod::RANDOM_WEIGHTED_ORIGIN;
      } else if (agentInsertionMethod == "ODS") {
        m_agentInsertionMethod = AgentInsertionMethod::ODS;
      } else if (agentInsertionMethod == "RANDOM_ODS") {
        m_agentInsertionMethod = AgentInsertionMethod::RANDOM_ODS;
      } else if (agentInsertionMethod == "CONDITIONAL_RANDOM_ODS") {
        m_agentInsertionMethod = AgentInsertionMethod::CONDITIONAL_RANDOM_ODS;
      } else if (agentInsertionMethod == "UNIFORM") {
        m_agentInsertionMethod = AgentInsertionMethod::UNIFORM;
      } else {
        throw std::runtime_error(
            std::format("Invalid 'agent_insertion_method' value in JSON file '{}': {}",
                        jsonConfigFile,
                        agentInsertionMethod));
      }
      if (!dynamicsConfig["kill_stagnant_agents"].error()) {
        m_dynamics->killStagnantAgents(
            dynamicsConfig["kill_stagnant_agents"].get_double().value());
      }
      if (!dynamicsConfig["importODsFromCSV"].error()) {
        if (hasDynamicODs) {
          spdlog::warn(
              "Both 'dynamic_ods' and 'importODsFromCSV' configurations are present in "
              "JSON file '{}'. "
              "'dynamic_ods' will take precedence and 'importODsFromCSV' will be "
              "ignored.",
              jsonConfigFile);
        } else {
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
      if (!dynamicsConfig["importTransitionMatrixFromJSON"].error()) {
        auto const importTransitionMatrixConfig =
            dynamicsConfig["importTransitionMatrixFromJSON"];
        auto const transitionMatrixFile =
            input_folder /
            std::filesystem::path(require_field(importTransitionMatrixConfig,
                                                "importTransitionMatrixFromJSON",
                                                "file")
                                      .get_string()
                                      .value());
        m_dynamics->importTransitionMatrixFromJSON(transitionMatrixFile.string());
      }
      if (!dynamicsConfig["mean_travel_distance"].error()) {
        m_dynamics->setMeanTravelDistance(
            dynamicsConfig["mean_travel_distance"].get_double().value());
      }
      if (!dynamicsConfig["mean_travel_time"].error()) {
        m_dynamics->setMeanTravelTime(
            dynamicsConfig["mean_travel_time"].get_double().value());
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
                : false,
            save_data["turn_counts"].get_bool().has_value()
                ? save_data["turn_counts"].get_bool().value()
                : false);
      }
    }
  }

  void TrafficSimulator::m_runDefault(std::vector<std::size_t> const& nAgentsPerTimeStep,
                                      std::optional<std::time_t> const deltaT) {
    if (deltaT.has_value()) {
      if (deltaT.value() <= 0) {
        throw std::invalid_argument(std::format(
            "Agent insertion delta time ({}) must be positive.", deltaT.value()));
      }
      auto const scheduleEndTime = static_cast<std::time_t>(
          m_initTime + nAgentsPerTimeStep.size() * deltaT.value());
      if (m_endTime != 0 && m_endTime != scheduleEndTime) {
        spdlog::warn(
            "The configured end time ({}) does not match the agent insertion schedule "
            "({} insertions every {} seconds). Using the schedule's end time ({}).",
            m_timeToStr(m_endTime),
            nAgentsPerTimeStep.size(),
            deltaT.value(),
            m_timeToStr(scheduleEndTime));
      }
      m_endTime = scheduleEndTime;
    }
    std::time_t agentInsertionDeltaT = deltaT.value_or(0);

    if (nAgentsPerTimeStep.empty()) {
      throw std::runtime_error(
          "Cannot run the simulation without an agent insertion schedule.");
    }

    if (m_endTime != 0 && m_endTime < m_initTime) {
      throw std::runtime_error(std::format(
          "End time ({}) precedes the initial time ({}); the simulation would not run.",
          m_timeToStr(m_endTime),
          m_timeToStr(m_initTime)));
    }
    auto totalTimeSteps = static_cast<std::time_t>(m_endTime - m_initTime);
    auto const nInsertions{nAgentsPerTimeStep.size()};

    if (agentInsertionDeltaT == 0) {
      if (m_endTime > m_initTime) {
        agentInsertionDeltaT = totalTimeSteps / static_cast<std::time_t>(nInsertions);
        if (totalTimeSteps % static_cast<std::time_t>(nInsertions) != 0) {
          spdlog::warn(
              "Total simulation time ({} seconds) is not perfectly divisible by the "
              "number of agent insertion steps ({}). The last agent insertion step "
              "will occur at time {} instead of the end time {}.",
              totalTimeSteps,
              nInsertions,
              m_timeToStr(m_initTime + agentInsertionDeltaT * nInsertions),
              m_timeToStr(m_endTime));
        }
      }
      if (agentInsertionDeltaT == 0) {
        agentInsertionDeltaT = 1;
      }
    }
    if (m_endTime == 0) {
      m_endTime =
          m_initTime + static_cast<std::time_t>(agentInsertionDeltaT * nInsertions);
      totalTimeSteps = static_cast<std::time_t>(m_endTime - m_initTime);
    }

    m_checkStreetStatusUpdates();
    m_preparePersistence();

    spdlog::info(
        "Starting simulation run from {} to {} ({} time steps) with agent insertion "
        "every {} seconds.",
        m_timeToStr(m_initTime),
        m_timeToStr(m_endTime),
        totalTimeSteps,
        agentInsertionDeltaT);
    auto pbar = dsf::utility::default_progress_bar("Running simulation", totalTimeSteps);

    std::optional<std::time_t> nextODUpdateTime =
        m_dynamicODsUpdate.empty() ? std::nullopt
                                   : std::make_optional(m_dynamicODsUpdate.front().time);
    if (nextODUpdateTime.has_value() && *nextODUpdateTime != 0) {
      throw std::runtime_error(std::format(
          "First dynamic OD update time must be 0 (initial time). Current value: {}",
          *nextODUpdateTime));
    }
    for (std::time_t currentStep{0}; currentStep < totalTimeSteps; ++currentStep) {
      if (nextODUpdateTime.has_value() && currentStep == *nextODUpdateTime) {
        m_dynamics->importODsFromCSV(m_dynamicODsUpdate.front().file);
        m_dynamicODsUpdate.pop();
        nextODUpdateTime = m_dynamicODsUpdate.empty()
                               ? std::nullopt
                               : std::make_optional(m_dynamicODsUpdate.front().time);
      }
      m_applyStreetStatusUpdates(currentStep);
      if (currentStep % agentInsertionDeltaT == 0) {
        auto const insertionIndex =
            static_cast<std::size_t>(currentStep / agentInsertionDeltaT);
        if (insertionIndex < nInsertions) {
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
                                                                      m_saveAgentData,
                                                                      m_saveTurnCounts}
                                                    : StepDataRequest{});

      if (shouldSave) {
        spdlog::debug("Saving step data at time step {}.", currentStep);
        m_flushStepData(std::move(stepData));
        if (m_savingInterval.value() == 0) {
          m_savingInterval.reset();
          m_saveAverageStats = false;
          m_saveStreetData = false;
          m_saveTravelData = false;
          m_saveAgentData = false;
          m_saveTurnCounts = false;
        }
      }
      pbar->update();
    }
    auto const [nAdded, nInserted, nArrived, nKilled, nRemaining] =
        m_dynamics->agentStats();
    m_logRunSummary(nAdded, nInserted, nArrived, nKilled, nRemaining);
    if (!m_streetStatusUpdates.empty()) {
      spdlog::warn(
          "{} scheduled street status changes were not made before the end of the run: "
          "they are kept for the next run, whose time steps count from its own start.",
          m_streetStatusUpdates.size());
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
    if (agentInsertionDeltaT <= 0) {
      throw std::invalid_argument(std::format(
          "Agent insertion delta time ({}) must be positive.", agentInsertionDeltaT));
    }
    if (checkDeltaT <= 0) {
      throw std::invalid_argument(
          std::format("Check delta time ({}) must be positive.", checkDeltaT));
    }
    auto const totalTimeSteps = static_cast<std::time_t>(m_endTime - m_initTime);

    m_checkStreetStatusUpdates();
    m_preparePersistence();

    spdlog::info(
        "Starting slow charge run from {} to {} ({} time steps) with agent insertion "
        " check every {} seconds.",
        m_timeToStr(m_initTime),
        m_timeToStr(m_endTime),
        totalTimeSteps,
        checkDeltaT);
    auto pbar = dsf::utility::default_progress_bar("Running simulation", totalTimeSteps);

    std::optional<std::time_t> nextODUpdateTime =
        m_dynamicODsUpdate.empty() ? std::nullopt
                                   : std::make_optional(m_dynamicODsUpdate.front().time);
    if (nextODUpdateTime.has_value() && *nextODUpdateTime != 0) {
      throw std::runtime_error(std::format(
          "First dynamic OD update time must be 0 (initial time). Current value: {}",
          *nextODUpdateTime));
    }
    std::size_t currentAgents{nInitialAgents};
    std::size_t previousAgents{0};
    for (std::time_t currentStep{0}; currentStep < totalTimeSteps; ++currentStep) {
      if (nextODUpdateTime.has_value() && currentStep == *nextODUpdateTime) {
        m_dynamics->importODsFromCSV(m_dynamicODsUpdate.front().file);
        m_dynamicODsUpdate.pop();
        nextODUpdateTime = m_dynamicODsUpdate.empty()
                               ? std::nullopt
                               : std::make_optional(m_dynamicODsUpdate.front().time);
      }
      m_applyStreetStatusUpdates(currentStep);
      if (currentStep > 0 && currentStep % checkDeltaT == 0) {
        if (m_dynamics->nAgents() < previousAgents) {
          currentAgents += agentIncrement;
        }
        previousAgents = m_dynamics->nAgents();
      }
      if (currentStep % agentInsertionDeltaT == 0) {
        try {
          m_dynamics->addAgents(currentAgents, m_agentInsertionMethod);
        } catch (std::overflow_error const& e) {
          // Saturation is what a slow charge looks for: report it and end the run
          spdlog::error(
              "The network is saturated at time step {} ({} agents, capacity {}): "
              "stopping the slow charge. {}",
              currentStep,
              m_dynamics->nAgents(),
              m_dynamics->graph().capacity(),
              e.what());
          break;
        }
      }

      bool const shouldSave =
          m_savingInterval.has_value() &&
          (m_savingInterval.value() == 0 || currentStep % m_savingInterval.value() == 0);
      auto stepData = m_dynamics->evolve(shouldSave ? StepDataRequest{m_saveAverageStats,
                                                                      m_saveStreetData,
                                                                      m_saveTravelData,
                                                                      m_saveAgentData,
                                                                      m_saveTurnCounts}
                                                    : StepDataRequest{});

      if (shouldSave) {
        m_flushStepData(std::move(stepData));
        if (m_savingInterval.value() == 0) {
          m_savingInterval.reset();
          m_saveAverageStats = false;
          m_saveStreetData = false;
          m_saveTravelData = false;
          m_saveAgentData = false;
          m_saveTurnCounts = false;
        }
      }
      pbar->update();
    }
    auto const [nAdded, nInserted, nArrived, nKilled, nRemaining] =
        m_dynamics->agentStats();
    m_logRunSummary(nAdded, nInserted, nArrived, nKilled, nRemaining);
    if (!m_streetStatusUpdates.empty()) {
      spdlog::warn(
          "{} scheduled street status changes were not made before the end of the run: "
          "they are kept for the next run, whose time steps count from its own start.",
          m_streetStatusUpdates.size());
    }
  }

  void TrafficSimulator::connectDataBase(std::string_view const dbPath,
                                         std::string_view const queries) {
    m_database = std::make_unique<SQLite::Database>(
        dbPath, SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
    // Several simulations may connect to a new database together.
    auto const strQueries = std::string(queries);
    auto const start = std::chrono::steady_clock::now();
    while (true) {
      try {
        m_database->exec(strQueries);
        break;
      } catch (SQLite::Exception const& e) {
        auto const busyTimeout = std::chrono::milliseconds{
            m_database->execAndGet("PRAGMA busy_timeout;").getInt()};
        if (e.getErrorCode() != SQLITE_BUSY ||
            std::chrono::steady_clock::now() - start >= busyTimeout) {
          throw;
        }
        spdlog::debug("Database {} is busy, running the connection queries again.",
                      dbPath);
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
      }
    }
    try {
      // Taken at once, so simulations connecting together migrate the database only once
      SQLite::Transaction transaction(*m_database,
                                      SQLite::TransactionBehavior::IMMEDIATE);
      migrateSchema(*m_database);
      transaction.commit();
    } catch (...) {
      m_database.reset();
      throw;
    }
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

  void TrafficSimulator::saveData(std::time_t const savingInterval,
                                  bool const saveAverageStats,
                                  bool const saveStreetData,
                                  bool const saveTravelData,
                                  bool const saveAgentData,
                                  bool const saveTurnCounts) {
    m_savingInterval = savingInterval;
    m_saveAverageStats = saveAverageStats;
    m_saveStreetData = saveStreetData;
    m_saveTravelData = saveTravelData;
    m_saveAgentData = saveAgentData;
    m_saveTurnCounts = saveTurnCounts;
    m_preparePersistence();
    spdlog::info(
        "Data saving configured: interval={}s, avg_stats={}, street_data={}, "
        "travel_data={}, agent_data={}, turn_counts={}",
        savingInterval,
        saveAverageStats,
        saveStreetData,
        saveTravelData,
        saveAgentData,
        saveTurnCounts);
  }

  void TrafficSimulator::m_scheduleStreetStatus(StreetStatusUpdate update) {
    // After the updates already scheduled for the same time, which are made first
    auto const it{std::ranges::upper_bound(
        m_streetStatusUpdates, update.time, {}, &StreetStatusUpdate::time)};
    m_streetStatusUpdates.insert(it, std::move(update));
  }
  void TrafficSimulator::scheduleStreetStatusById(std::time_t const time,
                                                  std::vector<Id> streetIds,
                                                  RoadStatus const status) {
    m_scheduleStreetStatus(StreetStatusUpdate{.time = time,
                                              .streetIds = std::move(streetIds),
                                              .streetName = std::nullopt,
                                              .status = status});
  }
  void TrafficSimulator::scheduleStreetStatusByName(std::time_t const time,
                                                    std::string streetName,
                                                    RoadStatus const status) {
    m_scheduleStreetStatus(StreetStatusUpdate{.time = time,
                                              .streetIds = {},
                                              .streetName = std::move(streetName),
                                              .status = status});
  }
  void TrafficSimulator::m_applyStreetStatusUpdates(std::time_t const currentStep) {
    while (!m_streetStatusUpdates.empty() &&
           m_streetStatusUpdates.front().time <= currentStep) {
      auto const update{std::move(m_streetStatusUpdates.front())};
      m_streetStatusUpdates.pop_front();
      auto& network{m_dynamics->graph()};
      if (update.streetName.has_value()) {
        network.setStreetStatusByName(*update.streetName, update.status);
      } else {
        network.setStreetStatusById(update.streetIds, update.status);
      }
      if (m_database == nullptr) {
        continue;
      }
      auto streetIds{update.streetIds};
      if (update.streetName.has_value()) {
        for (auto const& [streetId, pStreet] : network.edges()) {
          if (pStreet->name().find(*update.streetName) != std::string::npos) {
            streetIds.push_back(streetId);
          }
        }
        std::ranges::sort(streetIds);
      }
      SQLite::Transaction transaction(*m_database);
      SQLite::Statement insertStmt(*m_database,
                                   "INSERT INTO street_status (simulation_id, time_step, "
                                   "street_id, status) VALUES (?, ?, ?, ?)");
      for (auto const streetId : streetIds) {
        insertStmt.bind(1, static_cast<std::int64_t>(m_id));
        insertStmt.bind(2, static_cast<std::int64_t>(m_dynamics->time_step()));
        insertStmt.bind(3, static_cast<std::int64_t>(streetId));
        insertStmt.bind(4, update.status == RoadStatus::CLOSED ? "closed" : "open");
        insertStmt.exec();
        insertStmt.reset();
      }
      transaction.commit();
    }
  }
  void TrafficSimulator::m_checkStreetStatusUpdates() const {
    auto const& network{m_dynamics->graph()};
    for (auto const& update : m_streetStatusUpdates) {
      if (update.streetName.has_value()) {
        if (std::ranges::none_of(network.edges(), [&](auto const& idStreet) {
              return idStreet.second->name().find(*update.streetName) !=
                     std::string::npos;
            })) {
          spdlog::warn(
              "No street name contains \"{}\": the status change scheduled at time {} "
              "will change nothing.",
              *update.streetName,
              update.time);
        }
        continue;
      }
      for (auto const streetId : update.streetIds) {
        if (!network.edges().contains(streetId)) {
          throw std::out_of_range(
              std::format("Street with id {} of the status change scheduled at time {} "
                          "not found",
                          streetId,
                          update.time));
        }
      }
    }
  }
  void TrafficSimulator::setName(std::string_view const name) {
    m_name = std::string(name);
    m_safeName = std::string(name);
    std::ranges::replace(m_safeName, ' ', '_');
  }
  void TrafficSimulator::setId(Id const id) {
    if (!std::in_range<std::int64_t>(id)) {
      throw std::invalid_argument(std::format(
          "The simulation id ({}) does not fit in a 64-bit signed integer.", id));
    }
    m_id = id;
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

  void TrafficSimulator::m_dumpSimInfoSQL(std::int64_t const networkId) const {
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT OR REPLACE INTO simulation_info (id, name, init_time, end_time, "
        "agent_insertion_method, network_id, dsf_version) VALUES (?, ?, ?, ?, ?, ?, ?)");
    insertStmt.bind(1, static_cast<std::int64_t>(m_id));
    insertStmt.bind(2, m_name);
    insertStmt.bind(3, strInitTime());
    insertStmt.bind(4, strEndTime());
    std::string insertionMethodStr;
    switch (m_agentInsertionMethod) {
      case AgentInsertionMethod::RANDOM:
        insertionMethodStr = "random";
        break;
      case AgentInsertionMethod::RANDOM_WEIGHTED_ORIGIN:
        insertionMethodStr = "random_weighted_origin";
        break;
      case AgentInsertionMethod::ODS:
        insertionMethodStr = "ods";
        break;
      case AgentInsertionMethod::RANDOM_ODS:
        insertionMethodStr = "random_ods";
        break;
      case AgentInsertionMethod::CONDITIONAL_RANDOM_ODS:
        insertionMethodStr = "conditional_random_ods";
        break;
      case AgentInsertionMethod::UNIFORM:
        insertionMethodStr = "uniform";
        break;
      default:
        insertionMethodStr = "unknown";
    }
    insertStmt.bind(5, insertionMethodStr);
    insertStmt.bind(6, networkId);
    insertStmt.bind(7, std::string(DSF_VERSION));
    insertStmt.exec();
  }

  void TrafficSimulator::m_saveStreetDataSQL(
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const {
    if (streetDataRecords.empty()) {
      spdlog::debug("No street data records to save for time step {}.", time_step);
      return;
    }
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO road_data (time_step, simulation_id, street_id, density_vpk, "
        "avg_speed_kph, std_speed_kph, n_observations, counts, queue_length) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)");

    for (auto const& [streetId, record] : streetDataRecords) {
      // The coil name is edges.coilcode (see road_data_view)
      insertStmt.bind(1, time_step);
      insertStmt.bind(2, simulation_id);
      insertStmt.bind(3, static_cast<std::int64_t>(streetId));
      insertStmt.bind(4, record.density);
      if (record.avgSpeed.has_value()) {
        insertStmt.bind(5, record.avgSpeed.value());
        insertStmt.bind(6, record.stdSpeed.value());
      } else {
        insertStmt.bind(5);
        insertStmt.bind(6);
      }
      insertStmt.bind(7, static_cast<std::int64_t>(record.nObservations.value_or(0)));
      if (record.counts.has_value()) {
        insertStmt.bind(8, static_cast<std::int64_t>(record.counts.value()));
      } else {
        insertStmt.bind(8);
      }
      insertStmt.bind(9, static_cast<std::int64_t>(record.queueLength));
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
      auto stream{utility::openCSVFile(filename)};
      utility::CSVRowWriter writer{stream};
      if (!fileExists) {
        spdlog::info("Creating new CSV file for street data: {}", filename);
        writer.write_row("datetime",
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
            record.stdSpeed.has_value() ? std::format("{}", record.stdSpeed.value()) : "";
        writer.write_row(
            datetime,
            time_step,
            streetId,
            record.coilName.has_value() ? record.coilName.value() : "",
            std::format("{}", record.density),
            avgSpeed,
            stdSpeed,
            record.nObservations.value_or(0),
            record.counts.has_value() ? std::to_string(record.counts.value()) : "",
            std::format("{}", record.queueLength));
      }
      spdlog::debug(
          "Saved street data for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write street data CSV: {}", e.what());
    }
  }

  void TrafficSimulator::m_saveAvgStatsSQL(const std::int64_t time_step,
                                           const std::int64_t simulation_id,
                                           const AverageStatsRecord& averageStats) const {
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO avg_stats ("
        "simulation_id, time_step, n_ghost_agents, n_agents, "
        "mean_speed_kph, std_speed_kph, mean_density_vpk, std_density_vpk, "
        "mean_travel_time_s, mean_queue_length) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
    insertStmt.bind(1, simulation_id);
    insertStmt.bind(2, time_step);
    insertStmt.bind(3, static_cast<std::int64_t>(m_dynamics->ghostAgents()));
    insertStmt.bind(4, static_cast<std::int64_t>(m_dynamics->nAgents()));

    if (averageStats.nValidEdges > 0) {
      insertStmt.bind(5, averageStats.meanSpeed);
      insertStmt.bind(6, averageStats.stdSpeed);
      insertStmt.bind(9, averageStats.meanTravelTime);
    } else {
      insertStmt.bind(5);
      insertStmt.bind(6);
      insertStmt.bind(9);
    }
    insertStmt.bind(7, averageStats.meanDensity);
    insertStmt.bind(8, averageStats.stdDensity);
    insertStmt.bind(10, averageStats.meanQueueLength);
    insertStmt.exec();
  }

  void TrafficSimulator::m_saveAvgStatsCSV(const std::string& datetime,
                                           const std::int64_t time_step,
                                           const AverageStatsRecord& averageStats) const {
    auto const filename{m_generateCSVfilename("avg_stats")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      auto stream{utility::openCSVFile(filename)};
      utility::CSVRowWriter writer{stream};
      if (!fileExists) {
        spdlog::info("Creating new CSV file for average stats: {}", filename);
        writer.write_row("datetime",
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
      writer.write_row(datetime,
                       time_step,
                       m_dynamics->ghostAgents(),
                       m_dynamics->nAgents(),
                       meanSpeed,
                       stdSpeed,
                       std::format("{}", averageStats.meanDensity),
                       std::format("{}", averageStats.stdDensity),
                       meanTravelTime,
                       std::format("{}", averageStats.meanQueueLength));
      spdlog::debug(
          "Saved average stats for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write average stats CSV: {}", e.what());
    }
  }

  void TrafficSimulator::m_saveTravelDataSQL(
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_vector<TravelRecord> travelDTs) const {
    if (travelDTs.empty()) {
      spdlog::debug("No travel data records to save for time step {}.", time_step);
      return;
    }
    SQLite::Statement insertStmt(*this->database(),
                                 "INSERT INTO travel_data (time_step, simulation_id, "
                                 "distance_m, travel_time_s) VALUES (?, ?, ?, ?)");

    for (auto const& [distance, time] : travelDTs) {
      insertStmt.bind(1, time_step);
      insertStmt.bind(2, simulation_id);
      insertStmt.bind(3, distance);
      insertStmt.bind(4, time);
      insertStmt.exec();
      insertStmt.reset();
    }
  }

  void TrafficSimulator::m_saveTravelDataCSV(
      const std::string& datetime,
      const std::int64_t time_step,
      tbb::concurrent_vector<TravelRecord> travelDTs) const {
    if (travelDTs.empty()) {
      spdlog::debug("No travel data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{m_generateCSVfilename("travel_data")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      auto stream{utility::openCSVFile(filename)};
      utility::CSVRowWriter writer{stream};
      if (!fileExists) {
        spdlog::info("Creating new CSV file for travel data: {}", filename);
        writer.write_row("datetime", "time_step", "distance_m", "travel_time_s");
      }
      for (auto const& [distance, time] : travelDTs) {
        writer.write_row(
            datetime, time_step, std::format("{}", distance), std::format("{}", time));
      }
      spdlog::debug(
          "Saved travel data for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write travel data CSV: {}", e.what());
    }
  }

  void TrafficSimulator::m_saveAgentDataSQL(
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_unordered_map<Id, std::vector<AgentStreetRecord>> agentDataRecords)
      const {
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
      tbb::concurrent_unordered_map<Id, std::vector<AgentStreetRecord>> agentDataRecords)
      const {
    if (agentDataRecords.empty()) {
      spdlog::debug("No agent data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{m_generateCSVfilename("agent_data")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      auto stream{utility::openCSVFile(filename)};
      utility::CSVRowWriter writer{stream};
      if (!fileExists) {
        spdlog::info("Creating new CSV file for agent data: {}", filename);
        writer.write_row("agent_id", "edge_id", "time_step_in", "time_step_out");
      }
      for (auto const& [edge_id, data] : agentDataRecords) {
        for (auto const& [agent_id, ts_in, ts_out] : data) {
          writer.write_row(agent_id, edge_id, ts_in, ts_out);
        }
      }
      spdlog::debug("Saved agent data for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write agent data CSV: {}", e.what());
    }
  }
  void TrafficSimulator::m_saveTurnCountsSQL(const std::int64_t time_step,
                                             const std::int64_t simulation_id,
                                             TurnCountsDict turnCountsRecords) const {
    if (turnCountsRecords.empty()) {
      spdlog::debug("No turn counts records to save for time step {}.", time_step);
      return;
    }
    spdlog::trace("Saving {} turn counts records for time step {}.",
                  turnCountsRecords.size(),
                  time_step);
    SQLite::Statement insertStmt(*this->database(),
                                 "INSERT INTO turn_counts (time_step, simulation_id, "
                                 "source_edge_id, target_edge_id, counts) "
                                 "VALUES (?, ?, ?, ?, ?)");

    for (auto const& [edgeId, turnCounts] : turnCountsRecords) {
      for (auto const& [nextEdgeId, count] : turnCounts) {
        if (count == 0) {
          continue;  // Skip inserting records with zero counts to save memory
        }
        insertStmt.bind(1, time_step);
        insertStmt.bind(2, simulation_id);
        insertStmt.bind(3, static_cast<std::int64_t>(edgeId));
        insertStmt.bind(4, static_cast<std::int64_t>(nextEdgeId));
        insertStmt.bind(5, static_cast<std::int64_t>(count));
        insertStmt.exec();
        insertStmt.reset();
      }
    }
  }
  void TrafficSimulator::m_saveTurnCountsCSV(const std::string& datetime,
                                             const std::int64_t time_step,
                                             TurnCountsDict turnCountsRecords) const {
    if (turnCountsRecords.empty()) {
      spdlog::debug("No turn counts records to save for time step {}.", time_step);
      return;
    }
    auto const filename{m_generateCSVfilename("turn_counts")};
    bool fileExists = std::filesystem::exists(filename);

    try {
      auto stream{utility::openCSVFile(filename)};
      utility::CSVRowWriter writer{stream};
      if (!fileExists) {
        spdlog::info("Creating new turn counts CSV file: {}", filename);
        writer.write_row(
            "datetime", "time_step", "source_edge_id", "target_edge_id", "counts");
      }
      for (auto const& [edgeId, turnCounts] : turnCountsRecords) {
        for (auto const& [nextEdgeId, count] : turnCounts) {
          if (count == 0) {
            continue;  // Match the SQL writer, which also skips empty turns.
          }
          writer.write_row(datetime, time_step, edgeId, nextEdgeId, count);
        }
      }
      spdlog::debug(
          "Saved turn counts for time step {} to file {}.", time_step, filename);
    } catch (const std::exception& e) {
      spdlog::error("Failed to write turn counts CSV: {}", e.what());
    }
  }

  std::int64_t TrafficSimulator::networkId() const {
    if (m_dynamics == nullptr) {
      throw std::runtime_error("No road network imported: it has no id.");
    }
    return networkFingerprint(networkRows(m_dynamics->graph()));
  }

  void TrafficSimulator::m_dumpNetwork(std::int64_t const networkId) const {
    auto& db{*this->database()};
    SQLite::Statement stored(
        db,
        "SELECT EXISTS (SELECT 1 FROM edges WHERE network_id = ?) OR "
        "EXISTS (SELECT 1 FROM nodes WHERE network_id = ?)");
    stored.bind(1, networkId);
    stored.bind(2, networkId);
    if (stored.executeStep() && stored.getColumn(0).getInt() != 0) {
      spdlog::debug("Network {} is already in the database.", networkId);
      return;
    }
    auto const rows{networkRows(m_dynamics->graph())};
    SQLite::Statement insertEdgeStmt(
        db,
        "INSERT INTO edges (network_id, id, source, target, length, maxspeed, name, "
        "nlanes, coilcode, geometry) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
    for (auto const& edge : rows.edges) {
      insertEdgeStmt.bind(1, networkId);
      insertEdgeStmt.bind(2, edge.id);
      insertEdgeStmt.bind(3, edge.source);
      insertEdgeStmt.bind(4, edge.target);
      insertEdgeStmt.bind(5, edge.length);
      insertEdgeStmt.bind(6, edge.maxSpeed);
      insertEdgeStmt.bind(7, edge.name);
      insertEdgeStmt.bind(8, edge.nLanes);
      if (edge.coilCode.has_value()) {
        insertEdgeStmt.bind(9, *edge.coilCode);
      } else {
        insertEdgeStmt.bind(9);
      }
      insertEdgeStmt.bind(10, edge.geometry);
      insertEdgeStmt.exec();
      insertEdgeStmt.reset();
    }
    SQLite::Statement insertNodeStmt(
        db, "INSERT INTO nodes (network_id, id, type, geometry) VALUES (?, ?, ?, ?)");
    for (auto const& node : rows.nodes) {
      insertNodeStmt.bind(1, networkId);
      insertNodeStmt.bind(2, node.id);
      if (node.type.has_value()) {
        insertNodeStmt.bind(3, *node.type);
      } else {
        insertNodeStmt.bind(3);
      }
      if (node.geometry.has_value()) {
        insertNodeStmt.bind(4, *node.geometry);
      } else {
        insertNodeStmt.bind(4);
      }
      insertNodeStmt.exec();
      insertNodeStmt.reset();
    }
    spdlog::info("Stored network {} ({} edges, {} nodes) in the database.",
                 networkId,
                 rows.edges.size(),
                 rows.nodes.size());
  }

  void TrafficSimulator::m_preparePersistence() {
    if (m_dynamics == nullptr) {
      return;
    }
    if (m_saveAgentData) {
      Street::acquireAgentData();
    }
    if (m_saveTurnCounts && m_dynamics->turnCounts().empty()) {
      m_dynamics->initTurnCounts();
    }
    if (m_database == nullptr) {
      return;
    }
    // One write transaction for the whole setup, taken at once (IMMEDIATE): several simulations
    // may start together on the same database, and each of them must see the tables either
    // missing or complete. It also keeps the write lock for a single short commit instead of
    // one per network row, which made the other simulations wait past the busy timeout
    SQLite::Transaction transaction(*m_database, SQLite::TransactionBehavior::IMMEDIATE);
    // The views of the data tables read these
    createTable(*m_database, kSimulationInfo);
    createTable(*m_database, kEdges);
    createTable(*m_database, kNodes);
    auto const currentNetworkId{networkId()};
    m_dumpSimInfoSQL(currentNetworkId);
    m_dumpNetwork(currentNetworkId);
    if (m_saveStreetData) {
      createTable(*m_database, kRoadData);
    }
    if (m_saveAverageStats) {
      createTable(*m_database, kAvgStats);
    }
    if (m_saveTravelData) {
      createTable(*m_database, kTravelData);
    }
    if (m_saveAgentData) {
      createTable(*m_database, kAgentData);
    }
    if (m_saveTurnCounts) {
      createTable(*m_database, kTurnCounts);
    }
    if (!m_streetStatusUpdates.empty()) {
      createTable(*m_database, kStreetStatus);
    }
    transaction.commit();
  }

  void TrafficSimulator::m_flushStepData(StepDataResult stepData) {
    if (!stepData.streetData.has_value() && !stepData.travelData.has_value() &&
        !stepData.averageStats.has_value() && !stepData.agentData.has_value() &&
        stepData.turnCounts.empty()) {
      return;
    }

    auto const timeStep = static_cast<std::int64_t>(stepData.timeStep);

    if (m_database != nullptr) {
      auto const simulationId = static_cast<std::int64_t>(m_id);
      SQLite::Transaction transaction(*m_database);
      if (stepData.streetData.has_value()) {
        m_saveStreetDataSQL(timeStep, simulationId, std::move(*stepData.streetData));
      }
      if (stepData.travelData.has_value()) {
        m_saveTravelDataSQL(timeStep, simulationId, std::move(*stepData.travelData));
      }
      if (stepData.agentData.has_value()) {
        m_saveAgentDataSQL(timeStep, simulationId, std::move(*stepData.agentData));
      }
      if (stepData.averageStats.has_value()) {
        m_saveAvgStatsSQL(timeStep, simulationId, *stepData.averageStats);
      }
      if (!stepData.turnCounts.empty()) {
        m_saveTurnCountsSQL(timeStep, simulationId, std::move(stepData.turnCounts));
      }
      transaction.commit();
      return;
    }

    auto const datetime = m_timeToStr(m_initTime + stepData.timeStep);

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
    if (!stepData.turnCounts.empty()) {
      m_saveTurnCountsCSV(datetime, timeStep, std::move(stepData.turnCounts));
    }
  }
}  // namespace dsf::mobility
