# Data formats

## Edges file — `RoadNetwork::importEdges(path, separator)`

Format deduced from the extension via `fileExtMap`: `.csv`, `.json`, `.geojson`.
For CSV the separator is passed as the second argument and **defaults to `;`** — this repo's
files all use `;`, not `,`.
Implementation: `m_csvEdgesImporter` / `m_jsonEdgesImporter` in
[RoadNetwork.cpp](../src/dsf/mobility/RoadNetwork.cpp).

Required columns/fields:

| Column | Meaning |
| --- | --- |
| `id` | Street id |
| `source`, `target` | Node ids; the edge is directed `source -> target`. Self-loops are skipped with a warning. |
| `length` | Metres |
| `maxspeed` | km/h (converted internally to m/s; invalid values fall back to 30 km/h) |
| `name` | Street name |
| `type` | OSM-style class; matched by substring against `motorway`, `primary`, `secondary`, `tertiary`, `residential` to derive priority |

Optional columns — absent ones are simply not read, no error:

| Column | Meaning |
| --- | --- |
| `geometry` | WKT `LINESTRING (x y, x y, ...)`. Without it a warning is logged and streets get no geometry, which disables angle-based logic. |
| `nlanes` | Lane count (default 1) |
| `capacity` | Vehicles; default is derived from length / mean vehicle length |
| `priority` | Boolean; overrides the type-derived priority |
| `status` | `open` or `closed` (`RoadStatus`) |
| `coilcode` | Enables a `Counter` sensor on the street. Values `""`, `"null"`, `"nan"` are ignored. |
| `forbidden_turns` | `"srcId1-tgtId1,srcId2-tgtId2,..."` — turns this street may not feed into |
| `mobility_class` | `uint8_t` |
| `lane_mapping` | Per-lane `Direction`, parsed by substring: `left`, `right`, `straight`, `left;straight`, `right;straight` |

**Any other column becomes an edge attribute**, with type inferred as bool (`"true"`/`"false"`),
int64, double, string, or null. Read them back with `edge.getAttribute<T>("name")`.

## Node properties file — `RoadNetwork::importNodeProperties(path, separator)`

CSV only. Columns: `id`, `type`, `geometry`.

- `type` containing `traffic_signals` → the node is converted to a `TrafficLight`;
  containing `roundabout` → `Roundabout`. Anything else (including `N/A`) stays a plain
  `RoadJunction`/`Intersection`.
- `geometry` is WKT `POINT (lon lat)`.

Coordinates are (x, y) = (longitude, latitude); `geometry::haversine_km` assumes that order.

Working examples: [test/data/manhattan_edges.csv](../test/data/manhattan_edges.csv) and
[manhattan_nodes.csv](../test/data/manhattan_nodes.csv) — these are the fixtures the
Python binding tests build on.

## Legacy traffic-light CSV — `importTrafficLights(path)`

Separator `;`, columns in order: `id`, `sourceId`, `cycleTime`, `greenTime`. Each node's
rows are folded into two phases — streets sharing the node's first `greenTime` go to phase 0
(duration = that green time), the rest to phase 1 (duration = `cycleTime - firstGreenTime`).
All streets are added with `Direction::ANY`. For direction-level control, configure phases
programmatically via `TrafficLight::setPhases()` / `addPhase()`.

## Simulation config JSON — `TrafficSimulator::importConfig(path)`

Parsed with simdjson in [TrafficSimulator.cpp](../src/dsf/mobility/TrafficSimulator.cpp).
Sample: [examples/sample_sim_config.json](../examples/sample_sim_config.json) (note that the
sample is slightly out of date — trust the code, which is summarised below).

```jsonc
{
  "general": {
    "name": "sample_sim",              // REQUIRED
    "input_folder": "./examples/",     // REQUIRED, prefixed to road_network file paths
    "output_folder": "./output/",      // REQUIRED, created if missing
    "output_basename": "run1",         // optional, appended to output_folder as the prefix
    "database": "sample_sim.db",       // optional; without it, results go to CSV
    "init_time": "20250101 080000",    // optional; uint64 epoch, or "YYYYMMDD" / "YYYYMMDD hhmmss"
    "end_time":  "20250101 200000",    // optional, same forms
    "update_paths": { "interval": 300, "throw_on_empty": false },
    "save_data":    { "interval": 300, "avg": true, "road": true,
                      "travel": false, "agent": false, "turn_counts": false },
    "dynamic_ods": [                   // optional; time-phased OD swaps
      { "time": 0,    "file": "ods_phase1.csv" },
      { "time": 3600, "file": "ods_phase2.csv" }
    ]
  },
  "road_network": {                    // REQUIRED section
    "edges_file": "edges.csv",                    // REQUIRED
    "node_properties_file": "node_props.csv",     // REQUIRED
    "set_edge_weight": { "weight": "traveltime",  // REQUIRED; "traveltime" | "length" | attribute name
                         "threshold": 0.05 }      // REQUIRED
  },
  "dynamics": {                        // REQUIRED section
    "agent_insertion_method": "RANDOM",  // REQUIRED: RANDOM | UNIFORM | ODS | RANDOM_ODS | CONDITIONAL_RANDOM_ODS
    "seed": 42,                          // optional
    "max_concurrency": 8,                // optional
    "error_probability": 0.05,           // optional
    "kill_stagnant_agents": 40.0,        // optional (time tolerance factor)
    "mean_travel_distance": 5000.0,      // optional
    "mean_travel_time": 900,             // optional
    "importODsFromCSV": { "file": "ods.csv", "edges": false, "separator": ";" }  // optional
  }
}
```

Fields marked REQUIRED throw `std::runtime_error` naming the missing
`section.field` when absent. If both `dynamic_ods` and `importODsFromCSV` are present,
`dynamic_ods` wins and a warning is logged.

OD CSV fixtures live in [test/data/](../test/data/) (`ods_od_pairs.csv`,
`ods_random_ods.csv`, `ods_conditional_random_ods.csv`, `ods_phase1.csv`, ...) alongside
`dynamic_ods_*.json` configs covering both valid and malformed cases.

## Outputs

With a database connected, `TrafficSimulator` writes to SQLite (SQLiteCpp). Without one,
the same tables go to CSV named `<simId>_<safeName>_<table>.csv`, or
`<outputPrefix><table>.csv` when an output prefix is set. `safeName` is the simulation
name with spaces replaced by underscores; `simId` is the start time as `YYYYMMDDHHMMSS`.

| Table / CSV | Contents |
| --- | --- |
| `simulation_info` | One row per run: id, name, parameters |
| `nodes`, `edges` | Network snapshot |
| `road_data` | Per street per saving interval: `density_vpk`, `avg_speed_kph`, `std_speed_kph`, `n_observations`, `coil`, `counts`, `queue_length` |
| `avg_stats` | Network-wide per interval: `n_agents`, `n_ghost_agents`, `mean_speed_kph`, `std_speed_kph`, `mean_density_vpk`, `std_density_vpk` |
| `travel_data` | One row per completed trip: `distance_m`, `travel_time_s` |
| `agent_data` | Per agent per edge: `agent_id`, `edge_id`, `time_step_in`, `time_step_out` |
| `turn_counts` | Turn-by-turn movement counts |

Every row carries `simulation_id`, `datetime` and `time_step`. Inserts are batched.
Default connection pragmas: `busy_timeout=5000`, `journal_mode=WAL`, `synchronous=NORMAL`,
`temp_store=MEMORY`, `cache_size=-20000`.

Note that `examples/*.csv` in the working tree are old generated outputs — they are
git-ignored, not fixtures.
