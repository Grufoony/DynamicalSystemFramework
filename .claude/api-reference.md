# API cheat sheet

Signatures are abridged; open the header for defaults and doc comments. File links point
at the declaration.

## Shared vocabulary — [utility/Typedef.hpp](../src/dsf/utility/Typedef.hpp)

```cpp
using Id    = uint64_t;     // node, edge, agent, itinerary ids all share this
using Delay = uint16_t;     // traffic-light durations, in ticks

enum class SpeedFunction  { CUSTOM, LINEAR };
enum Direction            { RIGHT, RIGHTANDSTRAIGHT, STRAIGHT, ANY, LEFTANDSTRAIGHT, LEFT, UTURN };
enum class TrafficLightOptimization { SINGLE_TAIL, DOUBLE_TAIL };
enum class FileExt        { CSV, JSON, GEOJSON };   // + fileExtMap for extension lookup
enum train_t              { BUS, SFM, R, RV, IC, FRECCIA, FRECCIAROSSA, ES };
#define DSF_EXECUTION     // std::execution::par_unseq,  (empty on Apple)
```

`Direction` is ordered so that the numeric value encodes the turn: `RIGHT` = 0
(angle delta < 0) through `LEFT` = 5 (delta > 0), with `UTURN` = 6 for |delta| > π.
`Edge::deltaAngle(previousEdgeAngle)` produces the delta these are derived from.

Most classes have a `std::formatter` specialisation, so `std::format("{}", street)` works
for `Edge`, `Street`, `Intersection`, `TrafficLight`, `TrafficLightPhase`, `RoadJunction`,
`RoadStatus`, `Direction`, `Point`, `PolyLine` and `PathCollection`.

## `dsf::Network<node_t, edge_t>` — [base/Network.hpp](../src/dsf/base/Network.hpp)

Header-only template. Construction and lookup:

```cpp
template <typename TNode = node_t, typename... TArgs> void addNode(TArgs&&...);
template <typename TEdge = edge_t, typename... TArgs> void addEdge(TArgs&&...);
void addNDefaultNodes(std::size_t n);

node_t&  node(Id);            edge_t&  edge(Id);          edge_t& edge(Id src, Id tgt);
template <typename TNode> TNode& node(Id);   // dynamic_cast downcast
template <typename TEdge> TEdge& edge(Id);
auto const& nodes() const;    auto const& edges() const;  // unordered_map<Id, unique_ptr<...>>
auto nNodes() const;          auto nEdges() const;
```

Algorithms:

```cpp
PathCollection allPathsTo(Id targetId) const;               // node-space, all shortest next hops
virtual PathCollection allEdgePathsTo(Id targetEdgeId) const;  // dual graph (edge-space)
PathCollection shortestPath(Id sourceId, Id targetId) const;

void computeBetweennessCentralities();          // Brandes, weighted, on nodes
void computeEdgeBetweennessCentralities();      // Brandes, weighted, on edges
void computeEdgeKBetweennessCentralities(std::size_t K);  // Yen's K-shortest paths, TBB-parallel

virtual void setEdgeWeight(std::string_view weight, std::optional<double> threshold) = 0;
```

The returned `PathCollection` (`unordered_map<Id, vector<Id>>` of node/edge → next hops)
is guaranteed acyclic: only hops that strictly decrease the precomputed distance-to-target
are kept, which is what lets `PathCollection::explode(src, tgt)` enumerate paths without
looping.

## `dsf::mobility::RoadNetwork` — [mobility/RoadNetwork.hpp](../src/dsf/mobility/RoadNetwork.hpp)

`Network<RoadJunction, Street>`, move-only.

```cpp
// Import — format deduced from the extension (csv / json / geojson)
template <typename... TArgs> void importEdges(std::string const& file, TArgs&&...);   // e.g. ';' separator
template <typename... TArgs> void importNodeProperties(std::string const& file, TArgs&&...);
void importTrafficLights(std::string const& csvFile);      // legacy 4-column format

// Node type conversion (in place, on an existing node)
TrafficLight& makeTrafficLight(Id);
Roundabout&   makeRoundabout(Id);
Station&      makeStation(Id, unsigned int managementTime);
void          addCoil(Id streetId, std::string const& name = {});

// Network preparation
void adjustNodeCapacities();
void autoInitTrafficLights(double mainRoadPercentage = 0.6, /* defaultCycleDuration = 90 */);
void autoMapStreetLanes();
void autoAssignRoadPriorities();
void setEdgeWeight(std::string_view weight, std::optional<double> threshold);  // "traveltime" | "length" | attribute name

// Mutation during a run (road works, closures)
void setStreetStatusById(Id, RoadStatus);        void setStreetStatusByName(std::string const&, RoadStatus);
void changeStreetNLanesById(Id, int, std::optional<double> speedFactor);   // + ByName variant
void changeStreetCapacityById(Id, double factor);                          // + ByName variant

// Introspection / export
std::size_t nIntersections() const, nRoundabouts() const, nTrafficLights() const, nCoils() const;
auto capacity() const;                       // max simultaneous agents
Street const* street(Id source, Id destination) const;    // nullptr if absent
void describe(std::ostream& = std::cout) const;
void exportCSV(std::string_view folder) const;
void exportTrafficLights(std::string_view file) const;
```

`autoInitTrafficLights` only touches `TrafficLight` nodes with no phases yet; nodes with
fewer than 3 ingoing edges are downgraded back to plain `Intersection`. Priority streets
are detected by name, then speed limit, then lane count, then angle.

## `dsf::mobility::FirstOrderDynamics` — [mobility/FirstOrderDynamics.hpp](../src/dsf/mobility/FirstOrderDynamics.hpp)

The engine. Exposed to Python as `dsf.mobility.Dynamics`.

```cpp
FirstOrderDynamics(RoadNetwork&& graph, /* seed, ... */);
void prepareNetwork(bool adjustNodeCapacities = true, ...);

// Demand
void setOrigins(std::unordered_map<Id, double> const&);
void setDestinations(std::unordered_map<Id, double> const&);   // + initializer_list / container overloads
void setODs(std::vector<std::tuple<Id, Id, double>> const&);
void setConditionalODs(...);
void importODsFromCSV(std::string_view file, ...);
void addAgent(...); void addAgents(std::size_t n, AgentInsertionMethod);
void addAgentsUniformly(std::size_t n, ...);
void addItinerary(...);

// Tuning
void setSeed(unsigned);              void setConcurrency(std::size_t);     // from Dynamics
void setSpeedFunction(SpeedFunction, TArgs&&...);
void setErrorProbability(double);    void setPassageProbability(double);
void setUTurnPenaltyFactor(double);  void setForcePriorities(bool);
void setReinsertAgents(bool);        void setDataUpdatePeriod(Delay);
void setMeanTravelDistance(double);  void setMeanTravelTime(std::time_t);
void setUpdatePathsThrowOnEmpty(bool);
void killStagnantAgents(double timeToleranceFactor = 3.);

// Run
void updatePaths();
StepDataResult evolve(StepDataRequest const& = {});
void optimizeTrafficLights(/* TrafficLightOptimization, ... */);

// Metrics
Measurement<double> meanTravelTime(bool clearData = false);   // + meanTravelDistance, meanTravelSpeed
Measurement<double> streetMeanDensity() const;
Measurement<double> streetMeanFlow() const;                   // + (threshold, above) overload
auto nAgents() const;  auto ghostAgents() const;  auto agentStats() const;
TurnCountsDict const& turnCounts() const;  auto normalizedTurnCounts() const;
void initTurnCounts();  void resetTurnCounts();
void summary(std::ostream& = std::cout) const;
```

`StepDataRequest` is a plain struct of `bool save{AverageStats,StreetData,TravelData,AgentData,TurnCounts}`.
`StepDataResult` returns `timeStep` plus optional TBB concurrent containers:
`streetData` (`concurrent_map<Id, StreetDataRecord>`), `travelData`
(`concurrent_vector<pair<double,double>>` = distance, time), `averageStats`
(`AverageStatsRecord`) and agent data. Ask for nothing and `evolve()` skips the stats work.

## `dsf::mobility::TrafficSimulator` — [mobility/TrafficSimulator.hpp](../src/dsf/mobility/TrafficSimulator.hpp)

Config-driven wrapper: owns a `FirstOrderDynamics` and an optional `SQLite::Database`.

```cpp
TrafficSimulator();
explicit TrafficSimulator(std::string_view jsonConfigFile);   // = default ctor + importConfig
void importConfig(std::string_view jsonConfigFile);

void importRoadNetwork(std::string_view edgesFile, std::string_view nodePropertiesFile = {});
void connectDataBase(std::string const& dbPath, /* pragmas */);
void setName(std::string_view);        void setOutputPrefix(std::string_view);
void setTimeFrame(std::time_t initTime, std::optional<std::time_t> endTime = {});
void setAgentInsertionMethod(AgentInsertionMethod);
void updatePaths(std::time_t deltaT = 0, bool throwOnEmpty = true);
void saveData(std::time_t savingInterval, /* which tables */);

void run(std::vector<std::size_t> const& nAgentsPerTimeStep, ...);
void run(std::size_t nInitialAgents, ...);

auto* dynamics();  auto* database();  auto id() const;  auto const& name() const;
auto initTime() const;  auto strInitTime() const;   // "YYYY-MM-DD HH:MM:SS"
```

If no database is connected, the same tables are written as CSV files named
`<id>_<safeName>_<table>.csv` (or `<outputPrefix><table>.csv` when a prefix is set).
The simulation `id` is the start timestamp formatted `YYYYMMDDHHMMSS`.

## Supporting types

- **`Agent`** — [mobility/Agent.hpp](../src/dsf/mobility/Agent.hpp). Owns a
  `vector<shared_ptr<Itinerary>>` trip plus an index, current/next street, speed,
  travelled distance, spawn and free times, optional max distance/time for stochastic
  agents. `setSpawnTime()` is re-stamped on actual insertion so travel time measures time
  *in* the network, not time spent queued for a free slot.
- **`Itinerary`** — id, destination node, and a `PathCollection` of next hops
  (`setPath`, `path`, `save`/`load` to a binary file). Move-only.
- **`Measurement<T>`** — [utility/Measurement.hpp](../src/dsf/utility/Measurement.hpp).
  `{mean, std, n, is_valid}`; constructible from any container. Check `is_valid` before
  reading — an empty sample yields `is_valid == false`, not a NaN.
- **`Counter`** — [mobility/Sensors.hpp](../src/dsf/mobility/Sensors.hpp). Name + count,
  `operator++`, `reset()`. This is the "coil" attached to a `Street`.
- **`geometry::Point`** — immutable `(x, y)`, structured-binding enabled, parses WKT
  `POINT (...)`; free function `haversine_km(p1, p2)` treats coordinates as (lon, lat).
  **`geometry::PolyLine`** — `std::vector<Point>` subclass that parses WKT `LINESTRING (...)`.
- **`dsf::queue` / `dsf::priority_queue`** — [utility/queue.hpp](../src/dsf/utility/queue.hpp).
  Thin subclasses of the std adaptors exposing the underlying container (needed to iterate
  street queues without draining them).
- **Type traits** — `is_node`, `is_street`, `is_numeric` in
  [utility/TypeTraits/](../src/dsf/utility/TypeTraits/). `is_node`/`is_street` are
  hand-maintained explicit specialisation lists: **adding a new node or edge subclass means
  adding four specialisations** (`T`, `const T`, `const T&`, `unique_ptr<T>`).

## Python surface — [bindings.cpp](../src/dsf/bindings.cpp)

Module `dsf_cpp`, re-exported by [src/dsf/__init__.py](../src/dsf/__init__.py) as `dsf`.

```
dsf.__version__
dsf.logging                   # log level control
dsf.mobility.{Street, RoadJunction, Intersection, TrafficLight, TrafficLightPhase,
              RoadNetwork, Itinerary, PathCollection, Dynamics, TrafficSimulator}
dsf.mdt.TrajectoryCollection  # .to_pandas() / .to_polars() when those are installed
dsf.Measurement
dsf.{get_cartography, graph_from_gdfs, graph_to_gdfs,
     create_manhattan_cartography, to_folium_map}   # pure Python, cartography module
```

Note the rename: C++ `FirstOrderDynamics` → Python `mobility.Dynamics`. `mobility` and
`mdt` are also registered in `sys.modules` so `from dsf.mobility import RoadNetwork` works.

`cartography.py` wraps osmnx: `get_cartography()` fetches and processes an OSM area into
GeoDataFrames (normalising `maxspeed`, `nlanes`, bearings and turn headings),
`graph_to_gdfs`/`graph_from_gdfs` convert to and from networkx,
`create_manhattan_cartography()` generates a synthetic grid, `to_folium_map()` renders.
