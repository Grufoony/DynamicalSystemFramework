# Architecture

## Directory map

```
src/dsf/
  dsf.hpp             umbrella header; version constants; dsf::version(), dsf::log_to_file()
  bindings.cpp        nanobind module `dsf_cpp` (~1900 lines) -> submodules mobility, mdt, logging
  __init__.py         Python package `dsf`: re-exports dsf_cpp submodules + cartography
  base/               generic, domain-agnostic graph + dynamics layer
  mobility/           traffic domain: roads, junctions, agents, the simulation engine
  geometry/           Point, PolyLine, haversine, WKT parsing
  mdt/                "mobile data traces": GPS trajectory clustering / stop detection
  utility/            Typedef.hpp, Measurement, queue wrappers, progress bar, TypeTraits
  cartography/        pure Python: OSM -> edges.csv/nodes.csv via osmnx/geopandas
test/                 doctest C++ suites (base, geometry, mdt, mobility) + pytest (bindings, cartography)
benchmark/            Bench_*.cpp micro-benchmarks (opt-in, -DDSF_BENCHMARKS=ON)
examples/             mostly deprecated C++/Python demos; see sample_sim_config.json
```

Namespaces mirror directories: `dsf::` (base, utility), `dsf::mobility::`,
`dsf::geometry::`, `dsf::mdt::`.

## Layering

```
          TrafficSimulator            (mobility/TrafficSimulator.*)   config + I/O + run loop
                 |  owns unique_ptr
          FirstOrderDynamics          (mobility/FirstOrderDynamics.*) agents, ODs, evolve()
                 |  is-a
          Dynamics<network_t>         (base/Dynamics.hpp)             RNG, TBB arena, time step
                 |  owns unique_ptr
          RoadNetwork                 (mobility/RoadNetwork.*)        importers, TL init, lanes
                 |  is-a
          Network<RoadJunction, Street> (base/Network.hpp)            Dijkstra, betweenness, paths
```

`base/` knows nothing about traffic. `Network<node_t, edge_t>` is a header-only template
constrained by `requires(is_base_of_v<Node, node_t> && is_base_of_v<Edge, edge_t>)`; it
stores `unordered_map<Id, unique_ptr<...>>` for nodes and edges and provides the
graph algorithms. `RoadNetwork` is the only instantiation in-tree.

## Class hierarchy

```
Node  (base/Node.hpp)                      Edge (base/Edge.hpp)
 └── RoadJunction                           └── Road  (length, lanes, maxSpeed, forbiddenTurns, status)
      ├── Intersection                           └── Street (exit queues, moving agents, lane mapping, coil)
      │    └── TrafficLight  (final)
      ├── Roundabout
      └── Station
```

- `Node` carries id, optional `geometry::Point`, name, in/out edge id lists and a
  `variant<monostate,bool,int64_t,double,string>` attribute bag. `Edge` carries id, a
  `pair<Id,Id>` node pair (u -> v), a `geometry::PolyLine`, an angle and the same attribute bag.
  Arbitrary extra CSV columns / JSON fields land in those attribute bags with type inference.
- `Intersection` orders waiting agents in a `multimap<int16_t, unique_ptr<Agent>>` keyed by
  turn-angle delta, plus a `set<Id>` of priority streets.
- `TrafficLight` is a state machine over `vector<TrafficLightPhase>`; a phase = duration in
  ticks + green set `unordered_map<Id streetId, unordered_set<Direction>>`. Streets absent
  from the green set are red. `operator++` advances one tick. `Direction::ANY` is the
  catch-all entry produced by auto-deduction; `TrafficLight::isGreen()` applies the
  direction-fallback ladder, `TrafficLightPhase::containsGreen()` does not.
- `Street` holds one `dsf::queue<unique_ptr<Agent>>` **per lane** (`m_exitQueues`) plus a
  `dsf::priority_queue` of moving agents ordered by `freeTime()`. `m_laneMapping` assigns a
  `Direction` to each lane. An optional `Counter` ("coil") sensor counts passages at
  ENTRY / MIDDLE / EXIT.

Agents are owned by `unique_ptr` and physically *moved* between containers
(street queue -> node multimap -> next street). There is no central agent registry
holding live references; `FirstOrderDynamics::m_agents` holds agents not yet inserted.

## The simulation tick — `FirstOrderDynamics::evolve()`

Defined in [FirstOrderDynamics.cpp](../src/dsf/mobility/FirstOrderDynamics.cpp). Three
TBB `parallel_for` passes over `m_nodeIndices`, in this order:

1. **Streets** — for each node, for each ingoing edge: optionally accumulate traffic-light
   queue statistics, then `m_evolveStreet()` (advance moving agents, push arrivals into the
   correct lane exit queue). Per-street stats (density, mean speed, queue length, coil
   counts) are collected here when the `StepDataRequest` asks for them.
2. **Nodes** — `m_evolveNode()` moves up to `transportCapacity` agents out of each junction
   into their next street, honouring priorities, forbidden turns and traffic-light greens;
   then `++trafficLight` advances the phase state machine.
3. **Agents** — `m_evolveAgents()` updates speeds/positions of agents in transit.

`evolve()` takes a `StepDataRequest` (which of avg-stats / street / travel / agent /
turn-count data to collect) and returns a `StepDataResult` carrying the collected records
in TBB concurrent containers. Nothing is written to disk here — `TrafficSimulator` does that.

Routing: each agent follows an `Itinerary` (destination + a `PathCollection` of next hops).
`updatePaths()` recomputes them; the cadence is `update_paths.interval` in the config.
Path search runs on the **dual graph** (edge-to-edge) via
`RoadNetwork::allEdgePathsTo()` so that turn restrictions and u-turn penalties apply.
Hop graphs are kept acyclic by only retaining transitions that strictly decrease the
precomputed distance-to-target, which is what makes `PathCollection::explode()` terminate.

## Agent insertion

`AgentInsertionMethod`: `RANDOM`, `UNIFORM`, `ODS`, `RANDOM_ODS`, `CONDITIONAL_RANDOM_ODS`.
Origins/destinations are set via `setOrigins/setDestinations/setODs/setConditionalODs` or
`importODsFromCSV`. `TrafficSimulator` additionally supports time-phased OD swaps through
the `general.dynamic_ods` config array.

## Concurrency

`Dynamics` owns a `tbb::task_arena` plus a `tbb::global_control`; `setConcurrency(n)` caps
both so the OS does not spawn a thread per hardware core. `DSF_EXECUTION` in
[Typedef.hpp](../src/dsf/utility/Typedef.hpp) expands to `std::execution::par_unseq,`
everywhere except Apple, where parallel STL is unavailable and it expands to nothing.

## The `mdt` module (independent of mobility)

`TrajectoryCollection` ingests GPS pings (CSV or a column-oriented dataframe map), groups
them per user id into `Trajectory` objects, and `filter()` performs radius-based spatial
clustering (`PointsCluster`, centroid = coordinate-wise median) with a max-speed sanity
check to detect stops and split trajectories. It shares only `geometry::Point` with the
traffic side. Bindings can hand results back as pandas or polars frames.
