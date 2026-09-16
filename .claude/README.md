# Repo guide for AI agents

Orientation notes for **DynamicalSystemFramework (DSF)** — a C++20 agent-based
traffic/mobility simulation library with Python bindings, published as
`dsf-suite` on PyPI.

Read these in order of need:

| File | Use it when |
| --- | --- |
| [architecture.md](architecture.md) | You need the module map, class hierarchy, or the simulation tick order. **Start here.** |
| [api-reference.md](api-reference.md) | You need the public API of a specific class without opening the header. |
| [build-and-test.md](build-and-test.md) | You need to compile, run tests, lint, or reproduce CI. |
| [data-formats.md](data-formats.md) | You touch importers/exporters: edge/node CSV, GeoJSON, the JSON sim config, SQLite/CSV outputs. |
| [conventions.md](conventions.md) | You are writing or reviewing code here (style, naming, error handling, where things go). |

## 30-second summary

- **What it does**: builds a road network from CSV/JSON/GeoJSON, spawns agents on it,
  evolves them one time-step at a time through streets, intersections, traffic lights and
  roundabouts, and writes per-step metrics to SQLite or CSV.
- **Language split**: all simulation logic is C++ under [src/dsf/](../src/dsf/).
  Python is a thin layer — nanobind bindings ([src/dsf/bindings.cpp](../src/dsf/bindings.cpp))
  plus one pure-Python module, [cartography](../src/dsf/cartography/cartography.py),
  which pulls OSM data via `osmnx` and emits the CSVs the C++ side imports.
- **Version** lives in [src/dsf/dsf.hpp](../src/dsf/dsf.hpp) as
  `DSF_VERSION_MAJOR/MINOR/PATCH`; CMake regex-parses it. Bumping the version means
  editing that header, nothing else.
- **Entry points**: `dsf::mobility::TrafficSimulator` (config-file-driven, batteries
  included) wraps `dsf::mobility::FirstOrderDynamics` (the engine) which owns a
  `dsf::mobility::RoadNetwork`.

## Things that surprise people

- `Edge` and `Street` are **move-only** (copy ctor is `= delete`). Networks store
  `unique_ptr` to nodes/edges and are themselves move-only.
- The Python class `dsf.mobility.Dynamics` is C++ `FirstOrderDynamics` — the names differ.
- `Network::edge(id)` returns the base type; use the templated
  `edge<Street>(id)` / `node<TrafficLight>(id)` overloads to downcast (they `dynamic_cast`).
- Node/edge "types" are conversions, not constructors: `makeTrafficLight(id)`,
  `makeRoundabout(id)`, `makeStation(id, t)` replace an existing node in place.
- Debug builds enable `-Werror -fsanitize=address` (GCC) — warnings fail the build.
- `docs/`, `xml/`, `build/`, `cache/` and `examples/*.csv` are generated and
  git-ignored despite being present in the working tree.
