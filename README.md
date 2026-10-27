# DynamicalSystemFramework
[![Latest Release](https://img.shields.io/github/v/release/Grufoony/DynamicalSystemFramework)](https://github.com/Grufoony/DynamicalSystemFramework/releases/latest)
[![PyPI version](https://img.shields.io/pypi/v/dsf-suite)](https://pypi.org/project/dsf-suite/)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18745492.svg)](https://doi.org/10.5281/zenodo.18745492)

[![Standard](https://img.shields.io/badge/C%2B%2B-20/23-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B#Standardization)
[![TBB](https://img.shields.io/badge/TBB-2022.3.0-blue.svg)](https://github.com/oneapi-src/oneTBB)
[![SPDLOG](https://img.shields.io/badge/spdlog-1.17.0-blue.svg)](https://github.com/gabime/spdlog)
[![CSV](https://img.shields.io/badge/csv_parser-5.4.0-blue.svg)](https://github.com/vincentlaucsb/csv-parser)
[![JSON](https://img.shields.io/badge/simdjson-5.0.0-blue.svg)](https://github.com/simdjson/simdjson)
[![SQLite](https://img.shields.io/badge/SQLiteCpp-3.4.0-blue.svg)](https://github.com/SRombauts/SQLiteCpp)
[![codecov](https://codecov.io/gh/Grufoony/DynamicalSystemFramework/graph/badge.svg?token=JV53J6IUJ3)](https://codecov.io/gh/Grufoony/DynamicalSystemFramework)

The aim of this project is to rework the original [Traffic Flow Dynamics Model](https://github.com/Grufoony/TrafficFlowDynamicsModel).
This rework consists of a full code rewriting, in order to implement more features (like *intersections*) and get advantage from the latest C++ updates.

## Table of Contents
- [Installation](#installation)
- [Installation (from source)](#installation-from-source)
- [Examples](#examples)
- [Visualization & GUI](#visualization--gui)
- [Projects Using DSF](#projects-using-dsf)
- [For Developers](#for-developers)
  - [Testing](#testing)
  - [Benchmarking](#benchmarking)
- [Citing](#citing)
- [Bibliography](#bibliography)

## Installation
The library is available on `PyPI`:
```shell
pip install dsf-suite
```

To check the installation you can simply run:
```python
import dsf

print(dsf.__version__)
```

## Installation (from source)

### Requirements
The project requires `C++20` or greater, `cmake`, `tbb`, `simdjson`, `spdlog`, `csv-parser`, and `SQLiteCpp`.

To install requirements on Ubuntu:
```shell
sudo apt install cmake libtbb-dev
```
To install requirements on macOS:
```shell
brew install cmake tbb
```

### Installation (C++)
The library can be installed using CMake. To build and install the project in the default folder run:
```shell
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
sudo cmake --install build
```
Otherwise, it is possible to customize the installation path:
```shell
cmake -B build -DCMAKE_INSTALL_PREFIX=/path/to/install
```
then building and installing it with:
```shell
cmake --build build
cmake --install build
```

### Installation (Python)
If you want to use the library from Python, you can build the Python bindings using [nanobind](https://github.com/wjakob/nanobind). Make sure you have `tbb` installed:
```shell
sudo apt install libtbb-dev
```

Then, the installation is automatic via `uv`:
```shell
uv build
```
or via standard `pip`:
```shell
pip install .
```

After installation, verify the import in Python:
```python
import dsf

print(dsf.__version__)
```

If you encounter issues, ensure that the installation path is in your `PYTHONPATH` environment variable.

## Examples
Examples in this repository are being migrated. You can build legacy C++ examples by passing `-DDSF_EXAMPLES=ON` to CMake; output binaries will be generated under the `examples/` directory.

Updated and active examples are maintained in **[dsf-examples](https://github.com/Grufoony/dsf-examples)**.

## Pedestrian Mode

Pedestrian mode uses the same edge-based, first-order dynamics as the traffic model. A network is a directed graph: edges represent street segments and nodes represent intersections or roundabouts. Enabling pedestrian mode does **not** replace the queue-based movement model with a continuous pedestrian flow model. Streets retain their per-lane FIFO exit queues, moving-agent priority queue, finite storage capacity, and transport capacity.

### What pedestrian mode changes

When the network is prepared in pedestrian mode, the framework skips automatic lane mapping, automatic road-priority assignment, and automatic traffic-light initialization. It also disables stochastic route deviations (`errorProbability` rerouting). Agents therefore follow their itinerary's shortest-path successors, subject to the network's available paths and any explicitly configured turn restrictions. Existing junction mechanics and any explicitly imported or configured junction controls still govern transfers.

Pedestrian mode does not automatically reinterpret input speeds, capacities, or lengths as pedestrian values. The network's edge `maxspeed` is still used (with the normal import conversion from km/h to m/s), and the default capacity is still calculated using the framework's mean-vehicle-length setting. The default speed function remains linear with $\alpha = 0.8$. A pedestrian application should provide pedestrian speeds and appropriate street capacities/transport capacities in its network or configure them through the API.

### State and street capacity

At simulation step $t$, an agent occupies one of three places: the list of agents waiting to enter, a street's moving population, or a street exit/junction queue. A street with length $L$, $n$ lanes, and no explicit capacity has storage capacity

$$
C = \left\lceil \frac{L \, n}{\ell} \right\rceil,
$$

where $\ell$ is the configured mean vehicle length (5 m by default). A street is full when its moving and queued agents together reach $C$. Its moving density, used by the default speed rule, is the moving-agent count divided by $C$; agents already waiting in exit queues contribute delay at the junction rather than slowing moving agents directly.

With the default linear speed function,

$$
v_e(t) = v^{\max}_e \left(1 - \alpha \, \rho^{\mathrm{moving}}_e(t)\right),
\qquad \alpha = 0.8,
$$

where $v^{\max}_e$ is the edge's imported/configured maximum speed and $\rho^{\mathrm{moving}}_e$ is its normalized moving density. A user-selected constant or custom speed function can replace this rule. When an agent enters an edge, its free-flow completion step is set to

$$
t_{\mathrm{free}} = t + \left\lceil \frac{L_e}{v_e(t)} \right\rceil.
$$

The moving population is held in a priority queue ordered by this completion time. Once an agent's free time is reached, it joins one of the street's exit queues. With multiple lanes, the model chooses a lane compatible with the next turn where possible, favoring shorter exit queues; a single-lane street uses its only queue.

### Junction service and route progression

Each step, the model first processes streets and attempts to move eligible agents from street queues into their destination nodes. The queue is FIFO, so its front agent is considered first. A transfer is blocked when the node is full, the next street is full, or an applicable traffic-light/priority rule denies passage. Street and node transport capacities limit service. On a street, each lane gets one service attempt per step; a fractional capacity $c$ between 0 and 1 permits passage with probability $c$. At a node, the model makes $\lceil c \rceil$ service attempts, with the final attempt probabilistic when $c$ is fractional, so capacities above 1 can permit multiple agents per step.

At an intersection, the agent enters the junction's ordered internal queue and, when service is available and its next edge has room, moves onto that edge. Roundabouts use their own FIFO internal queue. The selected edge is the next edge on the agent's itinerary. The itinerary advances when the agent reaches the edge that terminates the current destination; the agent is removed after reaching the final destination. This is a discrete-time, agent-based queueing process, not a microscopic model of pedestrian body motion or collision avoidance.

### Origins, destinations, and insertion

Origins and destinations are edge IDs for the standard edge-based insertion methods. `setOrigins` assigns the edge weights used to choose starting edges; `setDestinations` creates itineraries whose destination is the corresponding edge. `RANDOM_ODS` independently samples an origin and destination according to their normalized weights, while `ODS` uses explicit `(origin edge, destination edge, weight)` tuples. For an inserted agent, its source node is the source endpoint of its selected origin edge, and that edge is its first edge in the itinerary. Entry may be delayed while the source node or edge has no capacity.

The `TrafficSimulator` insertion method defaults to `RANDOM`. In that mode, calls to `setOrigins` and `setDestinations` alone do not constrain the inserted agents; select an OD insertion method explicitly when those distributions are intended to drive the run. The number of agents offered at each insertion time is supplied by the simulation schedule.

### Per-step order

One call to `evolve()` performs the following high-level updates:

1. Process street travel completions and attempt street-queue transfers to junctions.
2. Process junction queues and move agents onto their next edges when capacity and service rules allow.
3. Admit waiting agents from their source nodes to their first edges when there is room.
4. Advance simulation time and, when requested, collect street, agent, travel, queue, and aggregate statistics.

The ordering is consequential: an agent's movement is constrained by discrete service opportunities and available storage, and congestion can propagate upstream as queues fill.

## Visualization & GUI
For real-time dynamic visualization and visual inspection of network flows, see **[dsf-gui](https://github.com/Grufoony/dsf-gui)**. It offers an interactive front-end designed specifically for monitoring simulations, analyzing queue dynamics, and visual debugging.

## Projects Using DSF
- **[bdt-mobility](https://github.com/Gemello-Digitale-Bologna/bdt-mobility)** — *Bologna Digital Twin Mobility Toolkit*: Data extraction pipeline and policy-impact traffic simulation framework built for defined Zones of Interest (ZOI).

## For Developers
This project includes a `pre-commit` hook setup.
To enable it locally, run:
```shell
pip install pre-commit
pre-commit install
```

### Testing
This project uses [Doctest](https://github.com/doctest/doctest) for unit testing.

Tests are built automatically in `Debug` or `Coverage` mode. To manually force test compilation:
```shell
cmake -B build -DDSF_TESTS=ON
cmake --build build -j$(nproc)
```

To execute the test suite:
```shell
ctest --test-dir build -j$(nproc) --output-on-failure
```

### Benchmarking
Core performance critical components are benchmarked using [Google Benchmark](https://github.com/google/benchmark).

To build benchmark targets:
```shell
cmake -B build -DDSF_BENCHMARKS=ON
cmake --build build -j$(nproc)
```
To run all benchmark executables:
```shell
cd benchmark
for f in ./*.out ; do ./$f ; done
```

## Citing
If you use this software, please cite it using the metadata in [CITATION.cff](CITATION.cff)
(GitHub's "Cite this repository" button exports it as BibTeX/APA), or via Zenodo DOI:
[10.5281/zenodo.18745492](https://doi.org/10.5281/zenodo.18745492).

## Bibliography
- **Mungai, Veronica** (2024) *Studio dell'ottimizzazione di una rete semaforica*. University of Bologna, Bachelor's Degree in Physics [L-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/32525/).
- **Berselli, Gregorio** (2024) *Advanced queuing traffic model for accurate congestion forecasting and management*. University of Bologna, Master's Degree in Physics [LM-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/32191/).
- **Berselli, Gregorio** (2022) *Modelli di traffico per la formazione della congestione su una rete stradale*. University of Bologna, Bachelor's Degree in Physics [L-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/26332/).
