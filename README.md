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