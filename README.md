# DynamicalSystemFramework
[![Latest Release](https://img.shields.io/github/v/release/Grufoony/DynamicalSystemFramework)](https://github.com/Grufoony/DynamicalSystemFramework/releases/latest)
[![PyPI version](https://img.shields.io/pypi/v/dsf-suite)](https://pypi.org/project/dsf-suite/)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18745492.svg)](https://doi.org/10.5281/zenodo.18745492)

[![Standard](https://img.shields.io/badge/C%2B%2B-20/23-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B#Standardization)
[![TBB](https://img.shields.io/badge/TBB-2022.3.0-blue.svg)](https://github.com/oneapi-src/oneTBB)
[![SPDLOG](https://img.shields.io/badge/spdlog-1.17.0-blue.svg)](https://github.com/gabime/spdlog)
[![CSV](https://img.shields.io/badge/csv_parser-5.1.0-blue.svg)](https://github.com/vincentlaucsb/csv-parser)
[![JSON](https://img.shields.io/badge/simdjson-4.6.1-blue.svg)](https://github.com/simdjson/simdjson)
[![SQLite](https://img.shields.io/badge/SQLiteCpp-3.3.3-blue.svg)](https://github.com/SRombauts/SQLiteCpp)
[![codecov](https://codecov.io/gh/Grufoony/DynamicalSystemFramework/graph/badge.svg?token=JV53J6IUJ3)](https://codecov.io/gh/Grufoony/DynamicalSystemFramework)

The aim of this project is to rework the original [Traffic Flow Dynamics Model](https://github.com/Grufoony/TrafficFlowDynamicsModel).
This rework consists of a full code rewriting, in order to implement more features (like *intersections*) and get advantage from the latest C++ updates.

## Table of Contents
- [Installation](#installation)
- [Installation (from source)](#installation-from-source)
- [Testing](#testing)
- [Benchmarking](#benchmarking)
- [Citing](#citing)
- [Bibliography](#bibliography)

## Installation
The library is available on `PyPI`:
```shell
pip install dsf-suite
```

To check the installation you can simply run
```python
import dsf

print(dsf.__version__)
```

## Installation (from source)

### Requirements
The project requires `C++20` or greater, `cmake`, `tbb` `simdjson`, `spdlog`, `csv-parser` and `SQLiteCpp`.
To install requirements on Ubuntu:
```shell
sudo apt install cmake libtbb-dev
```
To install requirements on macOS:
```shell
brew install cmake tbb
```

Utilities are written in python. To install their dependencies:
```shell
pip install -r ./requirements.txt
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
then building and installing it (eventually in sudo mode) with:
```shell
cmake --build build
cmake --install build
```

## Installation (Python)
If you want to use the library from Python, you can build the Python bindings using [pybind11](https://github.com/pybind/pybind11). Make sure you have Doxygen installed to generate the docstrings:
```shell
sudo apt install doxygen libtbb-dev
```

Then, the installation is automatic via `uv`:
```shell
uv build
```
or you can just use the classic `pip`:
```shell
pip install .
```

After installation, you should be able to import the module in Python:
```python
import dsf

print(dsf.__version__)
```

If you encounter issues, ensure that the installation path is in your `PYTHONPATH` environment variable.

## Testing
This project uses [Doctest](https://github.com/doctest/doctest) for testing.

If the project is compiled in `Debug` or `Coverage` mode, tests are always built.
Otherwise, you can add the `-DDSF_TESTS=ON` flag to enable test build.
```shell
cmake -B build -DDSF_TESTS=ON
cmake --build build -j$(nproc)
```

To run the tests use the command:
```shell
ctest --test-dir build -j$(nproc) --output-on-failure
```

## Benchmarking
Some functionalities of the library have been benchmarked in order to assess their efficiency.  
The benchmarks are performed using [Google Benchmarks](https://github.com/google/benchmark).
To build the benchmarks add the flag `-DDSF_BENCHMARKS=ON` :
```shell
cmake -B build -DDSF_BENCHMARKS=ON
cmake --build build -j$(nproc)
```
To run all the benchmarks together use the command:
```shell
cd benchmark
for f in ./*.out ; do ./$f ; done
```

## Citing

```BibTex
@software{berselli_2026_18745492,
  author       = {Berselli, Gregorio},
  title        = {DynamicalSystemFramework},
  month        = mar,
  year         = 2026,
  publisher    = {Zenodo},
  doi          = {10.5281/zenodo.18745492},
  url          = {https://doi.org/10.5281/zenodo.18745492},
}
```

## Bibliography
- **Mungai, Veronica** (2024) *Studio dell'ottimizzazione di una rete semaforica*. University of Bologna, Bachelor's Degree in Physics [L-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/32525/).
- **Berselli, Gregorio** (2024) *Advanced queuing traffic model for accurate congestion forecasting and management*. University of Bologna, Master's Degree in Physics [LM-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/32191/).
- **Berselli, Gregorio** (2022) *Modelli di traffico per la formazione della congestione su una rete stradale*. University of Bologna, Bachelor's Degree in Physics [L-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/26332/).