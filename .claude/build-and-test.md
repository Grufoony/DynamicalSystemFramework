# Build, test and lint

## Requirements

C++20 compiler, CMake >= 3.16, and **TBB installed system-wide** (`libtbb-dev` on Debian/
Ubuntu, `brew install tbb` on macOS, `vcpkg install tbb:x64-windows` on Windows).
Everything else — csv-parser 5.3.0, spdlog 1.17.0, simdjson 4.6.8, SQLiteCpp 3.3.3,
doctest 2.5.3, nanobind 2.15.0 — is pulled by `FetchContent` at configure time, so the
first configure needs network access and takes a while.

## C++

```bash
# Debug (default when no build type given): -Wall -Wextra -Wpedantic -Werror -g -pg
# plus -fsanitize=address on GCC. Also forces DSF_TESTS=ON.
cmake -B build && cmake --build build -j$(nproc)

# Release: -Ofast -flto=auto -march=native
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
sudo cmake --install build

# Coverage (what CI uses on Ubuntu): forces DSF_TESTS=ON, adds --coverage
cmake -B build -DCMAKE_BUILD_TYPE=Coverage && cmake --build build -j$(nproc)
```

`-DDSF_COMPATIBILITY_BUILD=ON` drops `-march=native`/`-Ofast` down to plain `-O3`; this is
what the wheel builds use so binaries stay portable.

Options: `DSF_TESTS`, `DSF_EXAMPLES`, `DSF_BENCHMARKS`, `DSF_BUILD_PIC`,
`BUILD_PYTHON_BINDINGS`, `DSF_COMPATIBILITY_BUILD` (all default `OFF`; `DSF_TESTS` is
forced `ON` in Debug and Coverage).

Warnings are errors in Debug and Coverage. Do not leave an unused parameter — mark it
`[[maybe_unused]]`, which is the existing idiom.

### Running C++ tests

Test executables land in `build/tests/`, one per `test/*/*.cpp` file (the CMake glob names
the target after the file stem).

```bash
ctest --test-dir build --output-on-failure -j$(nproc)
ctest --test-dir build -R Test_graph --output-on-failure   # one suite
./build/tests/Test_graph                                   # directly; doctest flags work
./build/tests/Test_graph -tc="*roundabout*"                # single test case
```

Suites use doctest with `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`, so each file is its own
binary with its own `main`. They locate fixtures relative to their own source path:
`std::filesystem::path(__FILE__).parent_path().parent_path() / "data"` → [test/data/](../test/data/).
That means they can be run from any working directory.

## Python

```bash
pip install -e .          # builds the C++ core + nanobind module (dsf_cpp) in one step
# or
uv build

pytest test -v -o "python_files=Test_*.py test_*.py Bind_*.py"
```

The non-standard `python_files` override is required — test files are named `Test_*.py`,
which pytest does not collect by default. CI passes it on the command line; there is no
`[tool.pytest.ini_options]` section in `pyproject.toml`.

Python tests live in [test/bindings/](../test/bindings/) (shared fixtures in
`conftest.py`, built on the Manhattan network in `test/data/`) and
[test/Test_cartography.py](../test/Test_cartography.py).

## Lint / format

```bash
clang-format --version              # must be 18 to match CI
pre-commit run --all-files          # clang-format on src|test|examples, ruff format + ruff --fix
ruff format --check $(git ls-files '*.py')
ruff check $(git ls-files '*.py' | grep -v 'src/dsf/__init__.py')
```

`src/dsf/__init__.py` is excluded from `ruff check` (its re-export shim trips F401).
`.clang-format`: Google base, 90-column limit, 2-space indent, namespaces indented,
`SortIncludes: false`, `ReflowComments: false`, left-aligned pointers and references.

## CI (`.github/workflows/`)

| Workflow | What it gates |
| --- | --- |
| `cmake_tests.yml` | Build + ctest on Ubuntu (Coverage) / macOS (Debug) / Windows (Debug, tests currently commented out); uploads lcov to Codecov |
| `pytest.yml` | `pip install -e .` then pytest on Python 3.12 and 3.14 |
| `binding.yml` | Wheel/sdist/editable install matrix across 3 OSes × 2 Python versions |
| `clang_format.yml` | clang-format 18 check over `src`, `test`, `examples` |
| `ruff.yml` | ruff format + check (pinned `ruff>=0.15,<0.16`) |
| `codeql.yml`, `flawfinder.yml` | Security scanning |
| `gh-pages.yml` | Doxygen docs to GitHub Pages |
| `pypi.yml`, `zenodo.yml`, `benchmark_release.yml` | Release automation |

All PR-triggered workflows skip draft PRs. `[skip ci]` in a commit subject is used in this
repo's history for doc-only changes.

## Benchmarks and docs

```bash
cmake -B build -DDSF_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release   # builds benchmark/Bench_*.cpp
doxygen Doxyfile                                                # INPUT = ./src/dsf + README.md -> docs/
```

Doxygen styling comes from the `extern/doxygen-awesome` submodule — run
`git submodule update --init` before generating docs.
